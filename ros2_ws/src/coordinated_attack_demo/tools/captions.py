#!/usr/bin/env python3
# Copyright 2026 Haniel Vásquez Morales
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Reads one recorded floor of the coordinated attack into the timeline its part
of the video is captioned from.

    captions.py --log run.log --t0 <capture start, unix s> [--out timeline.json]

Every caption is a line the run wrote, placed at the instant it was written:
what the work order said, when a message went out and arrived, who had the
beacon in line of sight, how far apart the two robots' starts were, what the
epistemic state said the model was after each update, and what
knowledge_view computed from that model. Nothing here decides what happened.
What is added is the notation.

The timeline has three parts:

  states    what the panel under the video shows from a given instant: the
            step in formal notation, the knowledge column, the policy node
  segments  the playback speed over each stretch of the capture
  facts     the run's figures, for the closing card

Formulas use a small markup the renderer understands: `_{...}` is a subscript
and `^{...}` a superscript.
"""

import argparse
import json
import re

LINE = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)')


def sub(name):
    """s1 -> s_{1}."""
    m = re.match(r'([a-z]+)(\d+)$', name)
    return f'{m.group(1)}_{{{m.group(2)}}}' if m else name


def job(stand):
    return f'job({sub(stand)})'


def k(agent):
    return f'K_{{{agent}}}'


def nested(text):
    """'K_south K_north job(s1)' -> 'K_{south} K_{north} job(s_{1})'."""
    out = re.sub(r'K_(\w+)', lambda m: k(m.group(1)), text)
    return re.sub(r'job\((\w+)\)', lambda m: job(m.group(1)), out)


def depth_text(depth, stand):
    if depth == 'C':
        return f'C_{{south,north}} {job(stand)}'
    return f'E^{{{depth}}} {job(stand)}'


def parse(log, t0):
    out = []
    with open(log, errors='replace') as fh:
        for raw in fh:
            m = LINE.search(raw)
            if m:
                out.append((float(m.group(1)) - t0, m.group(2), m.group(3).strip()))
    out.sort(key=lambda e: e[0])
    return out


def knows_from(line):
    """A [knows] line into the knowledge column."""
    head, _, body = line.partition(': ')
    shape = re.match(r'\[knows\] (\d+) worlds, (\d+) designated', head)
    out = {'worlds': int(shape.group(1)) if shape else 0,
           'designated': int(shape.group(2)) if shape else 0,
           'stand': None, 'knows': {}, 'depth': None, 'chain': []}
    for part in body.split(' · '):
        part = part.strip()
        m = re.match(r'K_(\w+) (~?)job_(\w+)', part)
        if m:
            out['knows'][m.group(1)] = not m.group(2)
            out['stand'] = m.group(3)
            continue
        m = re.match(r'depth job_(\w+) (C|E\^(\d+))', part)
        if m:
            out['stand'] = m.group(1)
            out['depth'] = 'C' if m.group(2) == 'C' else int(m.group(3))
            continue
        m = re.match(r'chain (.*)', part)
        if m:
            out['chain'] = m.group(1).split('>')
    return out


def build(log, t0, floor):
    events = parse(log, t0)
    states, facts, marks = [], {'floor': floor, 'messages': []}, {}
    know = {'worlds': 2, 'designated': 2, 'stand': None, 'knows': {}, 'depth': None, 'chain': []}
    node = None
    step = {
        'label': 'INITIAL MODEL',
        'title': 'M_{0}:  2 worlds, both designated,  w_{k} ⊨ job(s_{k})',
        'gloss': 'the work order names exactly one stand, and that much is common knowledge; '
                 'which one, only south can find out',
        'lines': ['s ⊨ C_{south,north} (job(s_{1}) ⊕ job(s_{2}))',
                  's ⊨ ¬Kw_{i} job(s_{k})   for both robots and both stands',
                  'lift(s)  requires  job(s) ∧ C_{south,north} job(s)'],
    }

    def emit(t, **changes):
        nonlocal step, know, node
        step = changes.get('step', step)
        know = changes.get('know', know)
        node = changes.get('node', node)
        states.append({'t': round(max(t, 0.0), 2), 'step': json.loads(json.dumps(step)),
                       'know': json.loads(json.dumps(know)), 'node': node})

    def add(t, text):
        step['lines'].append(text)
        emit(t)

    emit(0.0)
    applied_since = None
    last_applied = None

    for t, _who, msg in events:
        if msg.startswith('[knows]'):
            new = knows_from(msg)
            if new['stand'] is None:
                continue
            old_depth = know.get('depth')
            emit(t, know=new)
            # Say the depth once it is known after an update.
            if applied_since is not None and new['depth'] is not None and new['depth'] != old_depth:
                s = new['stand']
                if new['depth'] == 'C':
                    add(t, f's ⊨ {depth_text("C", s)}:  common knowledge, in one update')
                elif new['depth'] == 0:
                    who = [a for a, v in new['knows'].items() if v]
                    add(t, f's ⊨ {k(who[0]) if who else "K"} {job(s)},   s ⊭ E^{{1}} {job(s)}')
                else:
                    d = new['depth']
                    add(t, f's ⊨ E^{{{d}}} {job(s)},   s ⊭ E^{{{d + 1}}} {job(s)}')
                if new['chain']:
                    chain = ' considers '.join(new['chain'])
                    add(t, f'C fails: {chain} considers a world where ¬{job(s)}')
                applied_since = None
            continue

        m = re.match(r'\[epistemic_state\] applied ([\w-]+?)(?: -> ([\w-]+))?: (\d+) worlds, '
                     r'(\d+) designated', msg)
        if m:
            action, ev, w, d = m.groups()
            # The state reports an update once and the launch can echo it; a
            # product update is applied once, and is captioned once.
            if action == last_applied:
                continue
            last_applied = action
            applied_since = t
            add(t, f'M ⊗ E:  {w} worlds, {d} designated')
            if action.startswith('lift_'):
                facts['final_worlds'] = int(w)
            continue

        m = re.match(r'\[mission\] policy with (\d+) nodes, (\d+) leaves, in ([\d.]+) s', msg)
        if m:
            facts['policy'] = {'nodes': int(m.group(1)), 'leaves': int(m.group(2)),
                               'seconds': float(m.group(3))}
            marks['policy'] = t
            emit(t, step={
                'label': 'PLANNER',
                'title': f'π:  {m.group(1)} nodes,  {m.group(2)} leaves,  found in {m.group(3)} s',
                'gloss': 'AO* over the grounded task; the policy branches on what the work '
                         'order says',
                'lines': ['the radio is in the domain and the policy does not use it',
                          'both robots go to the viewpoints of the beacon in t_{2}']})
            continue

        m = re.match(r'\[mission\] no policy for lifted: the planner returned none after ([\d.]+) s', msg)
        if m:
            facts['no_policy_seconds'] = float(m.group(1))
            marks['policy'] = t
            emit(t, step={
                'label': 'PLANNER',
                'title': 'no policy for  lifted',
                'gloss': 'AO* exhausts the space of the radio floor: no sequence of four '
                         'messages makes the lift applicable',
                'lines': [f'the planner answered in {m.group(1)} s']})
            continue

        if msg.startswith('[mission] running the radio protocol instead'):
            add(t, 'run instead: read the order, then tell, ack, ack2, ack3, then lift')
            continue

        m = re.match(r'\[order\] (\w+) sets out for the work-order terminal', msg)
        if m:
            who = m.group(1)
            marks.setdefault('drive', []).append(t)
            emit(t, node=None, step={
                'label': 'SENSING ACTION',
                'title': f'read-order({who}, s_{{1}})',
                'gloss': f'semi-private sensing: {who} learns whether the order names s_{{1}}; '
                         f'north learns only that it was read',
                'lines': ['events  e-here: job(s_{1})     e-elsewhere: ¬job(s_{1})']})
            node = f'read-order_{who}_s1'
            continue

        m = re.match(r'\[order\] (\w+) at the terminal after (\d+) s', msg)
        if m:
            marks.setdefault('arrive', []).append(t)
            continue

        m = re.match(r'\[order\] (\w+) reads the work order: it names (\w+); asked about (\w+) -> ([\w-]+)',
                     msg)
        if m:
            who, named, asked, ev = m.groups()
            facts['order'] = named
            add(t, f'the order names {named}  ⇒  {ev}')
            continue

        m = re.match(r'\[radio\] (\w+)\((\w+), (\w+), (\w+)\): \w+ sends "([^"]*)" to \w+', msg)
        if m:
            kind, i, j, s, content = m.groups()
            marks.setdefault('radio', []).append(t)
            emit(t, node=f'{kind}_{i}_{j}_{s}', step={
                'label': 'LOSSY MESSAGE',
                'title': f'{kind}({i}, {j}, {sub(s)})',
                'gloss': 'delivered, lost, or never sent: the sender cannot tell delivered from '
                         'lost, the receiver cannot tell lost from never sent',
                'lines': [f'content:  {nested(content)}']})
            continue

        m = re.match(r'\[radio\] (\w+)\((\w+), (\w+), (\w+)\) delivered to (\w+); (\w+) cannot tell', msg)
        if m:
            kind, i, j, s = m.group(1), m.group(2), m.group(3), m.group(4)
            facts['messages'].append(kind)
            add(t, f'delivered to {j};  {i} cannot tell whether it was')
            continue

        m = re.match(r'\[[^\]]*\] knowledge requirement does not hold: \(C \((\w+) (\w+)\) job_(\w+)\)',
                     msg)
        if m:
            s = m.group(3)
            facts['refused'] = s
            marks['refused'] = t
            emit(t, node=f'lift_{s}', step={
                'label': 'JOINT ONTIC ACTION',
                'title': f'lift({sub(s)})      pre:  {job(s)} ∧ C_{{south,north}} {job(s)}',
                'gloss': 'the executor checks the precondition against the model before it '
                         'dispatches the action',
                'lines': [f'C_{{south,north}} {job(s)} does not hold:  lift is refused, '
                          'and neither robot moves']})
            applied_since = t
            continue

        m = re.match(r'\[view\] (\w+) sets out for its viewpoint', msg)
        if m:
            who = m.group(1)
            marks.setdefault('drive', []).append(t)
            emit(t, node=f'go-view_{who}', step={
                'label': 'PUBLIC ONTIC ACTION',
                'title': f'go-view({who})',
                'gloss': f'{who} drives to the mouth of t_{{2}} on its side; at-view({who}) '
                         'becomes common knowledge',
                'lines': []})
            continue

        m = re.match(r'\[view\] (\w+) at its viewpoint after (\d+) s; beacon in line of sight: (\w+), '
                     r'([\d.]+) m', msg)
        if m:
            who, secs, yes, dist = m.groups()
            marks.setdefault('arrive', []).append(t)
            facts.setdefault('views', {})[who] = {'seconds': int(secs), 'sees': yes == 'yes',
                                                  'distance': float(dist)}
            add(t, f'at the viewpoint after {secs} s;  the beacon in line of sight, {dist} m')
            continue

        m = re.match(r'\[signal\] (\w+) lights the (\w+) tier of the beacon; in line of sight of it: (.*)',
                     msg)
        if m:
            who, s, seen = m.groups()
            marks['signal'] = t
            facts['signal'] = {'stand': s, 'seen': seen}
            emit(t, node=f'signal_{who}_{s}', step={
                'label': 'ANNOUNCEMENT',
                'title': f'signal({who}, {sub(s)})      announces  {k(who)} {job(s)}',
                'gloss': 'an agent at a viewpoint observes it fully, any other is oblivious; '
                         'with both at their viewpoints it is public',
                'lines': [f'the {s} tier of the beacon is lit;  in line of sight of it:  {seen}']})
            continue

        m = re.match(r'\[lift\] lift\((\w+)\): both robots set out', msg)
        if m:
            s = m.group(1)
            marks['lift'] = t
            emit(t, node=f'lift_{s}', step={
                'label': 'JOINT ONTIC ACTION',
                'title': f'lift({sub(s)})      pre:  {job(s)} ∧ C_{{south,north}} {job(s)}  holds',
                'gloss': 'each robot drives to its own mouth of the stand; the load is between '
                         'them, and neither sees the other',
                'lines': []})
            continue

        m = re.match(r'\[lift\] both at (\w+), out of sight of each other; one start for both in ([\d.]+) s',
                     msg)
        if m:
            marks['lift_ready'] = t
            add(t, f'both at {m.group(1)}, out of sight of each other: one start for both')
            continue

        m = re.match(r'\[lift\] both under (\w+): starts ([\d.]+) s apart, arrivals ([\d.]+) s apart', msg)
        if m:
            facts['joint'] = {'starts': float(m.group(2)), 'arrivals': float(m.group(3))}
            add(t, f'both under the load:  starts {m.group(2)} s apart,  arrivals {m.group(3)} s apart')
            continue

        m = re.match(r'\[lift\] load_(\w+) raised ([\d.]+) m off the stand, (\d+) s after lift began', msg)
        if m:
            facts['raised'] = {'stand': m.group(1), 'height': float(m.group(2)),
                               'seconds': int(m.group(3))}
            marks['raised'] = t
            add(t, f'load_{m.group(1)} raised {m.group(2)} m:  lifted')
            continue

        if msg.startswith('[mission] mission complete'):
            marks['complete'] = t
            facts['complete'] = msg
            continue
        if msg.startswith('[mission] mission failed'):
            marks['complete'] = t
            facts['failed'] = msg
            continue

    states.sort(key=lambda s: s['t'])

    # Playback speed: the wait for the planning system and every transit are
    # sampled; the moments where captions fall close together run at real
    # time or slower.
    cuts = [(0.0, 1.0), (8.0, 4.0)]
    if 'policy' in marks:
        cuts.append((marks['policy'] - 0.5, 0.5))
        cuts.append((marks['policy'] + 7.0, 1.0))
    for a in marks.get('drive', []):
        cuts.append((a + 2.5, 4.0))
    for a in marks.get('arrive', []):
        cuts.append((a - 1.0, 0.7))
        cuts.append((a + 4.0, 1.0))
    for a in marks.get('radio', []):
        cuts.append((a - 0.3, 0.8))
    if 'refused' in marks:
        cuts.append((marks['refused'] - 0.5, 0.5))
        cuts.append((marks['refused'] + 4.0, 1.0))
    if 'signal' in marks:
        cuts.append((marks['signal'] - 0.5, 0.5))
        cuts.append((marks['signal'] + 6.0, 1.0))
    if 'lift' in marks:
        cuts.append((marks['lift'] + 2.0, 4.0))
    if 'lift_ready' in marks:
        cuts.append((marks['lift_ready'] - 1.0, 1.0))
    if 'raised' in marks:
        cuts.append((marks['raised'] - 4.0, 0.7))
        cuts.append((marks['raised'] + 3.0, 1.0))
    cuts.sort()
    segments = []
    for (a, speed), (b, _) in zip(cuts, cuts[1:] + [(None, None)]):
        if b is not None and b - a < 0.05:
            continue
        segments.append({'start': round(max(a, 0.0), 2),
                         'end': None if b is None else round(b, 2), 'speed': speed})
    return {'states': states, 'segments': segments, 'facts': facts, 'marks': marks}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--floor', default='beacon')
    ap.add_argument('--out')
    args = ap.parse_args()
    timeline = build(args.log, args.t0, args.floor)
    if args.out:
        with open(args.out, 'w') as fh:
            json.dump(timeline, fh, indent=1, ensure_ascii=False)
    for s in timeline['states']:
        print(f'{s["t"]:7.1f}  {s["step"]["label"]:20s} {s["step"]["title"][:70]}'
              f'   | {s["step"]["lines"][-1][:60] if s["step"]["lines"] else ""}')
    print(json.dumps(timeline['facts'], ensure_ascii=False))


if __name__ == '__main__':
    main()
