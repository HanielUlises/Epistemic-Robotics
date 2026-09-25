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
Reads a pass-through run's log into the timeline the video is captioned from.

    make_captions.py --log run.log --t0 <capture start, unix s> --out timeline.json

Every caption is a line the run itself wrote, placed at the instant it was
written, so a caption saying the carrier's map now shows the load appears at
the frame in which the fusion returned. Nothing here decides what happened:
which bay was read as what, how many cells an exchange carried, what the model
says each agent knows, are all numbers the performers, the knowledge nodes and
the epistemic state reported. What is added is the notation.

The timeline has three parts:

  states    what the panel under the video shows from a given instant: the
            step in formal notation, the knowledge table, the policy node
  segments  the playback speed over each stretch of the capture: transits are
            sampled, and the stretches where the epistemic events fall close
            together are slowed so each caption can be read
  facts     the run's figures, for the closing card

Formulas use a small markup the renderer understands: `_{...}` is a subscript.
"""

import argparse
import json
import re

LINE = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)')

SUB = {'t1': 't_{1}', 't2': 't_{2}', 't3': 't_{3}'}


def bay(name):
    return SUB.get(name, name)


def num(n):
    """Thousands separated by a thin space, as the pages set them."""
    return f'{int(n):,}'.replace(',', '\u2009')


def parse(log, t0):
    out = []
    with open(log, errors='replace') as fh:
        for raw in fh:
            m = LINE.search(raw)
            if m:
                out.append((float(m.group(1)) - t0, m.group(2), m.group(3).strip()))
    # By stamp, not by position in the file: the launch interleaves several
    # processes' output, and a line can reach the file after one stamped later.
    out.sort(key=lambda e: e[0])
    return out


def table_from(line, bays):
    """A [knows] line into rows: agent or group -> {bay: open|shut|?}."""
    head, _, facts = line.partition(': ')
    shape = re.match(r'\[knows\] (\d+) worlds, (\d+) designated', head)
    rows = {}
    for fact in facts.split(' · '):
        m = re.match(r'(K_(\w+)|D\{([\w,]+)\}) (~?)open_(\w+)', fact.strip())
        if not m:
            continue
        who = m.group(2) or 'D{' + m.group(3) + '}'
        rows.setdefault(who, {})[m.group(5)] = 'shut' if m.group(4) else 'open'
    return {'worlds': int(shape.group(1)) if shape else 0,
            'designated': int(shape.group(2)) if shape else 0,
            'rows': rows, 'bays': bays}


def build(log, t0, bays=('t1', 't2', 't3')):
    events = parse(log, t0)
    states, segments, facts = [], [], {}
    table = {'worlds': 3, 'designated': 3, 'rows': {}, 'bays': list(bays)}
    step = {'label': 'INITIAL MODEL', 'title': 'M_{0}', 'gloss': '', 'lines': []}
    node = None
    marks = {}

    def emit(t, **changes):
        nonlocal step, table, node
        if 'step' in changes:
            step = changes['step']
        if 'table' in changes:
            table = changes['table']
        if 'node' in changes:
            node = changes['node']
        states.append({'t': round(t, 2), 'step': json.loads(json.dumps(step)),
                       'table': json.loads(json.dumps(table)), 'node': node})

    def add_line(t, text):
        step['lines'].append(text)
        emit(t)

    step = {
        'label': 'INITIAL MODEL',
        'title': 'M_{0}:  3 worlds, all designated,  w_{k} ⊨ open(t_{k})',
        'gloss': 'exactly one bay is open, and that much is common knowledge; which one is not',
        'lines': ['s ⊨ C_{all} (open(t_{1}) ⊕ open(t_{2}) ⊕ open(t_{3}))',
                  's ⊨ ¬Kw_{i} open(t_{k})   for every agent i and bay k'],
    }
    emit(0.0)

    for t, node_name, msg in events:
        if msg.startswith('[knows]'):
            new = table_from(msg, list(bays))
            before = table['rows']
            emit(t, table=new)
            # Two moments the table shows and the caption should say.
            for who, cells in new['rows'].items():
                for t_, s in cells.items():
                    if s != 'open' or before.get(who, {}).get(t_) == 'open':
                        continue
                    individually = [a for a in new['rows'] if not a.startswith('D{')
                                    and a != who and new['rows'][a].get(t_) != 'open']
                    if who.startswith('D{') and 'distributed' not in facts:
                        group = who[2:-1].split(',')
                        facts['distributed'] = {'group': group, 'bay': t_}
                        add_line(t, f's ⊨ D_{{{",".join(group)}}} open({bay(t_)}),   yet '
                                    + ' and '.join(f'¬K_{{{g}}} open({bay(t_)})' for g in group)
                                    + ':  the pair knows it, neither does')
                    elif not who.startswith('D{') and 'elimination' not in facts \
                            and len(individually) >= 2:
                        carried = sum(a['bays'].get(t_, 0) for a in facts.get('absorbed', [])
                                      if a['to'] == who)
                        facts['elimination'] = {'agent': who, 'bay': t_, 'cells_carried': carried}
                        add_line(t, f's ⊨ K_{{{who}}} open({bay(t_)}) by elimination: the exchanges '
                                    f'carried {carried} cells of {t_}')
            continue

        m = re.match(r'\[reach\] carrier not in W\(dock\): \|W\| = (\d+) cells after (\d+) iterations', msg)
        if m and 'reach_out' not in facts:
            facts['reach_out'] = {'cells': int(m.group(1)), 'iterations': int(m.group(2))}
            step['lines'].append(
                f'carrier ∉ W(dock),  W = µZ.(dock ∨ (Safe_{{carrier}} ∧ ◇Z)):  '
                f'|W| = {num(m.group(1))} cells, fixed point after {m.group(2)} iterations')
            emit(max(t, 0.0))
            continue

        m = re.match(r'\[mission\] policy with (\d+) nodes, (\d+) leaves, \w+, in ([\d.]+) s', msg)
        if m:
            facts['policy'] = {'nodes': int(m.group(1)), 'leaves': int(m.group(2)),
                               'seconds': float(m.group(3))}
            marks['policy'] = t
            emit(t, step={
                'label': 'POLICY',
                'title': f'π:  {m.group(1)} nodes,  {m.group(2)} leaves,  found in {m.group(3)} s',
                'gloss': 'AO* over the grounded task; after each survey the policy branches on '
                         'what the scout reads',
                'lines': ['which branch runs is decided by the scouts\' maps, not by the planner']})
            continue

        m = re.match(r'\[survey\] (\w+) sets out for the mouth of (\w+)', msg)
        if m:
            who, t_ = m.group(1), m.group(2)
            marks.setdefault('survey_start', []).append(t)
            emit(t, node=f'survey_{who}_{t_}', step={
                'label': 'SENSING ACTION',
                'title': f'survey({who}, {bay(t_)})',
                'gloss': f'semi-private sensing: {who} learns whether {t_} is open; '
                         f'the others learn only that it looked',
                'lines': [f'events  e-open: open({bay(t_)})     e-shut: ¬open({bay(t_)})']})
            continue

        m = re.match(r'\[survey\] (\w+) at the mouth of (\w+) after (\d+) s', msg)
        if m:
            marks.setdefault('survey_arrive', []).append(t)
            continue

        m = re.match(r'\[survey\] (\w+) cannot see all of (\w+) from the mouth \((\d+) of (\d+) cells\)', msg)
        if m:
            add_line(t, f'from the mouth {m.group(3)} of {m.group(4)} cells of {m.group(2)} observed: '
                        f'undecided, so {m.group(1)} moves in')
            continue

        m = re.match(r'\[survey\] (\w+) read (\w+) on its own map: (\w+), (\d+) of (\d+) cells seen, '
                     r'(\d+) occupied -> ([\w-]+)', msg)
        if m:
            who, t_, verdict, seen, cells, occ, ev = m.groups()
            marks.setdefault('survey_read', []).append(t)
            facts.setdefault('surveys', []).append(
                {'agent': who, 'bay': t_, 'verdict': verdict, 'seen': int(seen),
                 'cells': int(cells), 'occupied': int(occ), 'event': ev})
            add_line(t, f'{who}\'s own SLAM map over {t_}: {seen} of {cells} cells observed, '
                        f'{occ} occupied  ⇒  {verdict}  ⇒  {ev}')
            continue

        m = re.match(r'\[epistemic_state\] applied ([\w-]+?)(?: -> ([\w-]+))?: (\d+) worlds, (\d+) designated',
                     msg)
        if m:
            action, ev, worlds, desig = m.groups()
            marks.setdefault('applied', []).append((t, action))
            what = f'M ⊗ E:  {worlds} worlds, {desig} designated'
            if action.startswith('survey_'):
                _, who, t_ = action.split('_')
                sign = '¬' if ev == 'e-shut' else ''
                add_line(t, f'{what}      s ⊨ K_{{{who}}} {sign}open({bay(t_)})')
            elif action.startswith('share-'):
                kind, frm, to, t_ = re.match(r'share-(\w+)_(\w+)_(\w+)_(\w+)', action).groups()
                sign = '¬' if kind == 'shut' else ''
                add_line(t, f'{what}      s ⊨ K_{{{to}}} {sign}open({bay(t_)})')
            elif action.startswith('cross_'):
                _, who, t_ = action.split('_')
                facts['collapsed'] = {'worlds': int(worlds), 'designated': int(desig)}
                add_line(t, f'cross is public: M collapses to {worlds} world,  '
                            f's ⊨ C_{{all}} open({bay(t_)})')
            continue

        m = re.match(r'\[share\] (\w+) sends its map to (\w+): share-(\w+)\((\w+), (\w+), (\w+)\)', msg)
        if m:
            frm, to, kind, _, _, t_ = m.groups()
            marks.setdefault('share_start', []).append(t)
            sign = '¬' if kind == 'shut' else ''
            emit(t, node=f'share-{kind}_{frm}_{to}_{t_}', step={
                'label': 'ANNOUNCEMENT',
                'title': f'share-{kind}({frm}, {to}, {bay(t_)})',
                'gloss': f'semi-private announcement of K_{{{frm}}} {sign}open({bay(t_)}): '
                         f'{to} hears it, the third agent observes only that {frm} spoke',
                'lines': [f'carried as a map: {frm}\'s knowledge map fused into {to}\'s '
                          f'(epistemic_slam::fuse)']})
            continue

        m = re.match(r'\[share\] (\w+) now holds (\d+) cells it had not observed, (\d+) of them in (\w+); '
                     r'(\d+) conflicts', msg)
        if m:
            to, n, nb, t_, conf = m.groups()
            facts.setdefault('shares', []).append(
                {'to': to, 'cells': int(n), 'in_bay': int(nb), 'bay': t_, 'conflicts': int(conf)})
            add_line(t, f'{num(n)} cells newly known to {to}, {nb} of them in {t_},  '
                        f'{conf} conflicts')
            continue

        m = re.match(r'\[knowledge\] (\w+) absorbed the map of (\w+): (\d+) cells newly known, '
                     r'(\d+) conflicts; in the bays:(.*)', msg)
        if m:
            per_bay = dict(kv.split('=') for kv in m.group(5).split())
            facts.setdefault('absorbed', []).append(
                {'to': m.group(1), 'from': m.group(2), 'bays': {k: int(v) for k, v in per_bay.items()}})
            continue

        m = re.match(r"\[share\] (\w+)'s knowledge map reads (\w+) (\w+), as announced", msg)
        if m:
            continue

        m = re.match(r'\[cross\] (\w+) sets out for the dock through (\w+): .*route ([\d.]+) m through (\w+)',
                     msg)
        if m:
            who, t_, length, via = m.groups()
            facts['route'] = {'bay': t_, 'length': float(length), 'via': via}
            marks['cross_start'] = t
            emit(t, node=f'cross_{who}_{t_}', step={
                'label': 'ONTIC ACTION',
                'title': f'cross({who}, {bay(t_)})      pre:  K_{{{who}}} open({bay(t_)})',
                'gloss': 'the route is a least fixed point over what the carrier knows',
                'lines': [f'W = µZ.(dock ∨ (Safe_{{{who}}} ∧ ◇Z)),   Safe_{{{who}}} = '
                          f'⟦free ∨ ⋁_{{t}} (t ∧ K_{{{who}}} open(t))⟧',
                          f'{t_} is in Safe only by the second disjunct: unknown in every map, '
                          f'lifted by knowledge (cyan)',
                          f'route {length} m through {via}']})
            continue

        m = re.match(r'\[cross\] (\w+) enters (\w+)', msg)
        if m:
            marks['enter'] = t
            add_line(t, f'the carrier enters {m.group(2)}: no robot has surveyed it')
            continue

        m = re.match(r"\[cross\] (\w+)'s own scan now covers (\w+): (\d+) of (\d+) cells seen", msg)
        if m:
            facts['own_scan'] = {'seen': int(m.group(3)), 'cells': int(m.group(4))}
            continue

        m = re.match(r'\[cross\] (\w+) at the dock after (\d+) s', msg)
        if m:
            facts['crossing_seconds'] = int(m.group(2))
            marks['dock'] = t
            add_line(t, f'at the dock after {m.group(2)} s:  delivered')
            continue

        if msg.startswith('[mission] mission complete'):
            marks['complete'] = t
            continue

    # Only the knowledge updates that follow the first state have to be
    # ordered; a [knows] line written before the capture opened is the initial
    # table, and belongs at zero.
    for s in states:
        s['t'] = max(0.0, s['t'])
    states.sort(key=lambda s: s['t'])

    # Playback speed. Transits are sampled; the stretch from a scout's arrival
    # to the end of its announcement, where three or four captions fall inside
    # ten seconds, is slowed; the crossing is sampled except around the moment
    # the carrier enters the bay nobody surveyed.
    # The opening: the initial model is read at real time, the wait for the
    # planning system is sampled, and the moment the policy arrives is slowed
    # so its caption can be read before the first survey replaces it.
    cuts = [(0.0, 1.0), (9.0, 3.0)]
    if 'policy' in marks:
        cuts += [(marks['policy'] - 0.5, 0.3)]
    for start, arrive in zip(marks.get('survey_start', []), marks.get('survey_arrive', [])):
        cuts += [(start + 1.5, 2.5), (arrive - 1.0, 0.5)]
    # After the reading the scout clears the approach, which is transit.
    for read in marks.get('survey_read', []):
        cuts += [(read + 2.0, 2.5)]
    for t_share in marks.get('share_start', []):
        cuts += [(t_share - 0.5, 0.5), (t_share + 5.0, 1.0)]
    if 'cross_start' in marks:
        cuts += [(marks['cross_start'], 0.6), (marks['cross_start'] + 6.0, 4.0)]
    if 'enter' in marks:
        cuts += [(marks['enter'] - 5.0, 1.0), (marks['enter'] + 6.0, 4.0)]
    if 'dock' in marks:
        cuts += [(marks['dock'] - 3.0, 1.0)]
    cuts.sort()
    for (a, speed), (b, _) in zip(cuts, cuts[1:] + [(None, None)]):
        segments.append({'start': round(a, 2), 'end': None if b is None else round(b, 2),
                         'speed': speed})
    return {'states': states, 'segments': segments, 'facts': facts, 'marks': marks}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    timeline = build(args.log, args.t0)
    with open(args.out, 'w') as fh:
        json.dump(timeline, fh, indent=1, ensure_ascii=False)
    print(f'{args.out}: {len(timeline["states"])} states, {len(timeline["segments"])} segments')
    for s in timeline['states']:
        print(f'{s["t"]:7.1f}  {s["step"]["label"]:15s} {s["step"]["title"][:60]}')


if __name__ == '__main__':
    main()
