#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
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
Replays a plan through the product update, and says what each action needed.

A plan is a list of action names. What makes it a plan is not visible in that
list: the reason the second action of `go, inspect, pickup` cannot be dropped
is that the third carries a modal precondition which is false in the model the
first produces and true in the model the second produces. That fact lives in
the epistemic states between the actions, and printing the actions loses it.

This grounds the task with plank's export, applies the events of the plan by
the product update

    W' = { (w,e) : M, w |= pre(e) },
    (w,e) R'_i (v,f)  iff  w R_i v and f in Q_i(e),
    V'(w,e) = { p : M, w |= post_e(p) } U { p in V(w) : post_e(p) undefined },

and reports, at every node of the plan, the model the node acts in, the
precondition of the event it applies, and which conjuncts of that
precondition are modal and where they hold.

The update is checked rather than assumed. `--against` takes the analysis of a
recorded run and requires the models this reconstructs to be isomorphic to the
models the executor measured, step for step, which is what licenses the
figures on the domain pages.
"""

import argparse
import json
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from model_check import (Model, model_of, extensions, holds, subformulas,  # noqa: E402
                         tex, show, parse, escape, dots, extension_figure,
                         DEFAULT_EXT_CAPTION, INK, RED)


# ────────────────────────────────────── the exported formula representation ──

def convert(node):
    """One formula, from plank's JSON into the AST the checker evaluates."""
    if isinstance(node, dict) and 'formula' in node and len(node) == 1:
        return convert(node['formula'])
    if isinstance(node, str):
        return ('atom', node)
    if 'connective' in node:
        kind = node['connective']
        if kind == 'not':
            return ('not', convert(node['formula']))
        if kind in ('and', 'or'):
            return (kind, tuple(convert(f) for f in node['formulas']))
        if kind == 'imply':
            left, right = (convert(f) for f in node['formulas'])
            return ('or', (('not', left), right))
        raise ValueError(f'unknown connective {kind}')
    name = node['modality-name']
    agents = node['modality-index']
    inner = convert(node['formula'])
    if name == 'box':
        return ('k', agents[0], inner)
    if name == 'Kw.box':
        return ('kw', agents[0], inner)
    if name == 'Kw.diamond':
        return ('not', ('kw', agents[0], inner))
    if name == 'C.box':
        return ('c', tuple(agents), inner)
    raise ValueError(f'unknown modality {name}')


# ───────────────────────────────────────────────────────── the product update ──

def observability(action, agent, model, world):
    """The observability class an agent is in, at one world.

    The action declares, per agent, a condition for each class it can occupy.
    The class is the one whose condition the world satisfies; a class named
    `default` applies where no other condition does.
    """
    conditions = action.get('observability-conditions', {}).get(agent, {})
    fallback = None
    for name, condition in conditions.items():
        formula = convert(condition)
        if formula == ('atom', 'true'):
            fallback = name
            continue
        if world in extensions(model, formula)[formula]:
            return name
    if fallback is not None:
        return fallback
    return next(iter(conditions), None)


def update(model, action, agents, designated_events=None):
    """M (x) E, restricted to the events the branch designates."""
    events = action['events']
    pre = {e: convert(action['preconditions'][e]) for e in events}
    post = {e: {atom: convert(f)
                for atom, f in (action['effects'].get(e) or {}).items()}
            for e in events}

    table = {}
    for e in events:
        table[e] = extensions(model, pre[e])[pre[e]]

    pairs = [(w, e) for e in events for w in model.worlds if w in table[e]]
    name = {pair: f'w{index}' for index, pair in enumerate(pairs)}

    labels, relations = {}, {a: {} for a in agents}
    for (w, e) in pairs:
        assigned = {p: True for p in model.labels[w]}
        for atom, condition in post[e].items():
            assigned[atom] = w in extensions(model, condition)[condition]
        labels[name[(w, e)]] = {p for p, truth in assigned.items() if truth}

    for agent in agents:
        for (w, e) in pairs:
            image = []
            klass = observability(action, agent, model, w)
            reachable = action['relations'].get(klass, {}).get(e, [])
            for (v, f) in pairs:
                if v in model.image(agent, w) and f in reachable:
                    image.append(name[(v, f)])
            relations[agent][name[(w, e)]] = image

    keep = designated_events if designated_events is not None \
        else action['designated']
    new_designated = [name[(w, e)] for (w, e) in pairs
                      if w in model.designated and e in keep]

    return Model([name[p] for p in pairs], relations, labels, new_designated), \
        {name[p]: p for p in pairs}


# ─────────────────────────────────────────────────────────────── comparison ──

def canonical(model, agents, atoms):
    """A model up to the naming of its worlds, for comparing two of them.

    Worlds are compared by the atoms that vary across the model and by the
    relations they stand in, iterated to a fixed point. Two models agree when
    the multisets of signatures agree and the designated signatures agree,
    which is bisimulation on a finite model.
    """
    signature = {w: repr(sorted(model.labels[w] & atoms)) for w in model.worlds}
    for _ in range(len(model.worlds) + 1):
        refined = {w: repr((signature[w],
                            [sorted({signature[v] for v in model.image(a, w)})
                             for a in agents]))
                   for w in model.worlds}
        classes = len(set(signature.values()))
        signature = refined
        if len(set(signature.values())) == classes:
            break
    return (sorted(signature.values()),
            sorted(signature[w] for w in model.designated))


# ─────────────────────────────────────────────────────── walking a solution ──

def branch_events(action, branch):
    """The event a branch of the plan selects, by index or by name."""
    event = branch.get('event')
    if isinstance(event, int):
        return action['events'][event]
    return event


def follow(task, plan, agents, choices=()):
    """The states a plan passes through, along one branch of it.

    `choices` names the event to take at each branch point, in order; where it
    runs out the first designated event is taken. The result is one entry per
    node, carrying the model the node acts in and the precondition of the
    event it applies, which is what a figure needs to say why the node is
    there.
    """
    model = model_of(task['initial-state'])
    taken = list(choices)
    track = [{'label': '0', 'action': 'initial state', 'model': model,
              'recorded': {}, 'event': None, 'precondition': None}]
    node, index = plan, 0
    while node is not None:
        action = task['actions'][node['action']]
        options = [branch_events(action, b) for b in node['branches']]
        # A choice is spent only where there is one to make.
        event = taken.pop(0) if (taken and len(options) > 1) else options[0]
        branch = node['branches'][options.index(event)]
        # An action is applicable when its designated events between them
        # cover the designated set: each event then carries away the worlds
        # whose precondition it satisfies, which is how one action produces
        # several continuations.
        covered = set()
        for candidate in options:
            formula = convert(action['preconditions'][candidate])
            covered |= extensions(model, formula)[formula]
        if not model.designated <= covered:
            raise SystemExit(f'{node["action"]} is not applicable: its events '
                             'do not cover the designated set')
        precondition = convert(action['preconditions'][event])
        model, _ = update(model, action, agents, [event])
        if not model.designated:
            raise SystemExit(f'{node["action"]} / {event} leaves the designated '
                             'set empty: that reading cannot occur here')
        index += 1
        track.append({'label': str(index), 'action': node['action'],
                      'model': model, 'recorded': {}, 'event': event,
                      'precondition': precondition})
        node = branch['subtree']
    return track


def guard(action, event, model):
    """The modal conjuncts of one event's precondition, and where they hold."""
    precondition = convert(action['preconditions'][event])
    conjuncts = precondition[1] if precondition[0] == 'and' else (precondition,)
    modal = [c for c in conjuncts if modal_depth(c) > 0]
    return [(c, extensions(model, c)[c]) for c in modal]


def modal_depth(node):
    kind = node[0]
    if kind == 'atom':
        return 0
    if kind == 'not':
        return modal_depth(node[1])
    if kind in ('and', 'or'):
        return max(modal_depth(p) for p in node[1])
    return 1 + modal_depth(node[2])


# ───────────────────────────────────────────────────────────────── figures ──

BOX_W, BOX_H = 268, 56
COL, ROW = 316, 104
PAD_X, PAD_TOP = 20, 46
ARG_CHARS = 34          # arguments that fit one line of the box

KIND = {'sensing': '#C8102E', 'ontic': '#111111', 'announcement': '#1A52A0'}


def kind_of(action):
    """The class of an action, from the type it declares rather than its name."""
    declared = action.get('action-type', '')
    for key in KIND:
        if key in declared:
            return key
    return 'ontic'


def split_name(name):
    """A grounded action as its head and its arguments, wrapped to the box."""
    head, _, rest = name.partition('_')
    lines, line = [], ''
    for word in rest.split('_'):
        if not word:
            continue
        if len(line) + len(word) + 1 > ARG_CHARS and line:
            lines.append(line)
            line = word
        else:
            line = f'{line} {word}'.strip()
    if line:
        lines.append(line)
    if len(lines) > 2:
        lines = lines[:2]
        lines[1] = lines[1][:ARG_CHARS - 1] + '&#8230;'
    return head, lines


def plan_diagram(task, plan, agents, formula, title, caption, taken=None):
    """The solution drawn as the graph it is, with the state at every node.

    A solution is an AND-OR tree: an action, and one continuation per event
    that action can produce. A sequence is the degenerate case with a single
    continuation throughout, so one drawing serves both, and it runs downwards
    because that is the direction a plan is read in and the direction that
    does not force the text to shrink as the plan gets longer.
    """
    nodes, edges = [], []

    def visit(node, model, depth, column, event_in):
        index = len(nodes)
        nodes.append({'node': node, 'model': model, 'depth': depth,
                      'column': column, 'event': event_in})
        if node is None:
            return column + 1
        action = task['actions'][node['action']]
        options = [branch_events(action, b) for b in node['branches']]
        nodes[index]['kind'] = kind_of(action)
        nodes[index]['guards'] = [(e, guard(action, e, model)) for e in options]
        nodes[index]['branching'] = len(options) > 1
        next_column = column
        for branch, event in zip(node['branches'], options):
            child, _ = update(model, action, agents, [event])
            edges.append((index, len(nodes), event if len(options) > 1 else None))
            next_column = visit(branch['subtree'], child, depth + 1,
                                next_column, event if len(options) > 1 else None)
        return next_column

    initial = model_of(task['initial-state'])
    visit(plan, initial, 0, 0, None)

    columns = max(n['column'] for n in nodes) + 1
    depths = max(n['depth'] for n in nodes) + 1
    width = PAD_X * 2 + (columns - 1) * COL + BOX_W
    height = PAD_TOP + (depths - 1) * ROW + BOX_H + 26

    def x_of(n):
        return PAD_X + n['column'] * COL

    def y_of(n):
        return PAD_TOP + n['depth'] * ROW

    out = [f'<svg viewBox="0 0 {width:.0f} {height:.0f}" width="100%" '
           f'style="max-width:{width:.0f}px;display:block;margin:0 auto" '
           'xmlns="http://www.w3.org/2000/svg" role="img" '
           f'aria-label="{escape(title)}">',
           '  <defs>',
           '    <marker id="pd-arr" viewBox="0 0 10 10" refX="9" refY="5" '
           'markerWidth="6" markerHeight="6" orient="auto-start-reverse">',
           '      <path d="M0 0 L10 5 L0 10 z" fill="#9A9A9A"/>',
           '    </marker>',
           '  </defs>',
           '  <style>',
           "    .pd-h{font-family:'Courier New',Courier,monospace;font-size:13px;"
           '      fill:#111;font-weight:bold}',
           "    .pd-a{font-family:'Courier New',Courier,monospace;font-size:10.5px;"
           '      fill:#5A5A5A}',
           '    .pd-t{font-size:9px;fill:#5A5A5A;letter-spacing:.06em}',
           "    .pd-q{font-family:'Courier New',Courier,monospace;font-size:10px;"
           '      fill:#5A5A5A}',
           "    .pd-e{font-family:'Courier New',Courier,monospace;font-size:10px;"
           '      fill:#C8102E}',
           '    .pd-n{font-size:10px;fill:#111;font-weight:bold}',
           '  </style>']


    for parent, child, event in edges:
        a, b = nodes[parent], nodes[child]
        x1, y1 = x_of(a) + BOX_W / 2, y_of(a) + BOX_H
        x2, y2 = x_of(b) + BOX_W / 2, y_of(b) - 7
        dim = '' if taken is None or event in (None, taken) else ' opacity=".34"'
        if abs(x1 - x2) < 1:
            path = f'M{x1:.0f} {y1:.0f} L{x2:.0f} {y2:.0f}'
        else:
            mid = (y1 + y2) / 2
            path = (f'M{x1:.0f} {y1:.0f} C{x1:.0f} {mid:.0f} {x2:.0f} {mid:.0f} '
                    f'{x2:.0f} {y2:.0f}')
        out.append(f'  <path d="{path}" stroke="#9A9A9A" stroke-width="1.3" '
                   f'fill="none" marker-end="url(#pd-arr)"{dim}/>')

    for index, n in enumerate(nodes):
        x, y = x_of(n), y_of(n)
        if n['event']:
            # The event that selects this continuation is named above the node
            # it leads to. An action box carries its number on the left, so the
            # name is set to the right of it; a leaf has no number, so the name
            # is set over the marker it belongs to.
            faded = ' opacity=".45"' if taken is not None and n['event'] != taken \
                else ''
            at, anchor = ((x + 2, '') if n['node'] is None
                          else (x + BOX_W, ' text-anchor="end"'))
            out.append(f'  <text class="pd-e" x="{at:.0f}" y="{y - 8:.0f}"'
                       f'{anchor}{faded}>{escape(n["event"])}</text>')
        if n['node'] is None:
            out.append(f'  <circle cx="{x + 14:.0f}" cy="{y + 16:.0f}" r="5.5" '
                       'fill="#111"/>')
            out.append(f'  <text class="pd-h" x="{x + 28:.0f}" y="{y + 20:.0f}">'
                       'goal reached</text>')
            out += chip(n['model'], formula, x + BOX_W, y + 16)
            continue
        colour = KIND[n['kind']]
        head, args = split_name(n['node']['action'])
        out.append('  <g>')
        out.append(f'    <rect x="{x:.0f}" y="{y:.0f}" width="{BOX_W}" '
                   f'height="{BOX_H}" rx="3" fill="#fff" stroke="{colour}" '
                   'stroke-width="1.5"/>')
        out.append(f'    <text class="pd-n" x="{x + 2:.0f}" y="{y - 8:.0f}">'
                   f'{index + 1}</text>')
        out.append(f'    <text class="pd-t" x="{x + 16:.0f}" y="{y - 8:.0f}" '
                   f'fill="{colour}">{n["kind"].upper()}'
                   f'{" &#183; BRANCH" if n["branching"] else ""}</text>')
        out.append(f'    <text class="pd-h" x="{x + 13:.0f}" y="{y + 22:.0f}">'
                   f'{escape(head)}</text>')
        for line, text in enumerate(args):
            out.append(f'    <text class="pd-a" x="{x + 13:.0f}" '
                       f'y="{y + 38 + line * 12:.0f}">{escape(text)}</text>')
        out += chip(n['model'], formula, x + BOX_W - 13, y + 20)
        out.append('  </g>')

    out.append('</svg>')

    legend = []
    for index, n in enumerate(nodes):
        if n['node'] is None:
            continue
        for event, conjuncts in n['guards']:
            prefix = f'<b>{escape(event)}</b>&#8202;: ' if n['branching'] else ''
            for f, where in conjuncts:
                legend.append(
                    f'<li><span class="ln">{index + 1}</span>{prefix}'
                    f'\\({tex(f)}\\) holds at {len(where)} of the '
                    f'{len(n["model"].worlds)} worlds of the model this action '
                    f'is given, and \\(|W^{{*}}| = '
                    f'{len(n["model"].designated)}\\)</li>')

    return ('<style>\n'
            '.pdleg{margin:.8rem 0 0;padding:0;list-style:none;font-size:.79rem;\n'
            '  color:var(--gray);display:grid;gap:.3rem;\n'
            '  border-top:1px solid var(--border);padding-top:.8rem}\n'
            '.pdleg .ln{display:inline-block;min-width:1.3rem;font-weight:600;\n'
            '  color:var(--black);font-family:var(--mono);font-size:.72rem}\n'
            '.pdkey{display:flex;gap:1.15rem;flex-wrap:wrap;font-size:.74rem;\n'
            '  color:var(--gray);margin:.9rem 0 0;align-items:center}\n'
            '.pdkey span{display:inline-flex;align-items:center}\n'
            '.pdkey i{display:inline-block;width:14px;height:10px;\n'
            '  border:1.5px solid;border-radius:2px;margin-right:.34rem}\n'
            '.pdkey .dot b{display:inline-block;width:7px;height:7px;\n'
            '  border-radius:50%;background:#111;border:1px solid #111;\n'
            '  margin-right:.34rem}\n'
            '.pdkey .dot b.e{background:#fff}\n'
            '.pdkey .dot b.d{background:#fff;border:1.4px solid #C8102E;\n'
            '  width:9px;height:9px}\n'
            '</style>\n'
            '<div class="fig">\n'
            '  <div class="fig-head">\n'
            f'    <span class="fig-title">{title}</span>\n'
            f'    <span class="note">\\(\\gamma \\equiv {tex(formula)}\\)</span>\n'
            '  </div>\n'
            '  <div class="fig-body">\n    '
            + '\n    '.join(out)
            + '\n    <div class="pdkey">'
            '<span><i style="border-color:#C8102E"></i>sensing</span>'
            '<span><i style="border-color:#111111"></i>ontic</span>'
            '<span><i style="border-color:#1A52A0"></i>announcement</span>'
            '<span class="dot"><b></b>a world at which the goal holds</span>'
            '<span class="dot"><b class="e"></b>a world at which it does not</span>'
            '<span class="dot"><b class="d"></b>a designated world</span>'
            '</div>'
            + ('\n    <ul class="pdleg">\n      ' + '\n      '.join(legend)
               + '\n    </ul>' if legend else '')
            + '\n  </div>\n'
            f'  <div class="fig-cap">{caption}</div>\n'
            '</div>\n')


def chip(model, formula, right, y):
    """The model a node is given, drawn inside the node and right-aligned.

    One dot per world, filled where the goal holds and ringed where the world
    is designated. It is set inside the box rather than beside it so that the
    width of a column cannot depend on the size of the model, which is what
    made a wide model in one column collide with the box in the next.
    """
    inside = extensions(model, formula)[formula]
    n = len(model.worlds)
    out = []
    for k, w in enumerate(model.worlds):
        cxx = right - (n - 1 - k) * 14
        if w in model.designated:
            out.append(f'  <circle cx="{cxx:.0f}" cy="{y:.0f}" r="6.2" '
                       'fill="none" stroke="#C8102E" stroke-width="1.1"/>')
        out.append(f'  <circle cx="{cxx:.0f}" cy="{y:.0f}" r="3.4" '
                   f'fill="{"#111" if w in inside else "#fff"}" '
                   'stroke="#111" stroke-width="1"/>')
    out.append(f'  <text class="pd-q" x="{right + 6:.0f}" y="{y + 21:.0f}" '
               f'text-anchor="end">|W| = {n}</text>')
    return out


# ───────────────────────────────────────────────────────────────────── cli ──

STATS = [
    ('|W|', lambda m: len(m.worlds)),
    ('|W^{*}|', lambda m: len(m.designated)),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)

    for name in ('matrix', 'diagram', 'report'):
        p = sub.add_parser(name)
        p.add_argument('--task', required=True, help='grounded task JSON')
        p.add_argument('--plan', required=True, help='the solution Aletheia wrote')
        p.add_argument('--out')
        p.add_argument('--title', default='')
        p.add_argument('--caption', default='')
        p.add_argument('--formula', help='the formula whose extension is drawn; '
                                         'the goal by default')
        p.add_argument('--extra', action='append', default=[])
        p.add_argument('--take', action='append', default=[],
                       help='repeatable; the event to take at each branch point')
        p.add_argument('--relation', action='append', default=[],
                       help='repeatable; an agent whose relation is counted')
        p.add_argument('--against', help='the analysis of a recorded run this '
                                         'reconstruction must reproduce')

    args = parser.parse_args()
    task = json.load(open(args.task))
    plan = json.load(open(args.plan))
    agents = task['language']['agents']
    formula = parse(args.formula) if args.formula else convert(task['goal'])

    if args.against:
        verify(task, agents, json.load(open(args.against)))

    stats = list(STATS)
    for agent in args.relation:
        stats.append((f'|R_{{\\texttt{{{agent}}}}}|',
                      lambda m, a=agent: sum(len(m.image(a, w)) for w in m.worlds)))

    if args.command == 'diagram':
        fragment = plan_diagram(
            task, plan, agents, formula,
            args.title or 'The solution, drawn, with the state at every node',
            args.caption or DEFAULT_DIAGRAM_CAPTION,
            args.take[0] if args.take else None)
        open(args.out, 'w').write(fragment)
        print(f'-> {args.out}')
        return

    track = follow(task, plan, agents, args.take)
    if args.command == 'matrix':
        fragment = extension_figure(
            track, formula,
            args.title or 'The goal, and where it holds, after every action',
            args.caption or DEFAULT_EXT_CAPTION,
            [parse(text) for text in args.extra], stats)
        open(args.out, 'w').write(fragment)
        print(f'{len(track)} nodes -> {args.out}')
    else:
        for entry in track:
            model = entry['model']
            print(f'{entry["label"]:>3}  {entry["action"]}'
                  f'{" / " + entry["event"] if entry["event"] else ""}')
            print(f'     |W|={len(model.worlds)}  W*={sorted(model.designated)}  '
                  f'goal={holds(model, formula)}')
            for node in subformulas(formula):
                print(f'     {show(node):<46} '
                      f'{sorted(extensions(model, formula)[node])}')


def verify(task, agents, analysis):
    """The reconstruction, required to match a run that was measured."""
    from model_check import timeline, agree
    measured = timeline(analysis, agents)
    atoms = set()
    for entry in measured:
        for w in entry['model'].worlds:
            atoms |= entry['model'].labels[w]

    model = model_of(task['initial-state'])
    steps = [None] + [(s['action'], s['outcome']) for s in analysis['steps']]
    for index, step in enumerate(steps):
        if step:
            model, _ = update(model, task['actions'][step[0]], agents,
                              [step[1]] if step[1] else None)
        if canonical(model, agents, atoms) != canonical(measured[index]['model'],
                                                        agents, atoms):
            raise SystemExit(f'step {index}: the reconstruction is not '
                             'bisimilar to the model that was measured')
        if agree(model, measured[index]['recorded']):
            raise SystemExit(f'step {index}: the reconstruction contradicts a '
                             'recorded verdict')
    print(f'{len(steps)} reconstructed models match the run that was measured')


DEFAULT_DIAGRAM_CAPTION = (
    'The solution as the AND-OR tree it is, drawn from the planner&#8217;s '
    'output. Beside each action is the model that action is given: one dot per '
    'world, filled where \\(\\gamma\\) holds and ringed where the world is '
    'designated, so a row in which every ringed dot is filled is a state '
    'satisfying the goal. An action with two outgoing edges has two designated '
    'events, and the continuation below each edge is applicable only in the '
    'state that edge produces, which is what makes a solution a policy rather '
    'than a sequence. The numbered notes give the modal conjuncts each action '
    'is guarded by; an action with no note has none, and is applicable on '
    'ontic grounds alone.')


if __name__ == '__main__':
    main()
