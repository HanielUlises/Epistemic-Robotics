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
Measures each agent's accessibility relation as a graph, at every step.

The run's report already records how many worlds an agent designates. That
counts vertices. This counts the edges and checks the closure properties they
satisfy, which is what decides the logic the run is in: an equivalence
relation gives S5 and reads as knowledge; a serial, transitive, Euclidean
relation that has lost reflexivity gives KD45 and reads as belief, which may
be false.

    relation_metrics.py --analysis out/analysis-bay2.json \\
                        --agents r1 r2 --out docs/img/warehouse-rmf-frames.svg

The properties are checked on the relation as recorded, so the verdict is
measured off the run rather than assumed from the domain's declared frame.
"""

import argparse
import json

CELL_W = 118
CELL_H = 54
LABEL_W = 132
PAD = 16
HEAD_H = 46


def properties(relation, worlds):
    """The closure properties of one agent's relation, on the worlds it has."""
    edges = {(w, v) for w in worlds for v in relation.get(w, [])}
    has = lambda w, v: (w, v) in edges

    reflexive = all(has(w, w) for w in worlds)
    serial = all(relation.get(w) for w in worlds)
    symmetric = all(has(v, w) for (w, v) in edges)
    transitive = all(has(w, y) for (w, x) in edges for (x2, y) in edges if x2 == x)
    euclidean = all(has(x, y) for (w, x) in edges for (w2, y) in edges if w2 == w)

    # How many distinct sets of worlds an agent considers possible. Under S5
    # this is the number of equivalence classes.
    images = {frozenset(relation.get(w, [])) for w in worlds}

    if reflexive and transitive and euclidean:
        frame = 'S5'
    elif serial and transitive and euclidean:
        frame = 'KD45'
    else:
        frame = '--'

    return {
        'edges': len(edges),
        'worlds': len(worlds),
        'classes': len(images),
        'reflexive': reflexive,
        'frame': frame,
    }


def render(rows, columns, agents, title):
    width = LABEL_W + len(columns) * CELL_W + 2 * PAD
    height = HEAD_H + len(agents) * CELL_H + 2 * PAD + 40

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}"',
        f'     width="{width}" height="{height}"',
        '     font-family="Arial, Helvetica, sans-serif" role="img"',
        f'     aria-label="{title}">',
        '  <style>',
        '    .cell  { fill:#FFFFFF; stroke:#E2E2E2; stroke-width:1 }',
        '    .left  { fill:#FFF5F6; stroke:#C8102E; stroke-width:1.5 }',
        "    .n     { font-family:'Courier New',Courier,monospace; font-size:12.5px;",
        '             fill:#111111 }',
        "    .sub   { font-family:'Courier New',Courier,monospace; font-size:10px;",
        '             fill:#7A7A7A }',
        '    .bad   { fill:#C8102E }',
        '    .head  { font-size:11.5px; fill:#5A5A5A }',
        '    .step  { font-family:\'Courier New\',Courier,monospace; font-size:10px;',
        '             fill:#9A9A9A }',
        '    .agent { font-size:12.5px; font-weight:bold; fill:#111111 }',
        '  </style>',
    ]

    for index, column in enumerate(columns):
        x = PAD + LABEL_W + index * CELL_W + CELL_W / 2
        out.append(f'  <text class="head" x="{x:.0f}" y="{PAD + 16}" '
                   f'text-anchor="middle">{column["label"]}</text>')
        out.append(f'  <text class="step" x="{x:.0f}" y="{PAD + 30}" '
                   f'text-anchor="middle">{column["step"]}</text>')

    for row, agent in enumerate(agents):
        y = PAD + HEAD_H + row * CELL_H
        out.append(f'  <text class="agent" x="{PAD}" y="{y + CELL_H / 2 + 4:.0f}">'
                   f'{agent}</text>')
        for index, column in enumerate(columns):
            m = rows[agent][index]
            x = PAD + LABEL_W + index * CELL_W
            klass = 'cell' if m['reflexive'] else 'left'
            out.append(f'  <rect class="{klass}" x="{x}" y="{y}" '
                       f'width="{CELL_W - 6}" height="{CELL_H - 8}" rx="3"/>')
            out.append(f'  <text class="n" x="{x + (CELL_W - 6) / 2:.0f}" '
                       f'y="{y + 20}" text-anchor="middle">'
                       f'|R| = {m["edges"]}</text>')
            frame_class = 'sub' if m['reflexive'] else 'sub bad'
            out.append(f'  <text class="{frame_class}" '
                       f'x="{x + (CELL_W - 6) / 2:.0f}" y="{y + 35}" '
                       f'text-anchor="middle">{m["frame"]}, '
                       f'{m["classes"]} class{"" if m["classes"] == 1 else "es"}</text>')

    foot = PAD + HEAD_H + len(agents) * CELL_H + 14
    out.append(f'  <text class="sub" x="{PAD}" y="{foot}">'
               '|R| counts the edges of the agent&#8217;s accessibility relation; '
               'classes counts the distinct sets of worlds it considers possible.</text>')
    out.append(f'  <text class="sub" x="{PAD}" y="{foot + 13}">'
               'A red cell marks a step at which the relation is no longer '
               'reflexive, so the frame has left S5 and the agent holds belief '
               'rather than knowledge.</text>')
    out.append('</svg>')
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--analysis', required=True)
    parser.add_argument('--agents', nargs='+', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--title', default='Each agent’s relation as a graph')
    args = parser.parse_args()

    analysis = json.load(open(args.analysis))
    # The initial state is recorded as the per-agent mapping directly; every
    # later step wraps the same mapping under "agents" beside its action.
    states = [(None, analysis['initial'])]
    states += [(s['action'], s['agents']) for s in analysis['steps']]

    columns, rows = [], {a: [] for a in args.agents}
    for index, (action, views) in enumerate(states):
        label = 'initial' if action is None else action.split('_')[0]
        columns.append({'label': label,
                        'step': '' if index == 0 else f'step {index}'})
        for agent in args.agents:
            structure = views[agent]['structure']
            rows[agent].append(properties(
                structure['relations'][agent], structure['worlds']))

    open(args.out, 'w').write(render(rows, columns, args.agents, args.title))
    print(f'{len(columns)} steps x {len(args.agents)} agents -> {args.out}')
    for agent in args.agents:
        print(f'  {agent}: ' + '  '.join(
            f'|R|={m["edges"]}/{m["frame"]}' for m in rows[agent]))


if __name__ == '__main__':
    main()
