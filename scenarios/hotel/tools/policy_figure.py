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
Draws the policy the planner produced, as the tree it is.

A conditional plan is an AND-OR tree: an action, and one continuation per
event the action can produce. Aletheia writes it out in that shape, so the
figure is read straight off the plan rather than redrawn by hand, and it
cannot drift from what the executor ran.

    policy_figure.py --plan out/warehouse-problem-plan.json \\
                     --out docs/img/warehouse-rmf-policy.svg \\
                     --branch-label 0 "pallet here" --branch-label 1 "aisle empty" \\
                     --taken 0

`--taken` marks the branch the recorded run went down, so the tree, the film
and the table describe one execution.
"""

import argparse
import json

ROW = 58
COL = 210
PAD_X = 40
PAD_TOP = 54
BOX_W = 186
BOX_H = 30

SENSING = ('inspect', 'look_into')
ONTIC = ('pickup', 'unload', 'contain', 'drop', 'pick')


def kind(action):
    head = action.split('_')[0]
    if head in SENSING:
        return 'sense'
    if head in ONTIC:
        return 'ontic'
    return 'move'


def layout(node, depth, column, out, edges, parent=None, event=None):
    """Place every node, spreading a branch point into adjacent columns."""
    index = len(out)
    out.append({'action': node['action'], 'depth': depth, 'column': column})
    if parent is not None:
        edges.append((parent, index, event))

    children = [b for b in node.get('branches', []) if b.get('subtree')]
    if len(children) == 1:
        layout(children[0]['subtree'], depth + 1, column, out, edges,
               index, children[0].get('event'))
        return

    # A branch point: one column each side of this node's own column.
    offsets = [-1, 1] if len(children) == 2 else range(len(children))
    for child, offset in zip(children, offsets):
        layout(child['subtree'], depth + 1, column + offset, out, edges,
               index, child.get('event'))


def render(nodes, edges, labels, taken, title):
    columns = [n['column'] for n in nodes]
    lo, hi = min(columns), max(columns)
    width = (hi - lo) * COL + BOX_W + 2 * PAD_X
    height = (max(n['depth'] for n in nodes)) * ROW + BOX_H + PAD_TOP + 30

    def cx(node):
        return PAD_X + (node['column'] - lo) * COL + BOX_W / 2

    def cy(node):
        return PAD_TOP + node['depth'] * ROW

    # Which nodes lie on the branch the run actually took.
    on_taken = set()
    if taken is not None:
        by_parent = {}
        for parent, child, event in edges:
            by_parent.setdefault(parent, []).append((child, event))
        stack = [0]
        while stack:
            here = stack.pop()
            on_taken.add(here)
            kids = by_parent.get(here, [])
            if len(kids) == 1:
                stack.append(kids[0][0])
            else:
                for child, event in kids:
                    if event == taken:
                        stack.append(child)

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width:.0f} {height:.0f}"',
        f'     width="{width:.0f}" height="{height:.0f}"',
        '     font-family="Arial, Helvetica, sans-serif" role="img"',
        f'     aria-label="{title}">',
        '  <style>',
        '    .box   { fill:#FFFFFF; stroke:#D6D6D6; stroke-width:1.4 }',
        '    .sense { stroke:#C8102E; stroke-width:2 }',
        '    .ontic { stroke:#111111; stroke-width:2 }',
        '    .off   { opacity:.34 }',
        "    .act   { font-family:'Courier New',Courier,monospace; font-size:12px;",
        '             fill:#111111 }',
        '    .edge  { stroke:#9A9A9A; stroke-width:1.3; fill:none; marker-end:url(#a) }',
        '    .took  { stroke:#5A5A5A; stroke-width:1.8 }',
        '    .brlab { font-size:11.5px; fill:#C8102E; font-weight:bold }',
        '  </style>',
        '  <defs>',
        '    <marker id="a" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7"',
        '            markerHeight="7" orient="auto">',
        '      <path d="M0,0 L8,4 L0,8 z" fill="#9A9A9A"/>',
        '    </marker>',
        '  </defs>',
    ]

    fanout = {}
    for parent, _, _ in edges:
        fanout[parent] = fanout.get(parent, 0) + 1

    for parent, child, event in edges:
        a, b = nodes[parent], nodes[child]
        x1, y1 = cx(a), cy(a) + BOX_H / 2
        x2, y2 = cx(b), cy(b) - BOX_H / 2 - 6
        classes = 'edge' + (' took' if child in on_taken else ' off')
        mid = (y1 + y2) / 2
        if abs(x1 - x2) < 1:
            out.append(f'  <path class="{classes}" d="M{x1:.0f},{y1:.0f} L{x2:.0f},{y2:.0f}"/>')
        else:
            out.append(f'  <path class="{classes}" d="M{x1:.0f},{y1:.0f} '
                       f'C{x1:.0f},{mid:.0f} {x2:.0f},{mid:.0f} {x2:.0f},{y2:.0f}"/>')
        if event in labels and fanout.get(parent, 0) > 1:
            out.append(f'  <text class="brlab" x="{(x1 + x2) / 2:.0f}" '
                       f'y="{mid - 4:.0f}" text-anchor="middle">{labels[event]}</text>')

    for index, node in enumerate(nodes):
        x, y = cx(node) - BOX_W / 2, cy(node) - BOX_H / 2
        shape = kind(node['action'])
        classes = 'box' + (f' {shape}' if shape != 'move' else '')
        dim = ' class="off"' if taken is not None and index not in on_taken else ''
        out.append(f'  <g{dim}>')
        out.append(f'    <rect class="{classes}" x="{x:.0f}" y="{y:.0f}" '
                   f'width="{BOX_W}" height="{BOX_H}" rx="4"/>')
        out.append(f'    <text class="act" x="{cx(node):.0f}" y="{cy(node) + 4:.0f}" '
                   f'text-anchor="middle">{node["action"]}</text>')
        out.append('  </g>')

    out.append('</svg>')
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--branch-label', nargs=2, action='append', default=[],
                        metavar=('EVENT', 'TEXT'))
    parser.add_argument('--taken', type=int, default=None,
                        help='event index the recorded run took')
    parser.add_argument('--title', default='The policy as a tree')
    args = parser.parse_args()

    plan = json.load(open(args.plan))
    nodes, edges = [], []
    layout(plan, 0, 0, nodes, edges)
    labels = {int(e): t for e, t in args.branch_label}
    open(args.out, 'w').write(
        render(nodes, edges, labels, args.taken, args.title))
    print(f'{len(nodes)} nodes -> {args.out}')


if __name__ == '__main__':
    main()
