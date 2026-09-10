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
Draws the multi-site policy, from the policy.

Nothing here is laid out by hand. `survey_sites_mission` writes what the
planner returned when `policy_out:=` names a file, and this reads that: the
actions, the branches, the outcomes each branch is taken on, and which nodes
are sensing actions. A figure drawn from a description of a policy is a claim
about it; this one is the policy.

    survey_sites_mission ... -p policy_out:=/tmp/policy.json
    plot_sites.py --policy /tmp/policy.json --out policy.svg
"""

import argparse
import json
import re

# The palette of the published pages, so the figures sit in the same document.
INK, SLATE, RULE = '#111111', '#5A5A5A', '#D6D6D6'
STEEL, SIGNAL, PANEL = '#1A52A0', '#C8102E', '#FAFAFA'

DONE = 4294967295

# The tree grows downward. Laid out left to right it is six columns wide and
# three rows tall, which on a page 1040 px across leaves the type at half its
# size; downward it is three columns and six rows, which is the shape of a
# page.
BOX_W, BOX_H = 232, 46
GAP_X, GAP_Y = 34, 46
TOP = 78

STYLE = f"""  <style>
    .ttl {{ font-size:13px; font-weight:bold; fill:{INK} }}
    .lbl {{ font-size:11.5px; fill:{SLATE} }}
    .wid {{ font-family:'Courier New',Courier,monospace; font-size:11.5px; fill:{INK} }}
    .ag  {{ font-family:'Courier New',Courier,monospace; font-size:10px; fill:{SIGNAL} }}
    .sm  {{ font-size:10px; fill:{SLATE} }}
  </style>"""


def layout(items):
    """Give every node a column and a row.

    Column is depth: how many actions have run before this one. Row is packed,
    a leaf at a time, so two arms of a branch never overlap however deep either
    of them goes. Laying it out by hand would mean re-laying it out whenever
    the planner returned something else, which is exactly when a figure stops
    being checked.
    """
    place, rows = {}, [0]

    def walk(index, depth):
        item = items[index]
        kids = [c for c in item['children'] if c != DONE]
        if not kids:
            row = rows[0]
            rows[0] += 1
        else:
            spans = [walk(c, depth + 1) for c in kids]
            row = sum(spans) / len(spans)
        place[index] = (depth, row)
        return row

    walk(0, 0)
    return place


def draw(items, place, index, out, seen):
    if index in seen:
        return
    seen.add(index)
    item = items[index]
    depth, row = place[index]
    x = 30 + row * (BOX_W + GAP_X)
    y = TOP + depth * (BOX_H + GAP_Y)

    accent = SIGNAL if item['sensing'] else INK
    out.append(
        f'<rect x="{x}" y="{y}" width="{BOX_W}" height="{BOX_H}" rx="4" '
        f'fill="{PANEL}" stroke="{accent}" stroke-width="1.3"/>')
    out.append(f'<text x="{x + 11}" y="{y + 19}" class="wid">{item["action"]}</text>')
    out.append(
        f'<text x="{x + 11}" y="{y + 34}" class="sm">'
        f'{"sensing &#183; the branch is here" if item["sensing"] else item["epistemic_action"]}'
        f'</text>')

    kids = [(c, item['outcomes'][i] if i < len(item['outcomes']) else '?')
            for i, c in enumerate(item['children'])]

    for child, outcome in kids:
        if child == DONE:
            out.append(
                f'<text x="{x + BOX_W / 2}" y="{y + BOX_H + 20}" class="sm" '
                f'text-anchor="middle" fill="{SLATE}">&#8595; goal reached</text>')
            continue
        cd, cr = place[child]
        cx = 30 + cr * (BOX_W + GAP_X)
        cy = TOP + cd * (BOX_H + GAP_Y)
        mid = y + BOX_H + GAP_Y / 2
        colour = SIGNAL if item['sensing'] else SLATE
        out.append(
            f'<path d="M{x + BOX_W / 2} {y + BOX_H} L{x + BOX_W / 2} {mid} '
            f'L{cx + BOX_W / 2} {mid} L{cx + BOX_W / 2} {cy - 8}" fill="none" '
            f'stroke="{colour}" stroke-width="1.3"/>')
        out.append(
            f'<polygon points="{cx + BOX_W / 2},{cy} {cx + BOX_W / 2 - 4.5},{cy - 9} '
            f'{cx + BOX_W / 2 + 4.5},{cy - 9}" fill="{colour}"/>')
        if item['sensing']:
            out.append(
                f'<text x="{cx + BOX_W / 2 + 8}" y="{cy - 13}" class="ag">'
                f'{outcome}</text>')
        draw(items, place, child, out, seen)


WORLD_R = 26


def applied(log_path):
    """The shape of the model after each action, read from the run's own log.

    `epistemic_state` prints a line per product update saying how many worlds
    the model has and how many of them are designated. Those are the numbers
    the figure draws. Counting them again here, or writing them out by hand,
    would be a second account of the run that could disagree with the first.
    """
    line = re.compile(
        r'applied (\S+?)(?: -> (\S+))?:\s+(\d+) worlds?, (\d+) designated')
    steps = []
    with open(log_path) as handle:
        for row in handle:
            found = line.search(row)
            if found:
                action, outcome, worlds, designated = found.groups()
                steps.append((action, outcome, int(worlds), int(designated)))
    return steps


def kripke(steps, out_path, caption):
    """Draw the model as the run reported it, one panel per product update.

    Worlds go in a grid rather than a row. The model ends at eight worlds and a
    row of eight at a legible radius is wider than the panel, so a row layout
    silently draws each panel over the next one -- which reads as a figure and
    is a mess.
    """
    panels = [('initially', steps[0][2], steps[0][3])] if steps else []
    for action, outcome, worlds, designated in steps:
        label = action if not outcome else f'{action} &#8594; {outcome}'
        panels.append((label, worlds, designated))

    cols = 3
    cell = 34
    pw, gap = 214, 26
    widest = max(w for _, w, _ in panels) if panels else 1
    rows = -(-widest // cols)

    # Four panels to a line. Seven in a row is 1 700 px, and a figure set to
    # the width of a 1 040 px column then renders its labels at six points.
    per_line = 4
    lines = -(-len(panels) // per_line)
    top = 96
    band = 34 + rows * cell + 30
    rule_y = top + lines * band - 12
    width = 30 + min(len(panels), per_line) * (pw + gap) - gap + 30
    height = rule_y + 30 + 17 * len(caption)

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
         f'width="{width}" height="{height}" '
         f'font-family="Arial, Helvetica, sans-serif" role="img" '
         f'aria-label="The epistemic model at each product update, as worlds '
         f'and designated worlds">',
         STYLE,
         '<text x="30" y="30" class="ttl">The model at each product update</text>',
         '<text x="30" y="48" class="lbl">Counts as the epistemic state '
         'reported them during the run</text>']

    for i, (label, worlds, designated) in enumerate(panels):
        line, slot = divmod(i, per_line)
        x = 30 + slot * (pw + gap)
        y0 = top + line * band
        s.append(f'<text x="{x}" y="{y0 - 24}" class="wid" font-size="9.5">'
                 f'{label}</text>')
        s.append(f'<text x="{x}" y="{y0 - 10}" class="lbl" font-size="10">'
                 f'{worlds} world{"s" if worlds != 1 else ""}, '
                 f'{designated} designated</text>')
        for w in range(worlds):
            cx = x + 16 + (w % cols) * cell
            cy = y0 + 14 + (w // cols) * cell
            if w < designated:
                s.append(f'<circle cx="{cx}" cy="{cy}" r="14.5" fill="none" '
                         f'stroke="{SIGNAL}" stroke-width="1.6"/>')
            s.append(f'<circle cx="{cx}" cy="{cy}" r="10" fill="{PANEL}" '
                     f'stroke="{INK}" stroke-width="1.2"/>')
        if i + 1 < len(panels) and slot + 1 < per_line:
            ax = x + pw - 4
            ay = y0 + 14
            s.append(f'<line x1="{ax}" y1="{ay}" x2="{ax + gap - 12}" y2="{ay}" '
                     f'stroke="{SLATE}" stroke-width="1.2"/>')
            s.append(f'<polygon points="{ax + gap - 4},{ay} {ax + gap - 13},{ay - 4.5} '
                     f'{ax + gap - 13},{ay + 4.5}" fill="{SLATE}"/>')

    s.append(f'<line x1="30" y1="{rule_y}" x2="{width - 30}" y2="{rule_y}" '
             f'stroke="{RULE}"/>')
    s.append(f'<text x="30" y="{rule_y + 18}" class="lbl">Red ring: a designated '
             f'world &#8212; one the model says may be the actual one.</text>')
    for n, part in enumerate(caption):
        s.append(f'<text x="30" y="{rule_y + 35 + n * 17}" class="lbl">{part}</text>')
    s.append('</svg>')
    with open(out_path, 'w') as handle:
        handle.write('\n'.join(s))
    print(f'{out_path}: {len(panels)} panels, up to {widest} worlds')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', required=True,
                    help='what survey_sites_mission wrote with policy_out:=')
    ap.add_argument('--out', required=True)
    ap.add_argument('--log', help='a run log, for the model figure')
    ap.add_argument('--kripke-out', help='where to write the model figure')
    ap.add_argument('--expanded', type=int, default=None,
                    help='AO* expansions, from the planner log, for the '
                         'subtitle. Left out when not given rather than '
                         'guessed at.')
    ap.add_argument('--depth', type=int, default=None)
    args = ap.parse_args()

    with open(args.policy) as handle:
        policy = json.load(handle)
    items = policy['items']
    place = layout(items)

    levels = max(d for d, _ in place.values()) + 1
    branches = max(r for _, r in place.values()) + 1
    width = 30 + branches * (BOX_W + GAP_X) + 60
    height = TOP + levels * (BOX_H + GAP_Y) + 62

    leaves = sum(1 for it in items for c in it['children'] if c == DONE)
    subtitle = f'{len(items)} nodes, {leaves} leaves'
    if args.depth is not None:
        subtitle += f', depth {args.depth}'
    if args.expanded is not None:
        subtitle += f' &#183; AO* expanded {args.expanded:,}'.replace(',', '&#8202;')

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width:.0f} '
        f'{height:.0f}" width="{width:.0f}" height="{height:.0f}" '
        f'font-family="Arial, Helvetica, sans-serif" role="img" '
        f'aria-label="The multi-site policy: two robots are sent to two of the '
        f'three sites, each scan branches on what its laser returned, and every '
        f'leaf is a private announcement">',
        STYLE,
        '<text x="30" y="30" class="ttl">The policy the solver returned</text>',
        f'<text x="30" y="48" class="lbl">{subtitle}</text>',
    ]
    draw(items, place, 0, out, set())

    rule_y = height - 46
    out.append(f'<line x1="30" y1="{rule_y}" x2="{width - 30:.0f}" y2="{rule_y}" '
               f'stroke="{RULE}"/>')
    out.append(
        f'<text x="30" y="{rule_y + 18}" class="lbl">Red outline: a sensing '
        f'action, and the only place the plan is not a sequence. The branch is '
        f'taken on what the laser returned, not on a state variable.</text>')
    out.append(
        f'<text x="30" y="{rule_y + 35}" class="lbl">Every leaf announces '
        f'privately: a broadcast satisfies the first two conjuncts of the goal '
        f'and destroys the third.</text>')
    out.append('</svg>')

    with open(args.out, 'w') as handle:
        handle.write('\n'.join(out))
    print(f'{args.out}: {len(items)} nodes, {leaves} leaves, {levels} deep')

    if args.log and args.kripke_out:
        steps = applied(args.log)
        if not steps:
            raise SystemExit(
                f'{args.log} records no product update. The figure would be an '
                f'illustration rather than the run, so none is written.')
        kripke(steps, args.kripke_out, [
            'Sensing contracts the model: an observation rules a world out.',
            'Private speech expands it: the agents outside the audience must go '
            'on considering possible a world in which nothing was said.',
        ])


if __name__ == '__main__':
    main()
