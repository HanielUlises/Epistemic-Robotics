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
Draws the figures for a run report from the measured analysis.

Everything here is computed from `analyse_run.py` output, so a figure cannot
drift from the run it describes: the world counts, the accessibility arcs and
the frame property are the ones the epistemic state reported when the actions
were replayed.

    figures.py --analysis out/analysis-l3.json --prefix out/fig- --agents ...
"""

import argparse
import json
import math

PALETTE = {
    'ink': '#111111', 'red': '#C8102E', 'gray': '#5A5A5A',
    'border': '#D6D6D6', 'panel': '#FAFAFA', 'steel': '#1A52A0',
}

HEAD = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
        'width="{w}" height="{h}" font-family="Arial, Helvetica, sans-serif" '
        'role="img" aria-label="{alt}">')

STYLE = """<style>
  .lbl  {{ font-size:11.5px; fill:{gray} }}
  .ttl  {{ font-size:13px; font-weight:bold; fill:{ink} }}
  .cap  {{ font-size:11.5px; fill:{gray}; font-style:italic }}
  .wid  {{ font-family:'Courier New',Courier,monospace; font-size:11px; fill:{ink} }}
  .atom {{ font-family:'Courier New',Courier,monospace; font-size:9.5px; fill:{gray} }}
  .atomh{{ font-family:'Courier New',Courier,monospace; font-size:9.5px;
           fill:none; stroke:{panel}; stroke-width:3; stroke-linejoin:round }}
  .node {{ fill:#FFFFFF; stroke:{gray}; stroke-width:1.4 }}
  .des  {{ fill:#FFFFFF; stroke:{red}; stroke-width:2.6 }}
  .act  {{ fill:#FDECEF; stroke:{red}; stroke-width:2.6 }}
  .arc  {{ stroke:{steel}; stroke-width:1.3; fill:none; marker-end:url(#a) }}
  .arc2 {{ stroke:{steel}; stroke-width:1.3; fill:none;
           marker-end:url(#a); marker-start:url(#b) }}
  .loop {{ stroke:{steel}; stroke-width:1.3; fill:none; marker-end:url(#a) }}
  .axis {{ stroke:{border}; stroke-width:1 }}
</style>
<defs>
  <marker id="a" viewBox="0 0 10 10" refX="9.5" refY="5" markerWidth="6"
    markerHeight="6" orient="auto">
    <path d="M0,0 L10,5 L0,10 z" fill="{steel}"/></marker>
  <marker id="b" viewBox="0 0 10 10" refX="0.5" refY="5" markerWidth="6"
    markerHeight="6" orient="auto">
    <path d="M10,0 L0,5 L10,10 z" fill="{steel}"/></marker>
</defs>""".format(**PALETTE)


def short(atom):
    """`leak-at_l3_suite` reads as `leak@l3` at figure size."""
    return (atom.replace('leak-at_', 'leak@').replace('contained_', 'sealed@')
                .replace('pallet-at_', 'pallet@').replace('_suite', '')
                .replace('delivered', 'delivered').replace('safe', 'safe'))


def kripke_panel(x0, y0, width, height, agent, step, out):
    """One agent's accessibility digraph over the worlds of one step."""
    graph = step['agents'][agent]['structure']
    worlds = graph.get('worlds', [])
    if not worlds:
        return
    designated = set(graph.get('designated', []))
    relation = graph.get('relations', {}).get(agent, {})
    labels = graph.get('labels', {})
    varying = graph.get('varying_atoms', [])

    # Worlds on a circle, which keeps every arc visible however many there are.
    radius = min(width, height) * 0.33
    cx, cy = x0 + width / 2, y0 + height / 2 + 6
    place = {}
    for i, w in enumerate(worlds):
        angle = -math.pi / 2 + 2 * math.pi * i / len(worlds)
        place[w] = (cx + radius * math.cos(angle), cy + radius * math.sin(angle))

    reflexive = step['agents'][agent]['reflexive']
    actual = step['agents'][agent]['includes_actual']
    note = 'reflexive' if reflexive else 'not reflexive'
    if not actual:
        note += ', excludes the actual world'
    out.append(f'<text class="ttl" x="{x0 + 8:.0f}" y="{y0 + 14:.0f}">{agent}</text>')
    out.append(f'<text class="lbl" x="{x0 + 8:.0f}" y="{y0 + 30:.0f}">{note}</text>')

    NODE_R = 15.0
    GAP = 3.5

    def trimmed(a, b):
        """The segment between two nodes, cut short of both circles.

        Drawing centre to centre puts the arrowhead underneath the target node,
        where it cannot be seen.
        """
        (ax, ay), (bx, by) = a, b
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length <= 2 * (NODE_R + GAP):
            return None
        ux, uy = dx / length, dy / length
        off = NODE_R + GAP
        return (ax + ux * off, ay + uy * off, bx - ux * off, by - uy * off)

    drawn = set()
    for w, targets in relation.items():
        if w not in place:
            continue
        for t in targets:
            if t not in place:
                continue

            if w == t:
                # A loop that leaves the circle and returns to it. Both ends sit
                # exactly on the boundary, so the arrowhead meets the node
                # instead of floating above it.
                px, py = place[w]
                # Outward from the middle of the panel. Every other arc runs
                # toward another node, so the radial direction away from the
                # centre is the one bearing on which nothing else is drawn.
                px, py = place[w]
                out_ang = math.atan2(py - cy, px - cx) if (px, py) != (cx, cy) \
                    else -math.pi / 2
                a0, a1 = out_ang - math.radians(38), out_ang + math.radians(38)
                rim = NODE_R + 2.0
                sx, sy = px + rim * math.cos(a0), py + rim * math.sin(a0)
                ex, ey = px + rim * math.cos(a1), py + rim * math.sin(a1)
                bulge = NODE_R + 22
                c1x = px + bulge * math.cos(a0 - math.radians(10))
                c1y = py + bulge * math.sin(a0 - math.radians(10))
                c2x = px + bulge * math.cos(a1 + math.radians(10))
                c2y = py + bulge * math.sin(a1 + math.radians(10))
                out.append(
                    f'<path class="loop" d="M{sx:.1f},{sy:.1f} '
                    f'C{c1x:.1f},{c1y:.1f} {c2x:.1f},{c2y:.1f} {ex:.1f},{ey:.1f}"/>')
                continue

            pair = tuple(sorted((w, t)))
            if pair in drawn:
                continue
            segment = trimmed(place[w], place[t])
            if segment is None:
                continue
            ax, ay, bx, by = segment
            # One line carries both directions when the relation is symmetric
            # here, which keeps two arrowheads off the same stretch of ink.
            mutual = w in relation.get(t, [])
            cls = 'arc2' if mutual else 'arc'
            if mutual:
                drawn.add(pair)
            out.append(f'<path class="{cls}" d="M{ax:.1f},{ay:.1f} '
                       f'L{bx:.1f},{by:.1f}"/>')

    for w in worlds:
        px, py = place[w]
        cls = 'des' if w in designated else 'node'
        out.append(f'<circle class="{cls}" cx="{px:.0f}" cy="{py:.0f}" r="15"/>')
        out.append(f'<text class="wid" x="{px:.0f}" y="{py + 4:.0f}" '
                   f'text-anchor="middle">{w}</text>')
        # One atom per line. Two on one line reaches into the neighbouring
        # node at this radius.
        true_here = [short(a) for a in varying if a in labels.get(w, [])]
        for k, atom in enumerate(true_here[:2]):
            # Halo first, so an edge crossing the label does not obscure it.
            for cls in ('atomh', 'atom'):
                out.append(f'<text class="{cls}" x="{px:.0f}" '
                           f'y="{py + 29 + k * 11:.0f}" '
                           f'text-anchor="middle">{atom}</text>')


def kripke_figure(analysis, agents, path, steps_wanted):
    steps = [('initial', analysis['initial'])] + [
        (s['action'], s['agents']) for s in analysis['steps']]
    chosen = [steps[i] for i in steps_wanted]

    pw, ph = 272, 226
    width = pw * len(agents) + 40
    height = ph * len(chosen) + 60

    out = [HEAD.format(w=width, h=height,
                       alt='Accessibility relations per agent at each step'), STYLE]
    for r, (label, snapshot) in enumerate(chosen):
        y0 = 30 + r * ph
        out.append(f'<text class="ttl" x="12" y="{y0 - 8:.0f}">'
                   f'after {label[:58]}</text>')
        for c, agent in enumerate(agents):
            x0 = 20 + c * pw
            out.append(f'<rect x="{x0}" y="{y0}" width="{pw - 12}" '
                       f'height="{ph - 26}" rx="3" fill="{PALETTE["panel"]}" '
                       f'stroke="{PALETTE["border"]}"/>')
            kripke_panel(x0, y0, pw - 12, ph - 26, agent,
                         {'agents': snapshot}, out)
    out.append(f'<text class="cap" x="12" y="{height - 12}">'
               'Nodes are worlds, arcs the agent\'s accessibility relation, '
               'red rings the worlds it designates. A missing self-loop is a '
               'world the agent excludes while it is the case.</text>')
    out.append('</svg>')
    open(path, 'w').write('\n'.join(out))
    print(f'  {path}')


def trajectory_figure(analysis, agents, path):
    """The run as a table: model size, and each agent's state after every step.

    The three agents share one model, so plotting |W| once per agent draws the
    same line three times. What separates them is the number of worlds each
    still designates, which is the difference between knowing and not, and
    whether the perspective still contains the world that is the case.
    """
    rows = [('initial', analysis['initial'])] + [
        (s['action'], s['agents']) for s in analysis['steps']]
    n = len(rows)

    col_w, row_h = 148, 46
    left, top = 132, 74
    width = left + col_w * n + 24
    height = top + row_h * (len(agents) + 1) + 92

    out = [HEAD.format(w=width, h=height,
                       alt='Model size and each agent\'s epistemic state after every step'),
           STYLE]

    for i, (label, _) in enumerate(rows):
        x = left + col_w * i
        text = 'initial' if i == 0 else label.split('_')[0]
        out.append(f'<text class="lbl" x="{x + col_w / 2:.0f}" y="{top - 34:.0f}" '
                   f'text-anchor="middle">{text[:16]}</text>')
        if i:
            out.append(f'<text class="atom" x="{x + col_w / 2:.0f}" '
                       f'y="{top - 20:.0f}" text-anchor="middle">step {i}</text>')

    # The shared model.
    out.append(f'<text class="ttl" x="12" y="{top + 26:.0f}">model</text>')
    out.append(f'<text class="lbl" x="12" y="{top + 40:.0f}">worlds</text>')
    for i, (_, snapshot) in enumerate(rows):
        x = left + col_w * i
        worlds = snapshot[agents[0]]['worlds']
        out.append(f'<rect x="{x + 6}" y="{top + 8}" width="{col_w - 12}" '
                   f'height="{row_h - 14}" rx="3" fill="{PALETTE["panel"]}" '
                   f'stroke="{PALETTE["border"]}"/>')
        out.append(f'<text class="wid" x="{x + col_w / 2:.0f}" y="{top + 32:.0f}" '
                   f'text-anchor="middle">|W| = {worlds}</text>')

    for r, agent in enumerate(agents):
        y = top + row_h * (r + 1)
        out.append(f'<text class="ttl" x="12" y="{y + 26:.0f}">{agent}</text>')
        for i, (_, snapshot) in enumerate(rows):
            entry = snapshot[agent]
            x = left + col_w * i
            knows = entry['designated'] == 1
            wrong = not entry['includes_actual']
            fill = '#FDECEF' if wrong else ('#FFFFFF' if knows else PALETTE['panel'])
            stroke = PALETTE['red'] if wrong else (
                PALETTE['ink'] if knows else PALETTE['border'])
            out.append(f'<rect x="{x + 6}" y="{y + 8}" width="{col_w - 12}" '
                       f'height="{row_h - 14}" rx="3" fill="{fill}" '
                       f'stroke="{stroke}" stroke-width="{2 if wrong else 1.2}"/>')
            state = 'knows' if knows else 'uncertain'
            if wrong:
                state = 'believes wrongly'
            out.append(f'<text class="wid" x="{x + col_w / 2:.0f}" '
                       f'y="{y + 26:.0f}" text-anchor="middle">|D| = '
                       f"{entry['designated']}</text>")
            colour = PALETTE['red'] if wrong else PALETTE['gray']
            out.append(f'<text class="atom" x="{x + col_w / 2:.0f}" '
                       f'y="{y + 38:.0f}" text-anchor="middle" fill="{colour}">'
                       f'{state}</text>')

    base = top + row_h * (len(agents) + 1)
    out.append(f'<text class="cap" x="12" y="{base + 28}">'
               '|W| is the size of the model the fleet shares; |D| is the number of '
               'worlds an agent still designates, so |D| = 1 is knowledge and |D| &gt; 1 '
               'is uncertainty.</text>')
    out.append(f'<text class="cap" x="12" y="{base + 46}">'
               'A red cell marks a perspective that no longer contains the world that '
               'is the case, which is where the accessibility relation stops being '
               'reflexive and the frame</text>')
    out.append(f'<text class="cap" x="12" y="{base + 64}">'
               'ceases to be S5.</text>')
    out.append('</svg>')
    open(path, 'w').write('\n'.join(out))
    print(f'  {path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--analysis', required=True)
    parser.add_argument('--agents', nargs='+', required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--kripke-steps', nargs='+', type=int, default=None,
                        help='indices into [initial, step 1, ...] to draw')
    args = parser.parse_args()

    analysis = json.load(open(args.analysis))
    total = len(analysis['steps']) + 1
    wanted = args.kripke_steps or list(range(total))
    wanted = [i for i in wanted if 0 <= i < total]

    print('figures:')
    trajectory_figure(analysis, args.agents, args.prefix + 'trajectory.svg')
    kripke_figure(analysis, args.agents, args.prefix + 'kripke.svg', wanted)


if __name__ == '__main__':
    main()
