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
Draws the epistemic side of the run: the model as it is updated, and the policy.

Neither is invented for the illustration. The world and designation counts are
the ones `epistemic_state` reported at each step of the recorded mission,

    applied goto-site_relay:            2 worlds, 2 designated
    applied scan_relay -> e-scan-dirty: 2 worlds, 1 designated
    applied relay-dirty_relay_scout:    3 worlds, 1 designated

and the policy is the four-node branching one the solver returned at depth 3.

    plot_epistemic.py --kripke k.svg --policy p.svg
"""

import argparse

# The palette of the published pages, so the figures sit in the same document.
INK, SLATE, RULE = '#111111', '#5A5A5A', '#D6D6D6'
STEEL, SIGNAL, PANEL = '#1A52A0', '#C8102E', '#FAFAFA'

STYLE = f"""  <style>
    .ttl {{ font-size:13px; font-weight:bold; fill:{INK} }}
    .lbl {{ font-size:11.5px; fill:{SLATE} }}
    .cap {{ font-size:11.5px; fill:{SLATE}; font-style:italic }}
    .wid {{ font-family:'Courier New',Courier,monospace; font-size:11px; fill:{INK} }}
    .ag  {{ font-family:'Courier New',Courier,monospace; font-size:10px; fill:{STEEL} }}
    .val {{ font-family:'Courier New',Courier,monospace; font-size:11.5px; fill:{INK} }}
  </style>"""


def world(cx, cy, name, val, designated, r=27):
    """A possible world. A designated one is ringed twice: it is a world the
    model says may be the actual one."""
    out = []
    if designated:
        out.append(f'<circle cx="{cx}" cy="{cy}" r="{r + 5}" fill="none" '
                   f'stroke="{SIGNAL}" stroke-width="1.6"/>')
    out.append(f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{PANEL}" '
               f'stroke="{INK}" stroke-width="1.3"/>')
    out.append(f'<text x="{cx}" y="{cy - 3}" class="val" text-anchor="middle">{val}</text>')
    out.append(f'<text x="{cx}" y="{cy + 11}" class="lbl" text-anchor="middle" '
               f'font-size="10">{name}</text>')
    return out


def link(x1, y1, x2, y2, agents, dash=False):
    """An accessibility edge, labelled by the agents that cannot tell the two
    worlds apart. S5, so the relation is an equivalence and drawn undirected."""
    d = ' stroke-dasharray="4 3"' if dash else ''
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    return [
        f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{STEEL}" '
        f'stroke-width="1.4"{d}/>',
        f'<rect x="{mx - 26}" y="{my - 9}" width="52" height="16" rx="3" '
        f'fill="#fff" stroke="none"/>',
        f'<text x="{mx}" y="{my + 3}" class="ag" text-anchor="middle">{agents}</text>',
    ]


def arrow(x1, y, x2, label, sub):
    mx = (x1 + x2) / 2
    return [
        f'<line x1="{x1}" y1="{y}" x2="{x2 - 8}" y2="{y}" stroke="{SLATE}" '
        f'stroke-width="1.2"/>',
        f'<polygon points="{x2},{y} {x2 - 9},{y - 4.5} {x2 - 9},{y + 4.5}" fill="{SLATE}"/>',
        f'<text x="{mx}" y="{y - 12}" class="wid" text-anchor="middle">{label}</text>',
        f'<text x="{mx}" y="{y + 20}" class="cap" text-anchor="middle">{sub}</text>',
    ]


def kripke(path):
    W, H = 900, 430
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
         f'width="{W}" height="{H}" font-family="Arial, Helvetica, sans-serif" '
         f'role="img" aria-label="The epistemic model at each step of the run: '
         f'two worlds both designated, then two with one designated after the '
         f'scan, then three after the private announcement">', STYLE]

    panels = [
        (30, 'Initial', '2 worlds, 2 designated',
         'Nobody knows whether the site is contaminated.'),
        (330, 'After scan_relay', '2 worlds, 1 designated',
         'Sensing narrows: the scan settles which world is actual.'),
        (630, 'After relay-dirty', '3 worlds, 1 designated',
         'Private speech widens: a world is added, not removed.'),
    ]
    for x, title, shape, cap in panels:
        s.append(f'<text x="{x}" y="26" class="ttl">{title}</text>')
        s.append(f'<text x="{x}" y="44" class="lbl">{shape}</text>')
        s.append(f'<text x="{x}" y="345" class="cap">{cap}</text>')

    # Panel 1: both worlds designated, every agent confuses them.
    s += link(90, 150, 90, 260, 's r o')
    s += world(90, 150, 'w1', '¬ c', True)
    s += world(90, 260, 'w2', 'c', True)

    # Panel 2: relay has sensed, so its relation separates them. The others
    # saw that a scan happened and not what it returned.
    s += link(390, 150, 390, 260, 's o')
    s += world(390, 150, 'w1', '¬ c', False)
    s += world(390, 260, 'w2', 'c', True)

    # Panel 3: the announcement is private, so the observer keeps a world in
    # which nothing was said. That world is the third.
    s += link(690, 150, 690, 260, 'o')
    s += link(690, 260, 800, 205, 'o')
    s += world(690, 150, 'w1', '¬ c', False)
    s += world(690, 260, 'w2', 'c', True)
    s += world(800, 205, 'w3', 'c', False)
    s.append(f'<text x="800" y="252" class="cap" text-anchor="middle" '
             f'font-size="10">nothing said</text>')

    s += arrow(150, 205, 300, 'scan_relay', 'semi-private sensing')
    s += arrow(450, 205, 600, 'relay-dirty', 'private announcement')

    s.append(f'<line x1="30" y1="368" x2="{W - 30}" y2="368" stroke="{RULE}"/>')
    s.append(f'<text x="30" y="386" class="lbl">'
             f'Double ring: a designated world. Edge label: the agents that '
             f'cannot tell the two worlds apart (s scout, r relay, o observer).'
             f'</text>')
    s.append('</svg>')
    open(path, 'w').write('\n'.join(s))
    print(f'{path}')


def node(x, y, w, text, sub, accent=INK):
    return [
        f'<rect x="{x}" y="{y}" width="{w}" height="46" rx="4" fill="{PANEL}" '
        f'stroke="{accent}" stroke-width="1.3"/>',
        f'<text x="{x + 12}" y="{y + 20}" class="wid">{text}</text>',
        f'<text x="{x + 12}" y="{y + 36}" class="lbl" font-size="10">{sub}</text>',
    ]


def policy(path):
    W, H = 900, 330
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
         f'width="{W}" height="{H}" font-family="Arial, Helvetica, sans-serif" '
         f'role="img" aria-label="The four-node branching policy: go to the '
         f'site, scan, and then one of two announcements according to what the '
         f'scan returned">', STYLE]
    s.append(f'<text x="30" y="26" class="ttl">The policy the solver returned</text>')
    s.append(f'<text x="30" y="44" class="lbl">'
             f'4 nodes, branching, depth 3 &#183; AO* expanded 40, generated 57</text>')

    s += node(30, 120, 200, 'goto-site_relay', 'public ontic')
    s += node(280, 120, 200, 'scan_relay', 'semi-private sensing')
    s += node(560, 70, 310, 'relay-dirty_relay_scout', 'private announcement')
    s += node(560, 190, 310, 'relay-clean_relay_scout', 'private announcement')

    s.append(f'<line x1="230" y1="143" x2="272" y2="143" stroke="{SLATE}" stroke-width="1.2"/>')
    s.append(f'<polygon points="280,143 271,138.5 271,147.5" fill="{SLATE}"/>')

    # The branch. This is the only place the plan is not a sequence, and the
    # thing it branches on is an observation rather than a state variable.
    s.append(f'<path d="M480 143 L520 143 L520 93 L552 93" fill="none" '
             f'stroke="{SIGNAL}" stroke-width="1.3"/>')
    s.append(f'<polygon points="560,93 551,88.5 551,97.5" fill="{SIGNAL}"/>')
    s.append(f'<path d="M480 143 L520 143 L520 213 L552 213" fill="none" '
             f'stroke="{SIGNAL}" stroke-width="1.3"/>')
    s.append(f'<polygon points="560,213 551,208.5 551,217.5" fill="{SIGNAL}"/>')
    # Clear of the boxes: the arrowheads land at x=560, so the labels sit to
    # the left of the fork rather than over what they point at.
    s.append(f'<text x="552" y="84" class="ag" fill="{SIGNAL}" '
             f'text-anchor="end">e-scan-dirty &#8594;</text>')
    s.append(f'<text x="552" y="204" class="ag" fill="{SIGNAL}" '
             f'text-anchor="end">e-scan-clean &#8594;</text>')

    s.append(f'<line x1="30" y1="280" x2="{W - 30}" y2="280" stroke="{RULE}"/>')
    s.append(f'<text x="30" y="298" class="lbl">'
             f'The branch is taken on what the scan observed, not on a state '
             f'variable. The recorded run took the upper arm.</text>')
    s.append(f'<text x="30" y="315" class="lbl">'
             f'Both arms announce privately: broadcasting satisfies the first '
             f'two conjuncts of the goal and destroys the third.</text>')
    s.append('</svg>')
    open(path, 'w').write('\n'.join(s))
    print(f'{path}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--kripke', required=True)
    ap.add_argument('--policy', required=True)
    a = ap.parse_args()
    kripke(a.kripke)
    policy(a.policy)
