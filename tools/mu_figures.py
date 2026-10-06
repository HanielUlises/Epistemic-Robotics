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
Draws the fixed-point figures of the README, in the style of its first three.

    tools/mu_figures.py --models <models.json> --out docs/img

Every region is computed here by the iteration mu_path_planner's mu_reach
documents, on a small floor written out below:

    Z_0 = {},   Z_(k+1) = goal  u  { v in Safe | some u in Z_k with v -> u }

over 4-connected cells, until Z_(k+1) = Z_k. A cell that has never been
observed is in no Safe set. The route is read off the result by descending the
iteration at which each cell entered.

The last figure is the other fixed point the project depends on. Common
knowledge is the greatest fixed point

    C_G phi  =  nu X . E_G (phi and X),

and its approximants from above are X_0 = W, X_(k+1) = [[E_G (phi and X_k)]].
They are drawn on the model the coordinated attack's radio protocol leaves after
four delivered messages, read from coordinated_attack_demo's export_models.py
output (--models).

  mu-approximants.png    Z_1 ... Z_* on the floor of Figure 1, sensed
  mu-iteration.png       the winning region coloured by iteration, and the route
  mu-epistemic.png       Safe without and with K open(t2), on a three-bay floor
  mu-common-knowledge.png  X_0 ... X_5 on the ten-world model
"""

import argparse
import json
import os

from PIL import Image, ImageDraw, ImageFont

# The palette of the existing figures.
WALL = (38, 38, 38)
WALL_LINE = (71, 71, 71)
FLOOR = (238, 241, 240)
FLOOR_LINE = (231, 234, 233)
REGION = (0, 114, 178)
REGION_LINE = (41, 132, 183)
GOAL = (0, 158, 115)
ROBOT = (213, 94, 0)
UNKNOWN = (156, 149, 140)
UNKNOWN_TEXT = (125, 118, 109)
LIFTED = (86, 180, 233)
INK = (17, 17, 17)
GRAY = (90, 90, 90)
WHITE = (255, 255, 255)
SOUTH = (51, 115, 204)
NORTH = (230, 128, 38)
RED = (200, 16, 46)

DEJAVU = '/usr/share/fonts/truetype/dejavu'
SUB = str.maketrans('0123456789', '\u2080\u2081\u2082\u2083\u2084\u2085\u2086\u2087\u2088\u2089')


def plural(n, word):
    return f'{n} {word}' + ('' if n == 1 else 's')


def font(size, bold=False):
    return ImageFont.truetype(f'{DEJAVU}/DejaVuSans{"-Bold" if bold else ""}.ttf', size)


# Figure 1's floor: a room above, a gap in the wall, a corridor of unobserved
# cells below it, and the robot in the room beneath.
SENSING = [
    '##################',
    '#................#',
    '#................#',
    '#.............G..#',
    '#................#',
    '########..########',
    '#.......??.......#',
    '#.......??.......#',
    '#.......??.......#',
    '#....R...........#',
    '#................#',
    '#................#',
    '##################',
]

# Three bays through a block; the open one is t2. In the plan the bays are
# unknown, and the loads in t1 and t3 are the world's, not the plan's.
BAYS = [
    '##################',
    '#...........D....#',
    '#................#',
    '#................#',
    '#................#',
    '##11####22####33##',
    '##11####22####33##',
    '#................#',
    '#................#',
    '#................#',
    '#.......R........#',
    '#................#',
    '##################',
]


def parse(rows):
    cells = {}
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            cells[(r, c)] = ch
    return cells


def least_fixed_point(cells, goal, safe):
    """mu_reach's iteration. Returns the order of entry: cell -> k."""
    entered = {}
    z = set()
    k = 0
    while True:
        k += 1
        nxt = set(goal)
        for (r, c) in cells:
            if (r, c) in safe and any((r + dr, c + dc) in z
                                      for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                nxt.add((r, c))
        for cell in nxt - z:
            entered[cell] = k
        if nxt == z:
            return entered, k - 1
        z = nxt


def route(entered, start):
    """Descend the iteration index from the start to a goal cell."""
    if start not in entered:
        return []
    path = [start]
    while entered[path[-1]] > 1:
        r, c = path[-1]
        nb = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))]
        nb = [n for n in nb if n in entered and entered[n] == entered[path[-1]] - 1]
        path.append(sorted(nb)[0])
    return path


def draw_floor(d, cells, x0, y0, s, region=(), colour=None, unknown=(), lifted=(), q=True):
    colour = colour or {}
    for (r, c), ch in cells.items():
        x, y = x0 + c * s, y0 + r * s
        if ch == '#':
            fill, line = WALL, WALL_LINE
        elif (r, c) in lifted and (r, c) in region:
            fill, line = LIFTED, LIFTED
        elif (r, c) in region:
            fill, line = colour.get((r, c), REGION), REGION_LINE
        elif (r, c) in unknown:
            fill, line = UNKNOWN, UNKNOWN
        else:
            fill, line = FLOOR, FLOOR_LINE
        d.rectangle([x, y, x + s - 1, y + s - 1], fill=fill, outline=line)
        if (r, c) in unknown and (r, c) not in region and q and s >= 14:
            d.text((x + s / 2, y + s / 2), '?', fill=(70, 66, 60), font=font(int(s * 0.5)),
                   anchor='mm')


def draw_marks(d, cells, x0, y0, s, path=()):
    if len(path) > 1:
        pts = [(x0 + c * s + s / 2, y0 + r * s + s / 2) for r, c in path]
        d.line(pts, fill=ROBOT, width=max(2, s // 7))
    for (r, c), ch in cells.items():
        x, y = x0 + c * s, y0 + r * s
        if ch in 'GD':
            d.rectangle([x + 1, y + 1, x + s - 2, y + s - 2], fill=GOAL)
        if ch == 'R':
            rad = s * 0.36
            d.ellipse([x + s / 2 - rad, y + s / 2 - rad, x + s / 2 + rad, y + s / 2 + rad],
                      fill=ROBOT, outline=WHITE, width=max(1, s // 11))


def find(cells, ch):
    return [rc for rc, v in cells.items() if v == ch]


def free_cells(cells, extra=()):
    return {rc for rc, v in cells.items() if v in '.GDR'} | set(extra)


# ── 1. the approximants ────────────────────────────────────────────────────

def approximants(out):
    cells = parse(SENSING)
    unknown = set(find(cells, '?'))
    safe = free_cells(cells, unknown)          # after the sensing action
    goal = set(find(cells, 'G'))
    start = find(cells, 'R')[0]
    entered, n = least_fixed_point(cells, goal, safe)
    picks = [1, 4, 8, 12, n]
    s = 13
    pw, ph = 18 * s, 13 * s
    gap, top, bottom = 18, 8, 40
    W = len(picks) * pw + (len(picks) + 1) * gap
    img = Image.new('RGB', (W, top + ph + bottom), WHITE)
    d = ImageDraw.Draw(img)
    for i, k in enumerate(picks):
        x0 = gap + i * (pw + gap)
        z = {c for c, j in entered.items() if j <= k}
        newest = {c for c, j in entered.items() if j == k}
        colour = {c: (0, 82, 128) for c in newest}
        last = k == n
        draw_floor(d, cells, x0, top, s, region=z, colour=colour)
        draw_marks(d, cells, x0, top, s, path=route(entered, start) if last else ())
        label = 'Z* = W' if last else 'Z' + str(k).translate(SUB)
        d.text((x0 + pw / 2, top + ph + 8), label, fill=INK, font=font(15, True), anchor='mt')
        d.text((x0 + pw / 2, top + ph + 26), plural(len(z), 'cell') + ('' if not last else
                f', fixed at k = {n}'), fill=GRAY, font=font(11), anchor='mt')
    img.save(os.path.join(out, 'mu-approximants.png'))
    return n


# ── 2. the region by iteration, and the route ───────────────────────────────

def iteration(out):
    cells = parse(SENSING)
    unknown = set(find(cells, '?'))
    safe = free_cells(cells, unknown)
    goal = set(find(cells, 'G'))
    start = find(cells, 'R')[0]
    entered, n = least_fixed_point(cells, goal, safe)
    s = 22
    img = Image.new('RGB', (18 * s, 13 * s), WHITE)
    d = ImageDraw.Draw(img)
    # Early iterations dark, late ones light: the order a route descends.
    colour = {}
    for c, k in entered.items():
        t = (k - 1) / max(1, n - 1)
        colour[c] = tuple(int(a + (b - a) * t) for a, b in zip((0, 82, 128), (176, 214, 236)))
    draw_floor(d, cells, 0, 0, s, region=set(entered), colour=colour)
    for c, k in entered.items():
        r, col = c
        d.text((col * s + s / 2, r * s + s / 2), str(k), fill=WHITE if k < n * 0.55 else INK,
               font=font(8), anchor='mm')
    draw_marks(d, cells, 0, 0, s, path=route(entered, start))
    img.save(os.path.join(out, 'mu-iteration.png'))
    return n, entered[start]


# ── 3. knowledge in the safe set ────────────────────────────────────────────

def epistemic(out):
    cells = parse(BAYS)
    bays = {t: {rc for rc, v in cells.items() if v == t} for t in '123'}
    goal = set(find(cells, 'D'))
    start = find(cells, 'R')[0]
    s = 22
    pw, ph = 18 * s, 13 * s
    gap, bottom = 24, 54
    img = Image.new('RGB', (2 * pw + 3 * gap, ph + bottom), WHITE)
    d = ImageDraw.Draw(img)
    unknown = bays['1'] | bays['2'] | bays['3']
    out_numbers = []
    for i, known in enumerate([(), ('2',)]):
        lifted = set().union(*(bays[t] for t in known)) if known else set()
        safe = free_cells(cells) | lifted
        entered, n = least_fixed_point(cells, goal, safe)
        x0 = gap + i * (pw + gap)
        draw_floor(d, cells, x0, 0, s, region=set(entered), unknown=unknown, lifted=lifted)
        for t, rcs in bays.items():
            r, c = min(rcs)
            d.text((x0 + (c + 1) * s, (r + 1) * s), f't{t}', fill=WHITE, font=font(11, True),
                   anchor='mm')
        draw_marks(d, cells, x0, 0, s, path=route(entered, start))
        inside = start in entered
        title = ('Safe = free' if not known else 'Safe = free ∨ (t2 ∧ K open(t2))')
        d.text((x0 + pw / 2, ph + 10), title, fill=INK, font=font(13, True), anchor='mt')
        d.text((x0 + pw / 2, ph + 31), f'{len(entered)} cells after {n} iterations; robot '
               + ('in W' if inside else 'not in W'), fill=RED if not inside else GRAY,
               font=font(11), anchor='mt')
        out_numbers.append((len(entered), n, inside))
    img.save(os.path.join(out, 'mu-epistemic.png'))
    return out_numbers


# ── 4. common knowledge, from above ─────────────────────────────────────────

CHAIN = {'u0': (0, 0), 'u9': (1, 0), 'u8': (2, 0), 'u7': (3, 0), 'u5': (4, 0), 'u3': (5, 0),
         'u6': (1.5, 1), 'u4': (2.5, 1), 'u2': (3.5, 1), 'u1': (4.5, 1)}


def common_knowledge(out, models):
    m = json.load(open(models))['radio'][5]
    worlds = [w['id'] for w in m['worlds']]
    stand = m['stand']
    phi = {w['id']: stand in w['job'] for w in m['worlds']}
    rel = {a: {w: {w} for w in worlds} for a in m['relations']}   # reflexive
    for a, pairs in m['relations'].items():
        for u, v in pairs:
            rel[a][u].add(v)
    actual = [w['id'] for w in m['worlds'] if w['designated']][0]
    xs = [set(worlds)]
    for _ in range(6):
        prev = xs[-1]
        xs.append({w for w in worlds
                   if all(phi[v] and v in prev for a in rel for v in rel[a][w])})
    sx, sy, rad = 46, 44, 15
    pw, ph = int(5 * sx + 2 * rad + 24), int(sy + 2 * rad + 18)
    cols, rows = 3, 2
    gap, lab = 16, 40
    img = Image.new('RGB', (cols * pw + (cols + 1) * gap, rows * (ph + lab) + gap), WHITE)
    d = ImageDraw.Draw(img)
    for k in range(6):
        x0 = gap + (k % cols) * (pw + gap) + rad + 12
        y0 = gap // 2 + (k // cols) * (ph + lab) + rad + 6
        pos = {w: (x0 + CHAIN[w][0] * sx, y0 + CHAIN[w][1] * sy) for w in worlds}
        for a, col in (('south', SOUTH), ('north', NORTH)):
            for u, v in m['relations'][a]:
                if u < v:
                    d.line([pos[u], pos[v]], fill=col, width=3)
        for w in worlds:
            x, y = pos[w]
            if w in xs[k]:
                fill, txt = REGION, WHITE
            elif not phi[w]:
                fill, txt = (228, 228, 228), RED
            else:
                fill, txt = WHITE, INK
            ring = RED if w == actual else (138, 138, 138)
            d.ellipse([x - rad, y - rad, x + rad, y + rad], fill=fill, outline=ring,
                      width=3 if w == actual else 1)
            if not phi[w]:
                d.text((x, y), 's2', fill=txt, font=font(9, True), anchor='mm')
        inside = actual in xs[k]
        d.text((x0 - rad, y0 + ph - rad - 2), 'X' + str(k).translate(SUB), fill=INK,
               font=font(14, True), anchor='lm')
        d.text((x0 - rad + 34, y0 + ph - rad - 2),
               plural(len(xs[k]), 'world') + '; actual ' + ('in' if inside else 'out'),
               fill=GRAY if inside else RED, font=font(11), anchor='lm')
    img.save(os.path.join(out, 'mu-common-knowledge.png'))
    return [len(x) for x in xs], max(k for k in range(len(xs)) if actual in xs[k])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--models', required=True,
                    help="coordinated_attack_demo/tools/export_models.py's output")
    ap.add_argument('--out', default='docs/img')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print('approximants: fixed point at k =', approximants(args.out))
    print('iteration: (k*, k at the robot) =', iteration(args.out))
    print('epistemic: (|W|, iterations, robot in W) =', epistemic(args.out))
    print('common knowledge: |X_k| =', *common_knowledge(args.out, args.models))


if __name__ == '__main__':
    main()
