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
Casts the rays of the building and checks what the domain assumes, in each of
the seven worlds:

- every segment of every route keeps CLEAR m from every wall and piece of
  clutter, and from rubble on any other way than its own;
- a scout at its vantage decides its own doorway, and decides it right, and
  does not decide the other;
- from the start of every robot, and from either end of the middle passage,
  no doorway is decided, and the rubble in the passage is out of sight.

Exits non-zero on the first failure.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid as G  # noqa: E402
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

CLEAR = 0.25
RMAX = 8.0
ANGLES = np.radians(np.arange(0.0, 360.0, 1.0))


def look(g, places, scans=3):
    lo = np.zeros((G.H, G.W))
    for x, y in places:
        rng = G.cast(g, x - 0.032, y, ANGLES, RMAX)
        for _ in range(scans):
            G.integrate(lo, x - 0.032, y, 0.0, ANGLES, rng, RMAX)
    return G.ternary(lo)


def segment_clear(g, a, b):
    n = max(2, int(math.dist(a, b) / 0.02))
    r = int(math.ceil(CLEAR / G.RES))
    for k in range(n + 1):
        x = a[0] + (b[0] - a[0]) * k / n
        y = a[1] + (b[1] - a[1]) * k / n
        i, j = G.cell(x, y)
        if g[max(j - r, 0):j + r + 1, max(i - r, 0):i + r + 1].any():
            return (round(x, 2), round(y, 2))
    return None


def sees(g, rubble_cells, place):
    """Whether a laser at `place` has any ray end on the given cells."""
    x, y = place
    rng = G.cast(g, x - 0.032, y, ANGLES, RMAX)
    for k, r in enumerate(rng):
        if np.isfinite(r):
            i, j = G.cell(x - 0.032 + (r + G.RES / 2) * math.cos(ANGLES[k]), y + (r + G.RES / 2) * math.sin(ANGLES[k]))
            if rubble_cells[j, i]:
                return True
    return False


def main():
    nodes = L.roadmap()
    fail = 0
    for world in W.WORLDS:
        g = G.truth(world)
        tag = ''.join(d.upper() if c else d for d, c in world.items())
        for name, route in L.ROUTE.items():
            way = name[2:] if name.startswith('m_') else None
            if way and not world[way]:
                continue
            for u, v in zip(route, route[1:]):
                bad = segment_clear(g, nodes[u], nodes[v])
                if bad:
                    print(f'FAIL {tag} {name} {u}-{v} within {CLEAR} m of something at {bad}')
                    fail += 1
        for s_, d, other in (('s1', 'e', 'w'), ('s2', 'w', 'e')):
            t = look(g, [L.VANTAGE[s_]])
            want = 'clear' if world[d] else 'blocked'
            got, got_other = G.decide(t, d), G.decide(t, other)
            ok = got == want and got_other == ''
            print(f'{"ok  " if ok else "FAIL"} {tag} {s_} at its vantage: {d} {got!r}, {other} {got_other or "undecided"}')
            fail += not ok
        if not world['m']:
            cells = np.zeros_like(g)
            cx, cy, sx, sy = L.RUBBLE['m']
            cells[G.box_cells((cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2))] = True
            for p in ('m0', 'hall', 'p0', f'p{len(L.PASSAGE) - 1}', 'above_m', 'v_e', 'v_w', 's1_0', 's2_0'):
                if sees(g, cells, nodes[p]):
                    print(f'FAIL {tag} the rubble in the middle passage is in sight from {p}')
                    fail += 1
        for p in ('m0', 's1_0', 's2_0', 'hall', 'p0', f'p{len(L.PASSAGE) - 1}'):
            t = look(g, [nodes[p]])
            got = [d for d in ('w', 'e') if G.decide(t, d)]
            if got:
                print(f'FAIL {tag} from {p} the {got} doorway is decided')
                fail += 1
    print('all checks passed' if not fail else f'{fail} check(s) failed')
    sys.exit(1 if fail else 0)


if __name__ == '__main__':
    main()
