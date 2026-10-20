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
Casts the rays of the hall and checks what the layout claims, with debris and
without:

- every segment of the roadmap keeps CLEAR m from every wall and from the debris;
- a robot at its spot decides the corridor, and decides it right;
- a robot at the dock, the station, or any place of the roadmap short of the
  mouth's lane, does not decide it;
- the two spots do not hide the corridor from each other.

Exits non-zero on the first failure.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid as G  # noqa: E402
import layout as L  # noqa: E402

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


def main():
    nodes, edges = L.roadmap()
    fail = 0
    for blocked in (False, True):
        g = G.truth(blocked)
        want = 'blocked' if blocked else 'clear'
        for u, v in edges:
            bad = segment_clear(g, nodes[u], nodes[v])
            if bad:
                print(f'FAIL {u}-{v} within {CLEAR} m of a wall at {bad} (debris {blocked})')
                fail += 1
        for r in ('r1', 'r2'):
            d = G.decide(look(g, [L.SPOT[r]]))
            print(f'{"ok  " if d == want else "FAIL"} {r} at its spot decides {d!r}, debris {blocked}')
            fail += d != want
        both = [L.SPOT['r1'], L.SPOT['r2']]
        g2 = g.copy()
        for x, y in both:     # each robot a disc of 0.1 m in the other's view
            i, j = G.cell(x, y)
            g2[j - 2:j + 3, i - 2:i + 3] = True
        for r, other in (('r1', 'r2'), ('r2', 'r1')):
            g3 = g.copy()
            i, j = G.cell(*L.SPOT[other])
            g3[j - 2:j + 3, i - 2:i + 3] = True
            d = G.decide(look(g3, [L.SPOT[r]]))
            print(f'{"ok  " if d == want else "FAIL"} {r} at its spot with {other} at its own decides {d!r}')
            fail += d != want
        for p in ('dock', 'station', 'w_s', 'e_s', 'w_m'):
            d = G.decide(look(g, [nodes[p]]))
            print(f'{"ok  " if d == "" else "FAIL"} from {p} the corridor is {d or "undecided"}')
            fail += d != ''
        for r in ('r1', 'r2'):
            route = [nodes[p] for p in L.ROUTE[r]]
            stops = []
            for a, b in zip(route, route[1:-1]):
                n = max(1, int(math.dist(a, b) / 0.5))
                stops += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]
            seen = [s for s in stops if G.decide(look(g, [s], scans=1))]
            print(f'     {r} decides it first at {seen[0] if seen else "its spot"} on its way (debris {blocked})')
    print('all checks passed' if not fail else f'{fail} check(s) failed')
    sys.exit(1 if fail else 0)


if __name__ == '__main__':
    main()
