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
What the domain assumes of the floor, checked by casting rays over it.

    check_views.py [--plot FILE]

In all sixteen states of the doors:

  1. from the bend of each lobby, on either side, the rule of tools/grid.py
     decides that lobby's door, and decides it right;
  2. from every point of the hall's and the corridor's driving lines, 0.25 m
     apart, a laser gets fewer returns off any door than the rule needs: a
     door is decided only from inside its lobby;
  3. every route of the roadmap keeps 0.18 m from everything a robot can hit,
     the doors as they stand when the route is driven (a crossing with its
     door open).

The plot is the floor plan the page draws.
"""

import argparse
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid as G  # noqa: E402
import layout as L  # noqa: E402

CLEAR = 0.18


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot')
    a = ap.parse_args()
    nodes, south, north, cross = L.roadmap()
    bad = 0
    most = 0
    states = [dict(zip(L.DOORS, v)) for v in itertools.product((True, False), repeat=4)]
    for state in states:
        g = G.world_grid(state)
        for d in L.DOORS:
            for side in ('S', 'N'):
                v = G.decide(d, side, G.scan(g, *G.bend_point(d, side)))
                want = 'open' if state[d] else 'shut'
                if v != want:
                    bad += 1
                    print(f'  {state}: from the {side} bend of {d}, decided {v!r}, is {want}')
        glimpse = 0
        for y in (L.HALL_Y, L.CORRIDOR_Y):
            for k in range(int((L.BUILDING[2] - L.BUILDING[0] - 1.0) / 0.25)):
                x = L.BUILDING[0] + 0.5 + 0.25 * k
                pts = G.scan(g, x, y)
                lo, hi = G.BAND_Y
                for d in L.DOORS:
                    seen = sum(1 for px, py in pts if abs(px - L.LEAF_X[d]) <= 0.12 and lo <= py <= hi)
                    glimpse = max(glimpse, seen)
                    if seen >= G.MIN_HITS:
                        bad += 1
                        print(f'  {state}: {seen} returns on door {d} from ({x:.2f}, {y}), outside its lobby')
        most = max(most, glimpse)
    print(f'views: {len(states)} states, every door decided from its bends; from the hall and the corridor at '
          f'most {most} returns on a door, fewer than the {G.MIN_HITS} the rule needs'
          if not bad else f'views: {bad} failures')
    # clearances, with every door open, and the crossings with their own door open
    g = G.world_grid({d: True for d in L.DOORS})
    worst = math.inf
    for u, v in south + north + cross:
        c = G.clearance(g, [nodes[u], nodes[v]])
        worst = min(worst, c)
        if c < CLEAR:
            bad += 1
            print(f'  route {u}-{v}: clearance {c:.2f} m')
    for lp in L.LOOPS:
        p = L.porter_path(lp)
        c = G.clearance(g, [nodes[n] for n in p])
        if c < 0.12:
            print(f'  the porter on {lp}: clearance {c:.2f} m')
    print(f'routes: least clearance {worst:.2f} m')
    if a.plot:
        plot(a.plot)
    sys.exit(1 if bad else 0)


def plot(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    g = G.world_grid({d: True for d in L.DOORS})
    fig, ax = plt.subplots(figsize=(11, 6.6))
    ax.imshow(g, origin='lower', cmap='Greys', extent=(G.X0, G.X0 + G.NX * G.RES, G.Y0, G.Y0 + G.NY * G.RES),
              vmin=0, vmax=1.6)
    nodes, south, north, cross = L.roadmap()
    for u, v in south + north + cross:
        ax.plot(*zip(nodes[u], nodes[v]), color='#7aa6c2', lw=0.8)
    for d in L.DOORS:
        pts = G.leaf_corners(d, 0.0)
        ax.fill(*zip(*pts), color='#c0392b', alpha=0.6)
        ax.text(L.DOOR_X[d], -1.6, d.upper(), ha='center', fontsize=12)
    ax.set_aspect('equal')
    ax.set_xlim(L.BUILDING[0] - 0.4, L.BUILDING[2] + 0.4)
    ax.set_ylim(L.BUILDING[1] - 0.4, L.BUILDING[3] + 0.4)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


if __name__ == '__main__':
    main()
