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

On each of the porter's six rounds, for every scout and every door:

  1. from where the scout stands at the door's mouth, a sound laser decides
     the door, and decides it right, with every other robot of the fleet
     standing at its own place at that mouth;
  2. from there a short-sighted laser decides every door open;
  3. from the mark before the door, both decide it right.

And every route of each robot's roadmap keeps 0.18 m from everything a robot
can hit, the doors as they stand when the route is driven (a crossing with
its door open).

The plot is the floor plan the page draws.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid as G  # noqa: E402
import layout as L  # noqa: E402
import ranks as R  # noqa: E402

CLEAR = 0.18
ROBOTS = sorted({r.name for act in R.FLEETS for r in R.FLEETS[act]})
SCOUTS = sorted({r.name for act in R.FLEETS for r in R.FLEETS[act] if r.role == 'scout'})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot')
    a = ap.parse_args()
    bad, least, farthest = 0, 10 ** 6, 0.0
    for loop in L.LOOPS:
        state = {d: d not in loop for d in L.DOORS}
        g0 = G.world_grid(state)
        for d in L.DOORS:
            want = 'open' if state[d] else 'shut'
            for s in SCOUTS:
                others = [G.stand(o, d, 'far') for o in ROBOTS if o != s and
                          math.dist(G.stand(o, d, 'far'), G.stand(s, d, 'far')) > 0.3]
                g = G.with_robots(g0, others)
                x, y = G.stand(s, d, 'far')
                farthest = max(farthest, math.dist((x, y), (L.DOOR_X[d], L.LEAF_Y)))
                pts = G.scan(g, x, y)
                v = G.decide(d, pts)
                if not state[d]:
                    least = min(least, G.on_leaf(d, pts))
                if v != want:
                    bad += 1
                    print(f'  {loop}: {s} at the mouth of {d} decides {v!r}, is {want}')
                if G.decide(d, G.short_sighted(pts, x, y)) != 'open':
                    bad += 1
                    print(f'  {loop}: {s}, short-sighted, at the mouth of {d} does not read it open')
                x, y = G.stand(s, d, 'near')
                pts = G.scan(g0, x, y)
                for name, p in (('sound', pts), ('short-sighted', G.short_sighted(pts, x, y))):
                    if G.decide(d, p) != want:
                        bad += 1
                        print(f'  {loop}: {s}, {name}, at the mark of {d} decides {G.decide(d, p)!r}, is {want}')
    print(f'views: {len(L.LOOPS)} rounds, every door decided right from its mouth (from up to {farthest:.2f} m, '
          f'at least {least} returns on a shut leaf, the rule needs {G.MIN_HITS}) and from its mark; '
          f'a laser blind beyond {L.FAR} m reads every door open from a mouth'
          if not bad else f'views: {bad} failures')
    g = G.world_grid({d: True for d in L.DOORS})
    worst = math.inf
    for r in ROBOTS:
        nodes, south, north, cross = L.roadmap(r)
        for u, v in south + north + cross:
            c = G.clearance(g, [nodes[u], nodes[v]])
            worst = min(worst, c)
            if c < CLEAR:
                bad += 1
                print(f'  {r}, route {u}-{v}: clearance {c:.2f} m')
    print(f'routes: least clearance {worst:.2f} m')
    if a.plot:
        plot(a.plot)
    sys.exit(1 if bad else 0)


def plot(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    g = G.world_grid({d: True for d in L.DOORS})
    fig, ax = plt.subplots(figsize=(11, 7.4))
    ax.imshow(g, origin='lower', cmap='Greys', extent=(G.X0, G.X0 + G.NX * G.RES, G.Y0, G.Y0 + G.NY * G.RES),
              vmin=0, vmax=1.6)
    for r in ROBOTS:
        nodes, south, north, cross = L.roadmap(r)
        for u, v in south + north + cross:
            ax.plot(*zip(nodes[u], nodes[v]), color='#7aa6c2', lw=0.6)
    for d in L.DOORS:
        pts = G.leaf_corners(d, 0.0)
        ax.fill(*zip(*pts), color='#c0392b', alpha=0.6)
        ax.text(L.DOOR_X[d], 2.0, d.upper(), ha='center', fontsize=12)
    ax.set_aspect('equal')
    ax.set_xlim(L.BUILDING[0] - 0.4, L.BUILDING[2] + 0.4)
    ax.set_ylim(L.BUILDING[1] - 0.4, L.BUILDING[3] + 0.4)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


if __name__ == '__main__':
    main()
