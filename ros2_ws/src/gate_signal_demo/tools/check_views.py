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
Checks, without a simulator, what r2 can see of r1 from its station.

    check_views.py

A laser of 360 rays and 8 m at r2's station, the walls as boxes and r1 as a
disc of the Burger's radius. r1 must be hit by several rays when it stands in
either gate, and by none at any point of either back route, inside either bay,
at its start or anywhere in the south corridor. What r2 sees of a passage it
may see: r1 enters a passage only on its way to that passage's gate. From r1's start
the order board's lit slot must be hit and the other free.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

R1 = 0.105


def ray_box(ox, oy, dx, dy, box):
    cx, cy, sx, sy = box
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmax >= max(tmin, 0) and tmin > 0 else math.inf


def ray_disc(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def hits(origin, target, walls, n=360, rng=8.0):
    ox, oy = origin
    count = 0
    for k in range(n):
        a = math.radians(k * 360.0 / n)
        dx, dy = math.cos(a), math.sin(a)
        wall = min(ray_box(ox, oy, dx, dy, w) for w in walls)
        t = ray_disc(ox, oy, dx, dy, *target, R1)
        if t < wall and t < rng:
            count += 1
    return count


def along(path, step=0.15):
    nodes, _ = L.roadmap()
    for u, v in zip(path, path[1:]):
        (x0, y0), (x1, y1) = nodes[u], nodes[v]
        n = max(1, int(math.dist((x0, y0), (x1, y1)) / step))
        for t in range(n + 1):
            yield (x0 + (x1 - x0) * t / n, y0 + (y1 - y0) * t / n)


def main():
    walls = L.walls()
    station = L.STATION[:2]
    bad = 0
    for b in ('x', 'y'):
        h = hits(station, L.gate(b), walls)
        print(f'r1 in gate {b}: hit by {h} rays from the station')
        bad += h < 3
        back = ['start', f's_{b}', f'sd_{b}', f'bay_{b}']
        seen = [p for p in along(back) if hits(station, p, walls)]
        print(f'back route to bay {b}: {len(seen)} points seen')
        bad += bool(seen)
        x0, y0, x1, y1 = L.bay_region(b)
        inside = [(x0 + (x1 - x0) * i / 8, y0 + (y1 - y0) * j / 8) for i in range(1, 8) for j in range(1, 8)]
        seen = [p for p in inside if hits(station, p, walls)]
        print(f'inside bay {b}: {len(seen)} of {len(inside)} points seen')
        bad += bool(seen)
    corridor = [(-5.8 + 0.2 * i, -4.8 + 0.2 * j) for i in range(59) for j in range(15)]
    seen = [p for p in corridor if hits(station, p, walls)]
    print(f'south corridor: {len(seen)} of {len(corridor)} points seen')
    bad += bool(seen)
    h = hits(station, L.START[:2], walls)
    print(f'r1 at its start: hit by {h} rays')
    bad += h > 0
    print('all views as the domain says' if not bad else f'{bad} failures')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
