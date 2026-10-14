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
Checks, without a simulator, that the hall's geometry gives each observation
the meaning the domain gives it.

    check_views.py

A laser of 360 rays and 8 m is cast over a rasterised floor from a handful of
headings at each pose, and every 5 cm cell a ray crosses is marked free and
every cell it ends in occupied, as an occupancy mapper would. A region is then
classified by the perception's rule: blocked if any cell is occupied, clear if
every cell is observed and free, unknown otherwise. For every world it checks:

  at lookout k     the zone is decided exactly when the view from k is clear,
                   and the window strip reads blocked exactly when it is not;
  at the board     the lit slot reads blocked and the other three clear;
  along the routes the zone is never decided, so no run learns it by passing.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

RES = 0.025
X0, Y0 = L.HALL[0] - 0.3, L.HALL[1] - 0.3
NX = int((L.HALL[2] - L.HALL[0] + 0.6) / RES)
NY = int((L.HALL[3] - L.HALL[1] + 0.6) / RES)
MAP = 0.05


def raster(clear, blocked):
    g = np.zeros((NX, NY), bool)

    def fill(x0, y0, x1, y1):
        i0, i1 = int((x0 - X0) / RES), int(math.ceil((x1 - X0) / RES))
        j0, j1 = int((y0 - Y0) / RES), int(math.ceil((y1 - Y0) / RES))
        g[max(i0, 0):i1, max(j0, 0):j1] = True
    for cx, cy, sx, sy in L.walls():
        fill(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
    for k in range(1, 5):
        if k != clear:
            fill(*L.screen_box(k))
    if blocked:
        fill(-L.CRATE, -L.CRATE, L.CRATE, L.CRATE)
    ys = L.SLOT_Y
    fill(L.BOARD_PANEL_X, min(ys) - 0.25, L.BOARD_PANEL_X + 0.06, max(ys) + 0.25)
    y = ys[clear - 1]
    fill(L.POST_X - 0.07, y - 0.07, L.POST_X + 0.07, y + 0.07)
    return g


def scan(g, x, y, yaws, free, occ):
    for yaw in yaws:
        for b in range(360):
            a = yaw + math.radians(b)
            dx, dy = math.cos(a), math.sin(a)
            r = 0.12
            while r < 8.0:
                px, py = x + dx * r, y + dy * r
                i, j = int((px - X0) / RES), int((py - Y0) / RES)
                if not (0 <= i < NX and 0 <= j < NY):
                    break
                mi, mj = int((px - X0) / MAP), int((py - Y0) / MAP)
                if g[i, j]:
                    occ.add((mi, mj))
                    break
                free.add((mi, mj))
                r += RES / 2


def classify(box, free, occ):
    x0, y0, x1, y1 = box
    cells = [(i, j) for i in range(int((x0 - X0) / MAP), int(math.ceil((x1 - X0) / MAP)))
             for j in range(int((y0 - Y0) / MAP), int(math.ceil((y1 - Y0) / MAP)))
             if x0 <= X0 + (i + 0.5) * MAP <= x1 and y0 <= Y0 + (j + 0.5) * MAP <= y1]
    if any(c in occ for c in cells):
        return 'blocked', len(cells)
    if all(c in free and c not in occ for c in cells):
        return 'clear', len(cells)
    seen = sum(1 for c in cells if c in free)
    return f'unknown ({seen}/{len(cells)})', len(cells)


def at(g, x, y, face):
    yaw = math.atan2(face[1] - y, face[0] - x)
    free, occ = set(), set()
    scan(g, x, y, [yaw + math.radians(d) for d in (0, 0.3, 0.6, -0.3, -0.6)], free, occ)
    return free, occ


def route_points(a, b, step=0.25):
    nodes, _ = L.roadmap()
    path, _ = L.shortest(a, b)
    for u, v in zip(path, path[1:]):
        (x0, y0), (x1, y1) = nodes[u], nodes[v]
        n = max(1, int(math.dist((x0, y0), (x1, y1)) / step))
        for t in range(n + 1):
            yield x0 + (x1 - x0) * t / n, y0 + (y1 - y0) * t / n, math.atan2(y1 - y0, x1 - x0)


def main():
    bad = 0
    for clear in range(1, 5):
        for blocked in (False, True):
            g = raster(clear, blocked)
            want_zone = 'blocked' if blocked else 'clear'
            for k in range(1, 5):
                x, y = L.lookout(k)
                free, occ = at(g, x, y, (0.0, 0.0))
                zone, n = classify(L.zone_region(), free, occ)
                win, _ = classify(L.window_region(k), free, occ)
                ok_zone = (zone == want_zone) if k == clear else zone.startswith('unknown')
                ok_win = (win == 'clear') if k == clear else (win == 'blocked')
                if not (ok_zone and ok_win):
                    bad += 1
                    print(f'world clear={clear} blocked={blocked} lookout {k}: '
                          f'zone {zone} of {n}, window {win}')
            free, occ = at(g, *L.BOARD, (L.BOARD_PANEL_X, sum(L.SLOT_Y) / 4))
            for k in range(1, 5):
                s, _ = classify(L.slot_region(k), free, occ)
                if s != ('blocked' if k == clear else 'clear'):
                    bad += 1
                    print(f'world clear={clear}: board slot {k} reads {s}')
            for a, b in (('start', 'l1'), ('start', 'l2'), ('l3', 'l2'), ('l2', 'l1'),
                         ('l1', 'l4'), ('l4', 'l3'), ('start', 'board')):
                # The route's own ends are where it is meant to look; what must
                # not happen is a view of the zone while passing between them.
                # A route between places that are not adjacent runs through
                # another lookout's spot, where the zone is in view if that
                # lookout is the open one. No run drives such a route in such a
                # world: the policy goes straight to the open lookout, and a
                # replanner never passes an unvisited lookout nearer than its
                # target. So those are not checked.
                nodes, _ = L.roadmap()
                if f'l{clear}' in L.shortest(a, b)[0][1:-1]:
                    continue
                free, occ = set(), set()
                for x, y, yaw in route_points(a, b):
                    if min(math.dist((x, y), nodes[a]), math.dist((x, y), nodes[b])) < 0.8:
                        continue
                    scan(g, x, y, [yaw], free, occ)
                zone, _ = classify(L.zone_region(), free, occ)
                if not zone.startswith('unknown'):
                    bad += 1
                    print(f'world clear={clear} blocked={blocked}: route {a}->{b} decides the zone: {zone}')
    print('all views as the domain says' if not bad else f'{bad} failures')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
