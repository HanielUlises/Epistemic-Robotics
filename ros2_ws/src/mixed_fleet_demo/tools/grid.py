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
The ward as an occupancy grid at a TurtleBot3's laser height, for a given state
of the doors; rays cast over it; and the rule a robot decides a door by, shared
with scripts/door_node.py.

A door is decided from a bend of its lobby, looking along the jog: returns on
the shut leaf's line say shut; returns from the jog's far end, beyond the
leaf's line, and none on it, say open; anything else decides nothing yet.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import furniture as F  # noqa: E402
import layout as L  # noqa: E402

RES = 0.05
X0, Y0 = L.BUILDING[0] - 0.5, L.BUILDING[1] - 0.5
NX = int(round((L.BUILDING[2] - L.BUILDING[0] + 1.0) / RES))
NY = int(round((L.BUILDING[3] - L.BUILDING[1] + 1.0) / RES))
LASER_MAX = 8.0
MIN_HITS = 5
BAND_Y = (L.JOG_Y[0] + 0.15, L.JOG_Y[1] - 0.15)     # the jog's height, clear of the open leaf


def _fill(g, x0, y0, x1, y1):
    i0, i1 = int(math.floor((x0 - X0) / RES)), int(math.ceil((x1 - X0) / RES))
    j0, j1 = int(math.floor((y0 - Y0) / RES)), int(math.ceil((y1 - Y0) / RES))
    g[max(j0, 0):max(j1, 0), max(i0, 0):max(i1, 0)] = True


def _segments(g, segs):
    for x0, y0, x1, y1 in segs:
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / (RES / 2)) + 1)
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        i = ((xs - X0) / RES).astype(int)
        j = ((ys - Y0) / RES).astype(int)
        ok = (i >= 0) & (i < NX) & (j >= 0) & (j < NY)
        g[j[ok], i[ok]] = True


_STATIC = None


def static():
    """Walls, the band's blocks and the furniture: everything but the doors."""
    global _STATIC
    if _STATIC is None:
        g = np.zeros((NY, NX), bool)
        for cx, cy, sx, sy in L.walls():
            _fill(g, cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
        for box in L.linen():
            _fill(g, *box)
        for model, x, y, yaw in L.FURNITURE:
            _segments(g, F.slice_segments(F.placed(model, x, y, yaw)))
        _STATIC = g
    return _STATIC.copy()


def leaf_corners(d, s):
    """The leaf's rectangle part way through its swing, s = 0 shut, 1 open."""
    cx, cy, yaw = L.leaf_swing(d, s)
    hl, ht = L.LEAF[0] / 2, L.LEAF[1] / 2
    # the leaf runs along its own y when yaw = 0
    c, sn = math.cos(yaw), math.sin(yaw)
    pts = []
    for ex, ey in ((-ht, -hl), (ht, -hl), (ht, hl), (-ht, hl)):
        pts.append((cx + ex * c - ey * sn, cy + ex * sn + ey * c))
    return pts


def world_grid(state):
    """The grid with each door open or shut, state a dict door -> open."""
    g = static()
    for d, o in state.items():
        pts = leaf_corners(d, 1.0 if o else 0.0)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        _fill(g, min(xs), min(ys), max(xs), max(ys))
    return g


def occupied(g, x, y):
    i, j = int((x - X0) / RES), int((y - Y0) / RES)
    return not (0 <= i < NX and 0 <= j < NY) or bool(g[j, i])


def cast(g, x, y, angle, rmax=LASER_MAX):
    c, s = math.cos(angle), math.sin(angle)
    r = 0.12
    while r <= rmax:
        if occupied(g, x + c * r, y + s * r):
            return r
        r += RES / 2
    return None


def scan(g, x, y, n=360):
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        r = cast(g, x, y, a)
        if r is not None:
            pts.append((x + r * math.cos(a), y + r * math.sin(a)))
    return pts


def decide(d, side, points):
    """'open', 'shut' or '' from a scan's returns in the world frame, from the
    bend of door d's lobby on side 'S' (south leg) or 'N' (north leg)."""
    x = L.LEAF_X[d]
    lo, hi = BAND_Y
    on_leaf = sum(1 for px, py in points if abs(px - x) <= 0.12 and lo <= py <= hi)
    if side == 'S':
        beyond = sum(1 for px, py in points if px >= x + 0.4 and lo <= py <= hi and px <= L.NORTH_X[d] + 1.0)
    else:
        beyond = sum(1 for px, py in points if px <= x - 0.4 and lo <= py <= hi and px >= L.DOOR_X[d] - 1.0)
    if on_leaf >= MIN_HITS:
        return 'shut'
    if beyond >= MIN_HITS and on_leaf == 0:
        return 'open'
    return ''


def bend_point(d, side):
    return (L.DOOR_X[d], L.BEND_Y) if side == 'S' else (L.NORTH_X[d], L.BEND_Y)


def clearance(g, path, step=0.05):
    """The least distance from a polyline to anything occupied, up to 1 m."""
    occ = np.argwhere(g)
    ox = X0 + (occ[:, 1] + 0.5) * RES
    oy = Y0 + (occ[:, 0] + 0.5) * RES
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        n = max(2, int(math.dist((ax, ay), (bx, by)) / step))
        for t in np.linspace(0, 1, n):
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            m = (np.abs(ox - px) < 1.0) & (np.abs(oy - py) < 1.0)
            if m.any():
                best = min(best, float(np.min(np.hypot(ox[m] - px, oy[m] - py))))
    return best
