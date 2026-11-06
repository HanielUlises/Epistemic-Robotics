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
The residence as an occupancy grid at a TurtleBot3's laser height, for a
given state of the doors; rays cast over it; and the rule a robot decides a
door by, shared with scripts/door_node.py.

A door is decided from the returns of a laser within DECIDE_RANGE of the door:
returns on the shut leaf's rectangle say shut, returns beyond the door line in
the passage's span and none on the leaf say open, and anything else decides
nothing yet.
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
    """Walls, linen rooms and furniture: everything but the doors."""
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


def leaf_box(d, open_):
    cx, cy, _ = L.leaf(d, open_)
    if open_:
        return (cx - L.LEAF[0] / 2, cy - L.LEAF[1] / 2, cx + L.LEAF[0] / 2, cy + L.LEAF[1] / 2)
    return (cx - L.LEAF[0] / 2, cy - L.LEAF[1] / 2, cx + L.LEAF[0] / 2, cy + L.LEAF[1] / 2)


def world_grid(state):
    """The grid with each door open or shut, state a dict door -> open."""
    g = static()
    for d, o in state.items():
        _fill(g, *leaf_box(d, o))
        if not o:
            # the jambs' gaps beside a shut 1 m leaf in a 1.2 m passage are closed by its frame
            x = L.DOOR_X[d]
            _fill(g, x - L.GAP / 2, L.DOOR_Y - 0.08, x + L.GAP / 2, L.DOOR_Y + 0.08)
    return g


def occupied(g, x, y):
    i, j = int((x - X0) / RES), int((y - Y0) / RES)
    return not (0 <= i < NX and 0 <= j < NY) or bool(g[j, i])


def cast(g, x, y, angle, rmax=LASER_MAX):
    """The range a ray from (x, y) along angle returns, or None past rmax."""
    c, s = math.cos(angle), math.sin(angle)
    r = 0.12
    while r <= rmax:
        if occupied(g, x + c * r, y + s * r):
            return r
        r += RES / 2
    return None


def scan(g, x, y, n=360):
    """A full 360-ray scan from (x, y): world points of the returns."""
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        r = cast(g, x, y, a)
        if r is not None:
            pts.append((x + r * math.cos(a), y + r * math.sin(a)))
    return pts


def decide(d, points):
    """'open', 'shut' or '' from a scan's returns in the world frame."""
    x = L.DOOR_X[d]
    inner = L.GAP / 2 - 0.12          # the passage's interior, clear of its side walls
    on_leaf = sum(1 for px, py in points if abs(px - x) <= inner and abs(py - L.DOOR_Y) <= 0.15)
    # From south of the band, a ray reaches north of the door line only through
    # an open passage, and only the decided one is in view (check_views.py).
    beyond = sum(1 for px, py in points if py >= L.DOOR_Y + 0.4)
    if on_leaf >= MIN_HITS:
        return 'shut'
    if beyond >= MIN_HITS and on_leaf == 0:
        return 'open'
    return ''


def clearance(g, path, step=0.05):
    """The least distance from a polyline to anything occupied, up to 1 m."""
    occ = np.argwhere(g)
    if not len(occ):
        return math.inf
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
