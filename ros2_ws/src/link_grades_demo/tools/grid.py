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
The occupancy grid every robot builds, and how a grid decides the corridor.

One grid for the hall, 5 cm cells. A cell holds a log-odds of occupancy; a
scan lowers it along each ray and raises it at the ray's end. A cell is known
free below FREE, known occupied above OCC, and unknown between. The same
module rasterises the hall's walls for tools/check_views.py, so the check and
the robots read the hall the same way.

A grid decides the corridor when its cells in layout.REGION say so:
blocked     at least BLOCKED_CELLS of them known occupied;
clear       none known occupied, and at least CLEAR_FRACTION of its 10 cm blocks
            (2 by 2 cells) holding a known free cell. A laser of 360 rays a
            degree apart leaves rows of 5 cm cells unswept at three metres
            when its rays run almost along them; at 10 cm none is.
Otherwise it does not decide, which is what a robot that has not looked has.
"""

import math

import numpy as np

import layout as L

RES = 0.05
ORIGIN = (L.HALL[0] - 0.2, L.HALL[1] - 0.2)
W = int(round((L.HALL[2] - L.HALL[0] + 0.4) / RES))
H = int(round((L.HALL[3] - L.HALL[1] + 0.4) / RES))
HIT, MISS = 0.85, -0.4
CLAMP = 4.0
FREE, OCC = -0.8, 1.2
BLOCKED_CELLS = 6
CLEAR_FRACTION = 0.9
UNKNOWN, KNOWN_FREE, KNOWN_OCC = -1, 0, 100


def cell(x, y):
    return int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)


def box_cells(b):
    x0, y0, x1, y1 = b
    i0, j0 = cell(x0, y0)
    i1, j1 = cell(x1, y1)
    return slice(max(j0, 0), min(j1 + 1, H)), slice(max(i0, 0), min(i1 + 1, W))


REGION_CELLS = box_cells((L.REGION[0] + RES / 2, L.REGION[1] + RES / 2, L.REGION[2] - RES / 2, L.REGION[3] - RES / 2))


def truth(blocked):
    """The hall as cells: True where a wall or the debris stands."""
    g = np.zeros((H, W), dtype=bool)
    for cx, cy, sx, sy in L.walls():
        g[box_cells((cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2))] = True
    if blocked:
        g[box_cells(L.debris_box())] = True
    return g


def ternary(logodds):
    t = np.full(logodds.shape, UNKNOWN, dtype=np.int8)
    t[logodds < FREE] = KNOWN_FREE
    t[logodds > OCC] = KNOWN_OCC
    return t


def merge(own, peer):
    """What a robot's map holds with a peer's map beside it: its own cell where
    it has one, the peer's where it has none."""
    if peer is None:
        return own
    return np.where(own == UNKNOWN, peer, own).astype(np.int8)


def decide(t):
    """'blocked', 'clear' or '' for a ternary grid."""
    r = t[REGION_CELLS]
    occ = int((r == KNOWN_OCC).sum())
    if occ >= BLOCKED_CELLS:
        return 'blocked'
    h, w = r.shape
    blocks = (r[:h // 2 * 2, :w // 2 * 2] == KNOWN_FREE).reshape(h // 2, 2, w // 2, 2).any(axis=(1, 3))
    if occ == 0 and blocks.mean() >= CLEAR_FRACTION:
        return 'clear'
    return ''


def integrate(logodds, x, y, yaw, angles, ranges, rmax):
    """One scan from the laser at (x, y, yaw) into the log-odds grid, in place."""
    a = yaw + angles
    ok = np.isfinite(ranges)
    hit = ok & (ranges < rmax)
    r = np.where(ok, np.minimum(ranges, rmax), rmax)
    steps = np.arange(0.0, rmax, RES / 2)
    for k in range(len(a)):
        n = int(r[k] / (RES / 2))
        if n <= 0:
            continue
        s = steps[:n]
        i = ((x + s * math.cos(a[k]) - ORIGIN[0]) / RES).astype(int)
        j = ((y + s * math.sin(a[k]) - ORIGIN[1]) / RES).astype(int)
        keep = (i >= 0) & (i < W) & (j >= 0) & (j < H)
        idx = np.unique(j[keep] * W + i[keep])
        if hit[k]:
            hx, hy = cell(x + r[k] * math.cos(a[k]), y + r[k] * math.sin(a[k]))
            if 0 <= hx < W and 0 <= hy < H:
                idx = idx[idx != hy * W + hx]
                logodds.flat[hy * W + hx] = min(CLAMP, logodds.flat[hy * W + hx] + HIT)
        logodds.flat[idx] = np.maximum(-CLAMP, logodds.flat[idx] + MISS)


def cast(g, x, y, angles, rmax):
    """Ranges of rays from (x, y) over a truth grid, for the checks."""
    out = np.full(len(angles), np.inf)
    for k, a in enumerate(angles):
        for s in np.arange(0.0, rmax, RES / 2):
            i, j = cell(x + s * math.cos(a), y + s * math.sin(a))
            if not (0 <= i < W and 0 <= j < H):
                break
            if g[j, i]:
                out[k] = s
                break
    return out


def known_area(t, mask=None):
    m = t != UNKNOWN if mask is None else (t != UNKNOWN) & mask
    return float(m.sum()) * RES * RES
