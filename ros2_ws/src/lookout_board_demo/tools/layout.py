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
The hall of the lookout board, in one place.

The zone is a square in the middle of a walled room. Each of the room's four
walls has a window, and each window opens onto a short embrasure, a channel
between two side walls, so that the zone can be seen through a window only
from in front of its embrasure: a robot passing along the corridor sees the
embrasure's walls and nothing of the room. In front of each embrasure is a
lookout. A screen, when it stands, closes a window from the inside of its
embrasure. A ring corridor runs round the room, and the robot starts in its
south-west corner beside a status board, a back panel with four slots in front
of it; slot k holds a post exactly when the view from lookout k is clear.

Every other file reads its geometry from here: the world, the drive node's
roadmap, the perception's regions and the film's shots. Coordinates are metres
in the world frame, which is also the map frame.
"""

import math

LOOKOUTS = ('n', 'e', 's', 'w')           # lookout k = 1..4 in this order
HALL = (-4.6, -4.6, 4.6, 4.6)             # inner faces of the hall walls
ROOM = 1.5                                # half side of the room, inner face
WALL = 0.1                                # wall thickness
WINDOW = 0.5                              # half width of a window
EMBRASURE = 0.8                           # depth of the channel outside a window
ZONE = 0.4                                # half side of the zone
LOOKOUT_R = ROOM + WALL + EMBRASURE + 0.45  # centre of the room to a lookout
SCREEN_R = ROOM + WALL + 0.15             # where a screen stands, inside its channel
START = (-3.45, -3.3, math.radians(135))
BOARD = (-3.6, -2.6)                      # where the robot stands to read the board
BOARD_PANEL_X = -4.55                     # the panel's face, against the west wall
SLOT_Y = (-1.85, -2.35, -2.85, -3.35)     # slot k's centre along the panel
SLOT_HALF = 0.12                          # half side of a slot's region
POST_X = BOARD_PANEL_X + 0.25             # where a lit slot's post stands
CRATE = 0.22                              # half side of the crate in a blocked zone

UNIT = {'n': (0.0, 1.0), 'e': (1.0, 0.0), 's': (0.0, -1.0), 'w': (-1.0, 0.0)}


def lookout(k):
    """Where the robot stands for lookout k (1-based), facing the zone."""
    ux, uy = UNIT[LOOKOUTS[k - 1]]
    return (ux * LOOKOUT_R, uy * LOOKOUT_R)


def screen_box(k):
    """[x0, y0, x1, y1] of the place screen k stands in."""
    ux, uy = UNIT[LOOKOUTS[k - 1]]
    cx, cy = ux * SCREEN_R, uy * SCREEN_R
    if ux:
        return (cx - 0.04, -WINDOW, cx + 0.04, WINDOW)
    return (-WINDOW, cy - 0.04, WINDOW, cy + 0.04)


def window_region(k):
    """The channel strip a check-k observation reads: occupied when screened."""
    ux, uy = UNIT[LOOKOUTS[k - 1]]
    r0, r1 = ROOM + WALL + 0.05, ROOM + WALL + 0.30
    w = WINDOW - 0.12
    if ux > 0:
        return (r0, -w, r1, w)
    if ux < 0:
        return (-r1, -w, -r0, w)
    if uy > 0:
        return (-w, r0, w, r1)
    return (-w, -r1, w, -r0)


def zone_region():
    return (-ZONE, -ZONE, ZONE, ZONE)


def slot_region(k):
    y = SLOT_Y[k - 1]
    return (POST_X - SLOT_HALF, y - SLOT_HALF, POST_X + SLOT_HALF, y + SLOT_HALF)


def walls():
    """(cx, cy, sx, sy) boxes: hall, room with four windows, embrasures."""
    x0, y0, x1, y1 = HALL
    t = WALL
    out = [((x0 + x1) / 2, y1 + t / 2, x1 - x0 + 2 * t, t),
           ((x0 + x1) / 2, y0 - t / 2, x1 - x0 + 2 * t, t),
           (x1 + t / 2, (y0 + y1) / 2, t, y1 - y0),
           (x0 - t / 2, (y0 + y1) / 2, t, y1 - y0)]
    r, w = ROOM + t / 2, WINDOW
    seg = ROOM + t - w                        # each wall is two segments either side of the window
    mid = w + seg / 2
    for ux, uy in UNIT.values():
        if ux:
            out += [(ux * r, mid, t, seg), (ux * r, -mid, t, seg)]
        else:
            out += [(mid, uy * r, seg, t), (-mid, uy * r, seg, t)]
    # embrasure side walls, from the room wall out to the channel mouth
    a, b = ROOM + t, ROOM + t + EMBRASURE
    c = (a + b) / 2
    for ux, uy in UNIT.values():
        for s in (-1, 1):
            if ux:
                out.append((ux * c, s * (w + t / 2), EMBRASURE, t))
            else:
                out.append((s * (w + t / 2), uy * c, t, EMBRASURE))
    return out


# The roadmap the drive node plans over: corners of the ring, the lookouts, the
# board and the start. Edges are straight segments clear of every wall.
CORNER = 3.55


def roadmap():
    c = CORNER
    nodes = {'start': START[:2], 'board': BOARD,
             'c_sw': (-c, -c), 'c_se': (c, -c), 'c_ne': (c, c), 'c_nw': (-c, c)}
    for k in range(1, 5):
        nodes[f'l{k}'] = lookout(k)
    # The board is a spur off the start: reading it is a detour and never a
    # short cut to anywhere.
    edges = [('start', 'c_sw'), ('start', 'board'),
             ('l1', 'c_nw'), ('l1', 'c_ne'), ('l2', 'c_ne'), ('l2', 'c_se'),
             ('l3', 'c_se'), ('l3', 'c_sw'), ('l4', 'c_sw'), ('l4', 'c_nw'),
             ('start', 'l3'), ('start', 'l4')]
    return nodes, edges


def shortest(a, b):
    """Shortest route over the roadmap, as a list of node names, and its length."""
    nodes, edges = roadmap()
    adj = {n: [] for n in nodes}
    for u, v in edges:
        d = math.dist(nodes[u], nodes[v])
        adj[u].append((v, d))
        adj[v].append((u, d))
    best, prev, todo = {a: 0.0}, {}, {a}
    while todo:
        u = min(todo, key=best.get)
        todo.remove(u)
        if u == b:
            break
        for v, d in adj[u]:
            if best[u] + d < best.get(v, math.inf):
                best[v] = best[u] + d
                prev[v] = u
                todo.add(v)
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1], best[b]
