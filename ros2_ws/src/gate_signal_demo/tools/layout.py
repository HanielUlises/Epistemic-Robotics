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
The hall of the gate signal, in one place.

Two bays, X to the west and Y to the east, each a walled room with a door
onto the south corridor. Between the bays a block of racking runs north from
the corridor, cut by two passages, one beside each bay, each closed at its
north end and opening by a side door into the bay beside it. Inside the
racking, between the passages, is r2's booth, joined to each passage by a
slit at the height of the gate. r1, the carrier, starts in the south corridor
before an order board of two slots.

r1 reaches its bay by the back route, along the south corridor and in by the
south door, or through its passage, past the gate at the mouth of the slit,
and in by the side door; the second is the longer. From its station in the
booth r2 sees, through the slits, the two gates and nothing of the south
corridor, of the bays' insides or of the back routes. r2 itself leaves the
booth through a slit and reaches a bay through that bay's passage.

Coordinates are metres in the world frame, which is also every robot's odometry
frame.
"""

import math

HALL = (-6.0, -5.0, 6.0, 3.4)
WALL = 0.1
SOUTH, NORTH = -1.8, 3.4                 # y extent of the bays, the passages and the racking
BAY_X = (-6.0, SOUTH, -3.1, NORTH)       # west room, inner extent
DOOR_HALF = 0.6
DOOR_X = -4.5                            # centre of bay X's south door
PASSAGE = (-3.1, -1.7)                   # x extent of passage X
PASSAGE_X = -2.4
SIDE_DOOR = (1.85, 3.05)                 # y extent of the side door from passage X into bay X
GATE_Y = -0.9                            # where r1 stops in its passage, at the mouth of the slit
BOOTH = (-0.8, -1.6, 0.8, -0.2)          # r2's booth inside the racking
SLIT = (GATE_Y - 0.45, GATE_Y + 0.45)    # y extent of the slits from the booth to the passages
START = (0.0, -3.7, math.radians(-90))   # r1, facing the order board
ORDER_BOARD_Y = -4.75                    # face of the order board
ORDER_SLOT_X = (-0.45, 0.45)             # slot X, slot Y
ORDER_POST_Y = ORDER_BOARD_Y + 0.25
STATION = (0.0, GATE_Y, math.radians(90))   # r2, in the booth, facing neither slit
BAY_CENTRE_Y = (SOUTH + NORTH) / 2


def mirror(x):
    return -x


def gate(b):
    """Where r1 stands in gate b ('x' or 'y')."""
    return (PASSAGE_X if b == 'x' else mirror(PASSAGE_X), GATE_Y)


def gate_region(b):
    x, y = gate(b)
    return (x - 0.35, y - 0.35, x + 0.35, y + 0.35)


def bay_centre(b):
    x = (BAY_X[0] + BAY_X[2]) / 2
    return (x if b == 'x' else mirror(x), BAY_CENTRE_Y)


def bay_region(b):
    x0, y0, x1, y1 = BAY_X
    return (x0, y0, x1, y1) if b == 'x' else (mirror(x1), y0, mirror(x0), y1)


def order_slot_region(b):
    x = ORDER_SLOT_X[0] if b == 'x' else ORDER_SLOT_X[1]
    return (x - 0.13, ORDER_POST_Y - 0.13, x + 0.13, ORDER_POST_Y + 0.13)


def walls():
    """(cx, cy, sx, sy) boxes."""
    x0, y0, x1, y1 = HALL
    t = WALL
    out = [((x0 + x1) / 2, y1 + t / 2, x1 - x0 + 2 * t, t), ((x0 + x1) / 2, y0 - t / 2, x1 - x0 + 2 * t, t),
           (x1 + t / 2, (y0 + y1) / 2, t, y1 - y0), (x0 - t / 2, (y0 + y1) / 2, t, y1 - y0)]

    def box(xa, ya, xb, yb):
        out.append(((xa + xb) / 2, (ya + yb) / 2, abs(xb - xa), abs(yb - ya)))
    for s in (1, -1):                      # 1: bay X and passage X (west); -1: east
        def X(v):
            return v * s
        # the bay's south wall, with its door; the hall's wall closes it to the north
        box(X(-6.0), SOUTH - t, X(DOOR_X - DOOR_HALF), SOUTH)
        box(X(DOOR_X + DOOR_HALF), SOUTH - t, X(PASSAGE[0]), SOUTH)
        # the wall between bay and passage, with the side door
        box(X(PASSAGE[0]), SOUTH - t, X(PASSAGE[0] + t), SIDE_DOOR[0])
        box(X(PASSAGE[0]), SIDE_DOOR[1], X(PASSAGE[0] + t), NORTH)
    xr = -PASSAGE[1]                       # inner face of the passages
    bx0, by0, bx1, by1 = BOOTH
    box(-xr, SOUTH - t, xr, by0)           # south of the booth: solid across
    for s in (1, -1):                      # beside the booth: solid but for the slit
        a, b = sorted((s * xr, s * bx1))
        box(a, by0, b, SLIT[0])
        box(a, SLIT[1], b, by1)
    box(-xr, by1, xr, NORTH)               # north of the booth: solid
    return out


def roadmap():
    """Named places, and straight segments clear of every wall."""
    n = {'start': START[:2], 'station': STATION[:2],
         's_x': (DOOR_X, -3.1), 's_y': (-DOOR_X, -3.1), 'sd_x': (DOOR_X, -2.3), 'sd_y': (-DOOR_X, -2.3),
         'bay_x': bay_centre('x'), 'bay_y': bay_centre('y'),
         'pass_x': (PASSAGE_X, -2.4), 'pass_y': (-PASSAGE_X, -2.4),
         'gate_x': gate('x'), 'gate_y': gate('y'),
         'side_x': (PASSAGE_X, sum(SIDE_DOOR) / 2), 'side_y': (-PASSAGE_X, sum(SIDE_DOOR) / 2),
         'in_x': (PASSAGE[0] - 0.7, sum(SIDE_DOOR) / 2), 'in_y': (-(PASSAGE[0] - 0.7), sum(SIDE_DOOR) / 2)}
    e = [('start', 's_x'), ('start', 's_y'), ('s_x', 'sd_x'), ('s_y', 'sd_y'), ('sd_x', 'bay_x'), ('sd_y', 'bay_y'),
         ('start', 'pass_x'), ('start', 'pass_y'), ('pass_x', 'gate_x'), ('pass_y', 'gate_y'),
         ('gate_x', 'side_x'), ('gate_y', 'side_y'), ('side_x', 'in_x'), ('side_y', 'in_y'),
         ('in_x', 'bay_x'), ('in_y', 'bay_y'), ('station', 'gate_x'), ('station', 'gate_y')]
    return n, e


def shortest(a, b, banned=()):
    nodes, edges = roadmap()
    adj = {k: [] for k in nodes}
    for u, v in edges:
        if u in banned or v in banned:
            continue
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
                best[v], prev[v] = best[u] + d, u
                todo.add(v)
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1], best[b]
