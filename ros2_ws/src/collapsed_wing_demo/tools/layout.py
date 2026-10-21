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
The collapsed wing, in one place.

A building 20 by 14 metres. Its south half holds the entrance hall in the
middle, where the medic m starts, and a wing on each side: the west wing,
which the scout s2 enters by the west entrance, and the east wing, which s1
enters by the east. A wall at y = 3 closes off the north wing, where the
victim lies. Three ways cross it. The west doorway opens from the west wing
and the east doorway from the east wing. Between them, from the north side of
the hall, a middle passage winds through collapsed rooms in three long legs,
west, east and west again, about 20 m in all: the longest way, and one no spot
sees the whole of.

Rubble may block any of the three, and at least one is clear. Rubble in a
doorway stands in its gap; rubble in the middle passage stands halfway along
its middle leg, out of sight from either end.

Each scout looks at its doorway from a spot beside the medic's way to it, so
that it does not stand in the way. Desks, columns and debris stand in the
wings, the hall and the north wing, off every route: they give the scan
matcher something to match and the lasers something to see.

Coordinates are metres in the world frame, which is every robot's odometry
frame. tools/check_views.py casts the rays and checks what the domain assumes.
"""

import math

BUILDING = (-10.0, -7.0, 10.0, 7.0)
WALL = 0.15
NORTH_Y = 3.0                                 # the wall that closes off the north wing
GAP = 1.2                                     # every doorway and the passage are this wide
DOOR_X = {'w': -5.5, 'e': 5.5}               # centre of each doorway in the north wall
HALL = (-3.5, -7.0, 3.5, -2.0)                # the entrance hall
HALL_DOOR_Y = -4.4                            # the hall's doors to the wings, centred here
BLOCK = (-3.5, -2.0, 3.5, NORTH_Y)            # the collapsed rooms the middle passage winds through

# The middle passage's centreline: in from the hall at its east end, three long
# legs, and out into the north wing at its west end.
PASSAGE = [(2.6, -2.0), (2.6, -1.0), (-2.6, -1.0), (-2.6, 0.4), (2.6, 0.4), (2.6, 1.8), (-2.6, 1.8),
           (-2.6, NORTH_Y)]

# Rubble: where it stands when a way is blocked, as (cx, cy, sx, sy).
RUBBLE = {'w': (DOOR_X['w'], NORTH_Y + 0.35, 1.12, 0.6),
          'e': (DOOR_X['e'], NORTH_Y + 0.35, 1.12, 0.6),
          'm': (0.0, 0.4, 0.6, 1.12)}
RUBBLE_H = 0.45

# What a scout's map must decide, per doorway: the gap and the half metre beyond it.
REGION = {d: (DOOR_X[d] - 0.5, NORTH_Y + 0.05, DOOR_X[d] + 0.5, NORTH_Y + 0.75) for d in ('w', 'e')}

START = {'m': (0.0, -5.6, math.radians(90)),
         's1': (7.6, -6.0, math.radians(90)),
         's2': (-7.6, -6.0, math.radians(90))}
VANTAGE = {'s1': (6.9, 1.6), 's2': (-6.9, 1.6)}  # beside the medic's way to the doorway
VICTIM = (0.0, 5.6)
SPEED = 0.16                                  # m/s, what the drive averages over a route


def walls():
    """(cx, cy, sx, sy) boxes: the outer walls, the north wall with its three
    openings, the hall's walls with their doors, and the collapsed block."""
    x0, y0, x1, y1 = BUILDING
    t = WALL
    out = []

    def box(xa, ya, xb, yb):
        out.append(((xa + xb) / 2, (ya + yb) / 2, abs(xb - xa), abs(yb - ya)))
    box(x0 - t, y0 - t, x1 + t, y0)
    box(x0 - t, y1, x1 + t, y1 + t)
    box(x0 - t, y0, x0, y1)
    box(x1, y0, x1 + t, y1)
    # the north wall, with the west and east doorways and the passage's exit
    h = GAP / 2
    px = PASSAGE[-1][0]
    cuts = sorted([DOOR_X['w'] - h, DOOR_X['w'] + h, px - h, px + h, DOOR_X['e'] - h, DOOR_X['e'] + h])
    xs = [x0] + cuts + [x1]
    for a, b in zip(xs[0::2], xs[1::2]):
        box(a, NORTH_Y, b, NORTH_Y + t)
    # the hall: west and east walls with a door to each wing, and its north wall
    hx0, hy0, hx1, hy1 = HALL
    for x in (hx0, hx1):
        box(x - t / 2, hy0, x + t / 2, HALL_DOOR_Y - h)
        box(x - t / 2, HALL_DOOR_Y + h, x + t / 2, hy1)
    # the collapsed block, solid but for the passage
    for s in block_solids():
        box(*s)
    return out


def block_solids(step=0.1):
    """The collapsed block less the passage, as boxes (x0, y0, x1, y1): the
    block on a grid of `step`, each row cut into runs outside the passage, and
    rows with the same runs merged."""
    bx0, by0, bx1, by1 = BLOCK
    h = GAP / 2

    def in_passage(x, y):
        for (ax, ay), (cx, cy) in zip(PASSAGE, PASSAGE[1:]):
            if min(ax, cx) - h <= x <= max(ax, cx) + h and min(ay, cy) - h <= y <= max(ay, cy) + h:
                return True
        return False
    nx, ny = round((bx1 - bx0) / step), round((by1 - by0) / step)
    rows = []
    for j in range(ny):
        y = by0 + (j + 0.5) * step
        runs, start = [], None
        for i in range(nx + 1):
            solid = i < nx and not in_passage(bx0 + (i + 0.5) * step, y)
            if solid and start is None:
                start = i
            if not solid and start is not None:
                runs.append((start, i))
                start = None
        rows.append(tuple(runs))
    out, j = [], 0
    while j < ny:
        k = j
        while k + 1 < ny and rows[k + 1] == rows[j]:
            k += 1
        for a, b in rows[j]:
            out.append((round(bx0 + a * step, 3), round(by0 + j * step, 3), round(bx0 + b * step, 3),
                        round(by0 + (k + 1) * step, 3)))
        j = k + 1
    return out


def clutter():
    """Desks, columns and debris, off every route, as (cx, cy, sx, sy, sz)."""
    out = []
    for k in (-1, 1):                         # each wing, mirrored
        out += [(k * 5.6, -6.2, 1.4, 0.6, 0.75), (k * 4.3, -1.0, 0.35, 0.35, 2.0), (k * 9.0, 0.0, 0.8, 1.6, 0.75),
                (k * 4.4, 2.2, 0.9, 0.5, 0.5), (k * 9.4, -3.0, 0.6, 1.4, 1.0), (k * 6.5, -2.6, 0.35, 0.35, 2.0)]
        out += [(k * 2.4, -6.3, 1.0, 0.6, 0.75), (k * 4.2, 5.6, 1.2, 0.6, 0.75), (k * 8.6, 6.3, 1.4, 0.6, 0.5),
                (k * 7.2, 3.9, 0.35, 0.35, 2.0), (k * 1.6, -5.0, 0.35, 0.35, 2.0)]
    out += [(0.0, 6.6, 1.6, 0.5, 0.6)]
    return out


def roadmap():
    """Named places; the routes below are lists of them."""
    n = {'m0': START['m'][:2], 'hall': (0.0, -3.6), 'hall_e': (2.6, -3.0),
         's1_0': START['s1'][:2], 's2_0': START['s2'][:2]}
    for k, d in ((-1, 'w'), (1, 'e')):
        n[f'hd_{d}'] = (k * 3.5, HALL_DOOR_Y)
        n[f'wing_{d}'] = (k * DOOR_X['e'], HALL_DOOR_Y)
        n[f'sw_{d}'] = (k * 7.6, HALL_DOOR_Y)
        n[f'below_{d}'] = (k * DOOR_X['e'], 1.5)
        n[f'door_{d}'] = (k * DOOR_X['e'], NORTH_Y)
        n[f'above_{d}'] = (k * DOOR_X['e'], 4.6)
        n[f'v_{d}'] = VANTAGE['s1' if d == 'e' else 's2']
    for i, p in enumerate(PASSAGE):
        n[f'p{i}'] = p
    n['above_p'] = (PASSAGE[-1][0], 4.6)
    n['above_m'] = (0.0, 4.6)
    n['victim'] = VICTIM
    return n


ROUTE = {
    # the scouts, each to its doorway's vantage
    's1': ['s1_0', 'sw_e', 'v_e'],
    's2': ['s2_0', 'sw_w', 'v_w'],
    # the medic, by each way, from the hall
    'm_w': ['m0', 'hall', 'hd_w', 'wing_w', 'below_w', 'door_w', 'above_w', 'above_m', 'victim'],
    'm_e': ['m0', 'hall', 'hd_e', 'wing_e', 'below_e', 'door_e', 'above_e', 'above_m', 'victim'],
    'm_m': ['m0', 'hall', 'hall_e'] + [f'p{i}' for i in range(len(PASSAGE))] + ['above_p', 'above_m', 'victim'],
}


def length(path):
    nodes = roadmap()
    return sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))


def trip(route):
    """The declared duration of a route, in whole seconds."""
    return round(length(ROUTE[route]) / SPEED)
