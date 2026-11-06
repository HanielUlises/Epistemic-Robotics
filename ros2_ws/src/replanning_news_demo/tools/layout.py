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
The residence, in one place.

A single storey 24 by 16 metres. The south half holds the lounge to the west,
the dining room in the middle and the nurses' station to the east, all opening
onto a hall that runs the width of the building. North of the hall a band of
linen rooms and a kitchenette, 2.6 m deep, carries the fire doors: three
passages cross it, each closed by a door near its north end. The west and east
passages are straight, so a robot at a passage's mouth sees the door down its
length. The middle passage turns twice, west to east and north again, so its
door cannot be seen from its mouth; the kitchenette beside it has a hatch onto
the passage's last leg, and from the hatch the door can be seen. North of the
band a corridor runs the width of the building, and four residents' rooms open
off it. The resident whose medicine the courier carries is in the north-west
room.

An open door's leaf lies against the corridor's south wall beside its
opening; a shut one stands across the passage.

Coordinates are metres in the world frame, which is every robot's odometry
frame. tools/check_views.py casts the rays and checks what the domain assumes.
"""

import math

BUILDING = (-12.0, -8.0, 12.0, 8.0)
WALL = 0.15
WALL_H = 1.3
BAND = (0.0, 2.6)                 # the band of linen rooms that carries the fire doors
HALL_S = -2.2                     # the hall's south wall
ROOMS_S = 4.4                     # the residents' rooms' south wall
GAP = 1.2                         # every passage is this wide
DOOR_Y = 2.45                     # where a shut door stands, in every passage
LEAF = (1.0, 0.16)                # Door_01: 1 m wide, 0.16 m thick

# Each passage's centreline, from its mouth in the hall to the corridor.
PASSAGE = {'w': [(-8.0, 0.0), (-8.0, 2.6)],
           'm': [(0.0, 0.0), (0.0, 1.3), (1.8, 1.3), (1.8, 2.6)],
           'e': [(7.0, 0.0), (7.0, 2.6)]}
DOOR_X = {'w': -8.0, 'm': 1.8, 'e': 7.0}
KITCHENETTE = (2.4, 0.0, 5.4, 2.6)
HATCH = (1.5, 2.4)                # the hatch in the kitchenette's west wall, as a span of y

START = {'c': (8.6, -4.4, math.radians(180)),       # the courier at the nurses' station
         's': (3.1, 1.6, math.radians(150)),        # the scout at the hatch
         'k': (10.4, 3.5, math.radians(180))}       # the cleaner, in the corridor's east end
VANTAGE = {'w': (-8.0, -0.55), 'e': (7.0, -0.55), 'm': START['s'][:2]}
GOAL = (-8.7, 6.0)                                  # beside the resident's bed
SPEED = 0.17                                        # m/s, what a drive averages


def _box(x0, y0, x1, y1):
    return ((x0 + x1) / 2, (y0 + y1) / 2, abs(x1 - x0), abs(y1 - y0))


def _hwall(y, x0, x1, cuts=()):
    """A wall along y from x0 to x1, less the openings in cuts, as boxes."""
    t = WALL / 2
    xs = [x0]
    for a, b in sorted(cuts):
        xs += [a, b]
    xs.append(x1)
    return [_box(a, y - t, b, y + t) for a, b in zip(xs[0::2], xs[1::2]) if b - a > 1e-6]


def _vwall(x, y0, y1, cuts=()):
    t = WALL / 2
    ys = [y0]
    for a, b in sorted(cuts):
        ys += [a, b]
    ys.append(y1)
    return [_box(x - t, a, x + t, b) for a, b in zip(ys[0::2], ys[1::2]) if b - a > 1e-6]


def walls():
    """(cx, cy, sx, sy) boxes, WALL_H high."""
    x0, y0, x1, y1 = BUILDING
    t = WALL
    h = GAP / 2
    out = [_box(x0 - t, y0 - t, x1 + t, y0), _box(x0 - t, y1, x1 + t, y1 + t),
           _box(x0 - t, y0, x0, y1), _box(x1, y0, x1 + t, y1)]
    # the hall's south wall: the lounge's wide opening, the dining room's, the station's door
    out += _hwall(HALL_S, x0, x1, cuts=[(-9.0, -5.0), (-2.6, 2.6), (4.6, 5.6)])
    out += _vwall(-3.5, y0, HALL_S)
    out += _vwall(3.5, y0, HALL_S)
    # the band: its south face with the three mouths and the kitchenette's door
    out += _hwall(BAND[0], x0, x1, cuts=[(-8.0 - h, -8.0 + h), (-h, h), (3.4, 4.4), (7.0 - h, 7.0 + h)])
    # its north face with the three passages' ends
    out += _hwall(BAND[1], x0, x1, cuts=[(-8.0 - h, -8.0 + h), (1.8 - h, 1.8 + h), (7.0 - h, 7.0 + h)])
    # the straight passages' sides
    for x in (-8.0, 7.0):
        out += _vwall(x - h, BAND[0], BAND[1])
        out += _vwall(x + h, BAND[0], BAND[1])
    # the middle passage: north from the hall, east, north again
    out += _vwall(-h, BAND[0], 1.3 + h)                    # the first leg's west side
    out += _hwall(1.3 + h, -h, 1.8 - h)                    # the second leg's north side
    out += _vwall(1.8 - h, 1.3 + h, BAND[1])               # the last leg's west side
    out += _vwall(h, BAND[0], 1.3 - h)                     # the first leg's east side
    out += _hwall(1.3 - h, h, 1.8 + h)                     # the second leg's south side
    out += _vwall(1.8 + h, 1.3 - h, BAND[1], cuts=[HATCH])  # the kitchenette's west wall, with the hatch
    out += _vwall(KITCHENETTE[2], BAND[0], BAND[1])        # the kitchenette's east wall
    # the residents' rooms
    out += _hwall(ROOMS_S, x0, x1, cuts=[(-9.4, -8.4), (-3.5, -2.5), (2.5, 3.5), (8.5, 9.5)])
    for x in (-6.0, 0.0, 6.0):
        out += _vwall(x, ROOMS_S, y1)
    return out


def linen():
    """The band's closed linen rooms, as (x0, y0, x1, y1): solid for the simulator."""
    h = GAP / 2
    b0, b1 = BAND[0] + WALL / 2, BAND[1] - WALL / 2
    return [(-12.0, b0, -8.0 - h - WALL / 2, b1), (-8.0 + h + WALL / 2, b0, -h - WALL / 2, b1),
            (KITCHENETTE[2] + WALL / 2, b0, 7.0 - h - WALL / 2, b1), (7.0 + h + WALL / 2, b0, 12.0, b1),
            (h + WALL / 2, b0, 1.8 + h - WALL / 2 + 1e-3, 1.3 - h - WALL / 2),
            (-h + WALL / 2, 1.3 + h + WALL / 2, 1.8 - h - WALL / 2, b1)]


def leaf(d, open_):
    """The door leaf of passage d: (cx, cy, yaw) for Door_01, whose leaf runs along its own x."""
    x = DOOR_X[d]
    if not open_:
        return (x, DOOR_Y, 0.0)
    # swung through 90 degrees about the east jamb, into the corridor, flat against its south wall
    hinge_x = x + GAP / 2
    return (hinge_x + LEAF[0] / 2 + 0.02, BAND[1] + WALL / 2 + LEAF[1] / 2 + 0.01, 0.0)


def floors():
    """(name, x0, y0, x1, y1, material) floor panels, drawn just above the ground."""
    x0, y0, x1, y1 = BUILDING
    return [
        ('lounge', x0, y0, -3.5, HALL_S, 'wood'),
        ('dining', -3.5, y0, 3.5, HALL_S, 'oak'),
        ('station', 3.5, y0, x1, HALL_S, 'tile'),
        ('hall', x0, HALL_S, x1, BAND[0], 'hall'),
        ('kitchenette', *KITCHENETTE, 'tile'),
        ('corridor', x0, BAND[1], x1, ROOMS_S, 'hall'),
        ('room1', x0, ROOMS_S, -6.0, y1, 'carpet'),
        ('room2', -6.0, ROOMS_S, 0.0, y1, 'wood'),
        ('room3', 0.0, ROOMS_S, 6.0, y1, 'wood'),
        ('room4', 6.0, ROOMS_S, x1, y1, 'wood'),
    ]


def passage_floor():
    """Each passage's floor, (d, x0, y0, x1, y1) panels, tinted so a viewer tells them apart."""
    h = GAP / 2
    return [('w', -8.0 - h, 0.0, -8.0 + h, BAND[1]), ('e', 7.0 - h, 0.0, 7.0 + h, BAND[1]),
            ('m', -h, 0.0, h, 1.3 + h), ('m', -h, 1.3 - h, 1.8 + h, 1.3 + h), ('m', 1.8 - h, 1.3 - h, 1.8 + h, BAND[1])]


# The AWS residential models placed in the residence: (model, x, y, yaw). Off
# every route by tools/check_views.py.
FURNITURE = [
    # the resident's room, north-west
    ('Bed_01', -10.75, 6.65, math.radians(-90)), ('NightStand_01', -11.55, 4.95, math.radians(90)),
    ('Wardrobe_01', -6.65, 7.45, math.radians(-90)), ('ChairD_01', -7.2, 5.3, math.radians(135)),
    ('Carpet_01', -9.2, 6.3, 0.0),
    # the second room
    ('Bed_01', -4.75, 6.65, math.radians(-90)), ('NightStand_01', -5.55, 4.95, math.radians(90)),
    ('ReadingDesk_01', -0.55, 6.6, math.radians(180)), ('ChairA_01', -1.2, 6.6, 0.0),
    # the third room
    ('Bed_01', 1.25, 6.65, math.radians(-90)), ('Wardrobe_01', 5.35, 7.45, math.radians(-90)),
    ('NightStand_01', 0.45, 4.95, math.radians(90)), ('FitnessEquipment_01', 4.9, 5.4, 0.0),
    # the fourth room
    ('Bed_01', 7.25, 6.65, math.radians(-90)), ('NightStand_01', 6.45, 4.95, math.radians(90)),
    ('SofaB_01', 11.3, 5.2, math.radians(180)), ('Wardrobe_01', 11.35, 7.45, math.radians(-90)),
    # the lounge
    ('Sofa_01', -10.6, -6.6, 0.0), ('CoffeeTable_01', -8.4, -5.4, math.radians(90)),
    ('SofaB_01', -6.6, -6.9, math.radians(150)), ('TVCabinet_01', -4.05, -5.6, math.radians(180)),
    ('Carpet_01', -8.6, -5.6, math.radians(90)), ('Trash_01', -4.2, -7.5, 0.0),
    # the dining room
    ('KitchenTable_01', 0.0, -5.6, math.radians(90)),
    ('ChairA_01', -0.7, -6.15, math.radians(90)), ('ChairA_01', 0.7, -6.15, math.radians(90)),
    ('ChairA_01', -0.7, -5.05, math.radians(-90)), ('ChairA_01', 0.7, -5.05, math.radians(-90)),
    ('ChairD_01', -2.6, -7.3, math.radians(45)), ('ChairD_01', 2.6, -7.3, math.radians(135)),
    # the nurses' station
    ('ReadingDesk_01', 10.9, -6.0, math.radians(180)), ('ChairA_01', 10.2, -6.0, 0.0),
    ('Refrigerator_01', 11.45, -3.2, math.radians(90)), ('ShoeRack_01', 6.5, -7.75, 0.0),
    ('ReadingDesk_01', 4.2, -6.6, 0.0),
    # the kitchenette
    ('CookingBench_01', 5.0, 1.9, math.radians(180)), ('Refrigerator_01', 4.8, 0.55, math.radians(180)),
    # the corridor
    ('Vase_01', -11.7, 3.0, 0.0), ('Vase_01', 11.7, 4.0, 0.0),
]


def roadmap():
    """Named places, and the edges the drives route over: (nodes, south edges, north edges).

    The south graph is the hall and the rooms off it, with each passage as far as
    its door, so that a robot stopped anywhere south of the doors is routed out by
    the way it came. The north graph is the corridor and the resident's room.
    Crossings run on fixed routes, through their own passage and nowhere else."""
    n = {'st': START['c'][:2], 'stn': (5.1, -3.3), 'sth': (5.1, -1.1),
         'hw': (-8.0, -1.1), 'hlw': (-5.0, -1.1), 'hm': (0.0, -1.1), 'hk': (3.9, -1.1), 'he': (7.0, -1.1),
         'hee': (10.0, -1.1),
         'vw': VANTAGE['w'], 'vm': (0.0, -0.55), 've': VANTAGE['e'],
         'kd': (3.9, 0.5), 'ks': VANTAGE['m'],
         'pw': (-8.0, 1.4), 'pm1': (0.0, 1.3), 'pm2': (1.8, 1.3), 'pm3': (1.8, 1.9), 'pe': (7.0, 1.4),
         'nw': (-8.0, 3.5), 'nm': (1.8, 3.5), 'ne': (7.0, 3.5), 'nr': (-8.9, 3.5), 'n2': (-3.0, 3.5),
         'rd': (-8.9, 4.4), 'r': GOAL, 'k0': START['k'][:2]}
    south = [('st', 'stn'), ('stn', 'sth'), ('sth', 'hk'), ('sth', 'he'), ('he', 'hee'), ('hk', 'hm'),
             ('hm', 'hlw'), ('hlw', 'hw'), ('hw', 'vw'), ('hm', 'vm'), ('he', 've'), ('hk', 'kd'), ('kd', 'ks'),
             ('vw', 'pw'), ('vm', 'pm1'), ('pm1', 'pm2'), ('ve', 'pe')]
    north = [('nw', 'nr'), ('nr', 'rd'), ('rd', 'r'), ('nw', 'n2'), ('n2', 'nm'), ('nm', 'ne'), ('ne', 'k0')]
    return n, south, north


CROSS = {'w': ['vw', 'pw', 'nw'], 'm': ['vm', 'pm1', 'pm2', 'pm3', 'nm'], 'e': ['ve', 'pe', 'ne']}


def route(graph, a, b):
    """The shortest route from place a to place b over the given edges."""
    nodes = roadmap()[0]
    adj = {}
    for u, v in graph:
        w = math.dist(nodes[u], nodes[v])
        adj.setdefault(u, []).append((v, w))
        adj.setdefault(v, []).append((u, w))
    dist, prev, todo = {a: 0.0}, {}, {a}
    while todo:
        u = min(todo, key=dist.get)
        todo.discard(u)
        if u == b:
            break
        for v, w in adj.get(u, []):
            if dist[u] + w < dist.get(v, math.inf):
                dist[v] = dist[u] + w
                prev[v] = u
                todo.add(v)
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1]


def length(path):
    nodes = roadmap()[0]
    return sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))


def trip(path):
    """The declared duration of a route, in whole seconds."""
    return max(1, round(length(path) / SPEED))
