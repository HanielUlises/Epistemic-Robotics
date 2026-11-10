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
The ward, in one place.

A hospital floor 28 by 18.4 metres, at night. The south half holds the day
room to the west, the porters' lodge, the treatment room and the pharmacy to
the east, all opening onto a hall that runs the width of the building. North
of the hall a band of linen rooms and sluices, 5 m deep, carries four fire
passages, A to D from west to east: each runs straight through the band and
is closed by a fire door near its north end. A robot at a passage's mouth, in
the hall, looks down the passage at its door from more than five metres; a
robot that has driven in to the mark before the door reads it from 1.2 m.
North of the band a corridor runs the width of the building, and four ward
bays open off it.

On his round the night porter shuts two of the four doors; which two, nobody
on the floor has seen.

A passage is as wide as a bed is long: a bed-mover that has driven in cannot
turn round, and backs out at a crawl.

Every robot keeps to its own line along the hall (LANES), so that robots pass
one another without giving way; the places of the hall are named alike for
all, and stand on each robot's own line.

Coordinates are metres in the world frame, which is every robot's odometry
frame. tools/check_views.py casts the rays and checks what the domain assumes.
"""

import math

SHIFT = 2.4                      # how much deeper the band is than the first ward's, and all north of it
BUILDING = (-14.0, -8.0, 14.0, 8.0 + SHIFT)
WALL = 0.15
WALL_H = 1.3
HALL_S = -2.2                    # the hall's south wall
BAND = (0.0, 2.6 + SHIFT)        # the band of linen rooms and sluices that carries the fire passages
WARDS_S = 4.4 + SHIFT            # the ward bays' south wall
GAP = 1.2                        # every passage is this wide
LEAF = (1.16, 0.06)              # a fire door's leaf: across the passage, and its thickness
LEAF_Y = 4.4                     # where the doors stand, across their passages
MARK_Y = 3.2                     # the mark before a door, where it is read from close by
HALL_Y = -0.8                    # the middle line of the hall
CORRIDOR_Y = 3.5 + SHIFT         # the line robots drive along the corridor
FAR = 2.5                        # a short-sighted laser returns nothing from beyond this
# each robot's own line along the hall: the bed-movers nearest the band, the scouts south of them
LANES = {'b': -0.45, 'g': -0.85, 's': -1.25, 't': -1.65, 'u': -1.65}
# where each robot stands at a mouth, east of the passage's middle line: a scout a little to one side, so
# that two scouts see past each other, a bed-mover beside the mouth, out of every scout's view of the door
ASIDE = {'b': -0.9, 'g': -0.9, 's': -0.3, 't': 0.3, 'u': 0.3}

DOORS = ('a', 'b', 'c', 'd')
DOOR_X = {'a': -11.0, 'b': -4.5, 'c': 2.0, 'd': 8.0}      # each passage's middle line

# The south rooms: (name, x0, x1, the span of their opening onto the hall).
SOUTH_ROOMS = [('dayroom', -14.0, -7.5, (-12.3, -10.7)), ('lodge', -7.5, -1.5, (-3.6, -2.4)),
               ('treatment', -1.5, 5.0, (-1.2, 0.0)), ('pharmacy', 5.0, 14.0, (10.7, 12.3))]
# The ward bays: (name, x0, x1, the span of their opening onto the corridor).
WARDS = [('ward1', -14.0, -7.0, (-12.1, -10.9)), ('ward2', -7.0, 0.5, (-1.9, -0.7)),
         ('ward3', 0.5, 7.0, (3.4, 4.6)), ('ward4', 7.0, 14.0, (12.0, 13.2))]

# The rooms the robots start in, (x, y, yaw), and the beds they deliver to.
ROOMS = {'hd': (-11.5, -4.3, math.radians(90)),     # the day room, west
         'ht': (-0.6, -4.6, math.radians(90)),      # the treatment room, in the middle
         'hp': (11.5, -5.0, math.radians(90)),      # the pharmacy, east
         'ld': (-3.0, -5.2, math.radians(90))}      # the porters' lodge
BEDS = {1: (-11.5, 6.1 + SHIFT), 2: (-1.3, 6.1 + SHIFT), 3: (3.4, 5.9 + SHIFT), 4: (12.0, 5.9 + SHIFT)}
SPEED = 0.17                                      # m/s, what a drive averages
CRAWL = 0.08                                      # m/s, a bed-mover backing out of a passage


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


def lobby(d):
    """The free space of door d's passage, as a list of one (x0, y0, x1, y1)."""
    h = GAP / 2
    return [(DOOR_X[d] - h, BAND[0], DOOR_X[d] + h, BAND[1])]


def walls():
    """(cx, cy, sx, sy) boxes, WALL_H high: the outside, the rooms' partitions, the
    hall's south wall and the wards' south wall. The band is solid, linen()."""
    x0, y0, x1, y1 = BUILDING
    t = WALL
    out = [_box(x0 - t, y0 - t, x1 + t, y0), _box(x0 - t, y1, x1 + t, y1 + t),
           _box(x0 - t, y0, x0, y1), _box(x1, y0, x1 + t, y1)]
    out += _hwall(HALL_S, x0, x1, cuts=[r[3] for r in SOUTH_ROOMS])
    for _, a, _, _ in SOUTH_ROOMS[1:]:
        out += _vwall(a, y0, HALL_S)
    out += _hwall(WARDS_S, x0, x1, cuts=[w[3] for w in WARDS])
    for _, a, _, _ in WARDS[1:]:
        out += _vwall(a, WARDS_S, y1)
    return out


def linen():
    """The band less its passages, as solid (x0, y0, x1, y1) blocks: linen rooms and sluices."""
    xs = [BUILDING[0]]
    for d in DOORS:
        x0, _, x1, _ = lobby(d)[0]
        xs += [x0, x1]
    xs.append(BUILDING[2])
    return [(a, BAND[0], b, BAND[1]) for a, b in zip(xs[0::2], xs[1::2]) if b - a > 1e-6]


def leaf(d, open_):
    """The leaf of door d: (cx, cy, yaw) for a box LEAF[0] long along its own y and
    LEAF[1] thick. Shut, it stands across the passage; open, it stands along the
    passage's west wall, swung north about its hinge there, its end in the corridor."""
    return leaf_swing(d, 1.0 if open_ else 0.0)


def leaf_swing(d, s):
    """The leaf part way: s = 0 shut, s = 1 open."""
    hx, hy = DOOR_X[d] - GAP / 2 + 0.02, LEAF_Y
    ang = math.radians(90.0) * s            # anticlockwise seen from above: from pointing east to pointing north
    half = LEAF[0] / 2
    cx = hx + half * math.cos(ang) + LEAF[1] / 2 * math.sin(ang)
    cy = hy + half * math.sin(ang)
    return cx, cy, ang - math.radians(90.0)


def floors():
    """(name, x0, y0, x1, y1, material) floor panels, drawn just above the ground."""
    x0, y0, x1, y1 = BUILDING
    out = [(name, a, y0, b, HALL_S, mat) for (name, a, b, _), mat in
           zip(SOUTH_ROOMS, ('wood', 'lino', 'tile', 'tile'))]
    out += [('hall', x0, HALL_S, x1, BAND[0], 'hall'), ('corridor', x0, BAND[1], x1, WARDS_S, 'hall')]
    out += [(name, a, WARDS_S, b, y1, mat) for (name, a, b, _), mat in
            zip(WARDS, ('lino', 'lino', 'lino', 'lino'))]
    return out


def passage_floor():
    """Each passage's floor, (d, x0, y0, x1, y1) panels, tinted so a viewer tells them apart."""
    return [(d, *r) for d in DOORS for r in lobby(d)]


# Hospital models from Gazebo Fuel (Open Robotics, CC BY 4.0) and AWS RoboMaker
# (MIT-0): (model, x, y, yaw). Off every route by tools/check_views.py.
FURNITURE = [
    # ward 1
    ('TrolleyBedPatient', -12.75, 7.0, math.radians(0)), ('BedTable', -11.1, 7.25, math.radians(180)),
    ('IVStand', -13.5, 5.6, 0.0), ('Drawer', -9.1, 7.5, math.radians(-90)),
    ('TrolleyBed', -9.2, 5.55, math.radians(0)),
    # ward 2
    ('TrolleyBedPatient', -6.4, 7.0, 0.0), ('IVStand', -7.0, 5.4, 0.0), ('BedTable', -4.6, 7.3, math.radians(180)),
    ('TrolleyBed', -3.0, 7.0, 0.0), ('Drawer', -2.0, 5.2, math.radians(180)),
    # ward 3
    ('TrolleyBedPatient', 2.6, 7.0, math.radians(180)), ('BedTable', 1.1, 7.3, 0.0),
    ('IVStand', 4.9, 7.4, 0.0), ('PatientWheelChair', -0.7, 7.3, math.radians(90)),
    ('Drawer', 4.9, 5.0, math.radians(180)),
    # ward 4
    ('TrolleyBedPatient', 7.3, 7.0, 0.0), ('TrolleyBedPatient', 12.6, 7.0, math.radians(180)),
    ('IVStand', 6.0, 5.4, 0.0), ('BedTable', 8.9, 7.3, math.radians(180)), ('Drawer', 13.5, 5.2, math.radians(180)),
    # the day room
    ('aws:Sofa_01', -12.6, -6.6, 0.0), ('aws:CoffeeTable_01', -10.4, -6.2, 0.0),
    ('PotatoChipChair', -8.4, -6.3, math.radians(180)), ('aws:SofaB_01', -10.4, -7.45, math.radians(90)),
    ('VendingMachine', -8.0, -3.1, math.radians(180)), ('aws:Trash_01', -13.5, -2.7, 0.0),
    # the porters' lodge
    ('OfficeChairBlack', -5.0, -6.4, 0.0), ('MetalCabinet', -6.6, -7.5, math.radians(90)),
    ('StorageRackCovered', -4.0, -7.55, 0.0), ('MopCart3', -5.6, -3.4, math.radians(0)),
    # the treatment room
    ('XRayMachine', 2.0, -7.1, 0.0), ('InstrumentCart1', -0.8, -7.4, 0.0), ('SurgicalTrolley', 4.3, -5.0, math.radians(90)),
    ('TrolleyBed', 2.0, -4.6, 0.0), ('KitchenSink', -0.5, -7.5, 0.0),
    # the pharmacy
    ('StorageRack', 6.5, -7.5, 0.0), ('StorageRack', 8.0, -7.5, 0.0), ('StorageRackCovered', 13.5, -6.5, math.radians(90)),
    ('MetalCabinet', 13.6, -3.4, math.radians(-90)), ('ParkingTrolleyMin', 7.6, -4.4, 0.0),
    ('BPCart', 9.4, -7.5, 0.0),
    # the hall and the corridor
    ('PatientWheelChair', -13.3, -1.0, math.radians(90)), ('Scrubs', 13.6, -0.6, math.radians(180)),
    ('MopCart3', -13.4, 3.0, math.radians(90)), ('BPCart', 13.5, 3.5, math.radians(180)),
]


FURNITURE = [(m, x, y + SHIFT if y > 2.6 else y, yaw) for m, x, y, yaw in FURNITURE]


def roadmap(robot=None):
    """Named places and the edges the drives route over: (nodes, south, north, cross).

    The hall and the corridor are each a chain of places, one at every passage's
    mouth; the rooms hang off them. The hall's places stand on the robot's own
    line (LANES), or on the hall's middle line for no robot. In each passage m<d>
    is the mark before the door and n<d> the corridor beyond it; v<d> is where the
    robot stands at the mouth (ASIDE) and e<d> the mouth's middle on its line."""
    y = LANES.get(robot, HALL_Y)
    n = {'hd': ROOMS['hd'][:2], 'ht': ROOMS['ht'][:2], 'hp': ROOMS['hp'][:2], 'ld': ROOMS['ld'][:2],
         'sd': (-11.5, y), 'st': (-0.6, y), 'sp': (11.5, y), 'sl': (-3.0, y)}
    for d in DOORS:
        n['v' + d] = (DOOR_X[d] + ASIDE.get(robot, 0.0), y)
        n['e' + d] = (DOOR_X[d], y)
        n['m' + d] = (DOOR_X[d], MARK_Y)
        n['n' + d] = (DOOR_X[d], CORRIDOR_Y)
    n.update({'nw': (-11.5, CORRIDOR_Y), 'n2': (-1.3, CORRIDOR_Y), 'n3': (4.0, CORRIDOR_Y), 'n4': (12.6, CORRIDOR_Y),
              'w1': (-11.5, 4.8 + SHIFT), 'w2': (-1.3, 4.8 + SHIFT), 'w3': (4.0, 4.8 + SHIFT),
              'w4': (12.6, 4.8 + SHIFT)})
    for k, p in BEDS.items():
        n[f'bed{k}'] = p
    south = [('hp', 'sp'), ('sp', 'vd'), ('vd', 'vc'), ('vc', 'st'), ('st', 'sl'), ('sl', 'vb'), ('vb', 'va'),
             ('va', 'sd'), ('st', 'ht'), ('sl', 'ld'), ('sd', 'hd')]
    north = [('nw', 'na'), ('na', 'nb'), ('nb', 'n2'), ('n2', 'nc'), ('nc', 'n3'), ('n3', 'nd'), ('nd', 'n4'),
             ('nw', 'w1'), ('w1', 'bed1'), ('n2', 'w2'), ('w2', 'bed2'), ('n3', 'w3'), ('w3', 'bed3'),
             ('n4', 'w4'), ('w4', 'bed4')]
    # a scout drives from where it stands at a mouth straight to the mark; a bed-mover by the mouth's middle (e<d>)
    legs = (('v', 'e'), ('e', 'm'), ('m', 'n')) if abs(ASIDE.get(robot, 0.0)) > GAP / 2 else (('v', 'm'), ('m', 'n'))
    cross = [(p + d, q + d) for d in DOORS for p, q in legs]
    return n, south, north, cross


def route(edges, a, b, robot=None):
    """The shortest route from place a to place b over the given edges."""
    nodes = roadmap(robot)[0]
    adj = {}
    for u, v in edges:
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


def length(path, robot=None):
    nodes = roadmap(robot)[0]
    return sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))


def trip(path, speed=SPEED, robot=None):
    """The declared duration of a route, in whole seconds, with a few seconds per turn."""
    nodes = roadmap(robot)[0]
    turns = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        u = (nodes[b][0] - nodes[a][0], nodes[b][1] - nodes[a][1])
        v = (nodes[c][0] - nodes[b][0], nodes[c][1] - nodes[b][1])
        ang = abs(math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1]))
        turns += ang / math.radians(90)
    return max(1, round(length(path, robot) / speed + 3.0 * turns))


# The porter's rounds: the two doors he shut.
LOOPS = [('c', 'd'), ('a', 'b'), ('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')]
