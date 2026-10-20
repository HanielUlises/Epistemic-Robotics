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
The hall of the link grades, in one place.

A hall 14 by 8 metres. Along its north side runs a corridor, a dead end 5 m
long and 1 m wide whose mouth opens west; debris may stand in its middle. Only
a robot at the mouth can decide the corridor with its laser: from anywhere
else the corridor's walls are in the way, and a ray down the corridor ends on
its end wall, so that a corridor without debris is seen to be empty and not
merely not seen.

r1 starts at the west dock, about 6 m from the mouth; r2 at the east station,
about 13 m from it. Two spots at the mouth, one for each robot, side by side,
so that neither stands in the other's view.

Coordinates are metres in the world frame, which is also every robot's
odometry frame. tools/check_views.py casts the rays and checks all of this.
"""

import math

HALL = (-7.0, -4.0, 7.0, 4.0)
WALL = 0.1
CORRIDOR = (-3.0, 1.6, 2.0, 2.6)          # inner faces: x0 (the mouth), y0, x1 (the end wall), y1
DEBRIS = (-0.5, 2.1, 0.5)                 # centre and side of the debris box
REGION = (-1.0, 1.7, 0.0, 2.5)            # what a robot's map must decide
DOCK = (-5.5, -2.8, math.radians(90))     # r1
STATION = (5.8, -2.8, math.radians(180))  # r2
SPOT = {'r1': (-3.75, 2.3), 'r2': (-3.75, 1.85)}
SPEED = 0.16                              # m/s, what the drive averages over a route, for declared durations


def walls():
    """(cx, cy, sx, sy) boxes."""
    x0, y0, x1, y1 = HALL
    t = WALL
    out = []

    def box(xa, ya, xb, yb):
        out.append(((xa + xb) / 2, (ya + yb) / 2, abs(xb - xa), abs(yb - ya)))
    box(x0 - t, y0 - t, x1 + t, y0)
    box(x0 - t, y1, x1 + t, y1 + t)
    box(x0 - t, y0, x0, y1)
    box(x1, y0, x1 + t, y1)
    cx0, cy0, cx1, cy1 = CORRIDOR
    box(cx0, cy0 - t, cx1 + t, cy0)           # south wall
    box(cx0, cy1, cx1, cy1 + t)               # north wall
    box(cx1, cy0 - t, cx1 + t, y1)            # end wall, up to the hall's north wall
    box(cx0 - t, cy1, cx0, y1)                # closing the space above the corridor
    return out


def debris_box():
    cx, cy, s = DEBRIS
    return (cx - s / 2, cy - s / 2, cx + s / 2, cy + s / 2)


def roadmap():
    """Named places, and straight segments clear of every wall."""
    n = {'dock': DOCK[:2], 'station': STATION[:2], 'spot_r1': SPOT['r1'], 'spot_r2': SPOT['r2'],
         'w_s': (-4.6, -1.5), 'w_n1': (-4.6, SPOT['r1'][1]), 'w_m': (-4.2, 0.3), 'w_n2': (-4.6, SPOT['r2'][1]),
         'e_s': (4.5, -1.5)}
    e = [('dock', 'w_s'), ('w_s', 'w_n1'), ('w_n1', 'spot_r1'),
         ('station', 'e_s'), ('e_s', 'w_m'), ('w_m', 'w_n2'), ('w_n2', 'spot_r2')]
    return n, e


ROUTE = {'r1': ['dock', 'w_s', 'w_n1', 'spot_r1'], 'r2': ['station', 'e_s', 'w_m', 'w_n2', 'spot_r2']}


def length(path):
    nodes, _ = roadmap()
    return sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))


def trip(r):
    """The declared duration of r's drive to its spot, in whole seconds."""
    return round(length(ROUTE[r]) / SPEED)
