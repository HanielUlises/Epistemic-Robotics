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
The muddy robots' floor: the AWS RoboMaker small warehouse, the world the
RoboticsAcademy Amazon warehouse exercise uses, unchanged, with a muster
point, a calibration bay and a public address added on its open floor.

The floor plan is warehouse_scenario's, rasterised from the world's own
collision meshes at 5 cm, so the racks a robot drives round are the racks it
collides with. Coordinates are the world's.

  muster     the largest open circle on the floor, 3.6 m of clearance about
             (-1.15, -2.35). The robots stand on a ring of 1.2 m about its
             centre, facing in, so that each sees every other robot's lamp.
  bay        the calibration bay, in the room at the north-west corner,
             reached up the west corridor.
  stations   where each robot works, and where it starts and returns to:
             the ends of the west and the middle corridors.
  pa         the public address: a mast at the muster point's edge.

Every robot carries a status lamp on its mast, lit when its calibration has
drifted. Its own camera looks forward and out; the lamp is above it and
behind its laser, and it cannot see it.
"""

import math
import os

# ─── The floor plan ─────────────────────────────────────────────────────────

RESOLUTION = 0.05
GRID_ORIGIN = (-7.0, -10.5)


def floorplan_yaml():
    """warehouse_scenario's floor plan, installed or in the source tree."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', '..', 'warehouse_scenario', 'maps',
                               'aws_small_warehouse.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory('warehouse_scenario'),
                                          'maps', 'aws_small_warehouse.yaml'))
    except Exception:   # noqa: BLE001  outside a sourced workspace
        pass
    for path in candidates:
        if os.path.exists(path):
            return os.path.abspath(path)
    raise RuntimeError('warehouse_scenario/maps/aws_small_warehouse.yaml not found')


# ─── The muster point ───────────────────────────────────────────────────────

MUSTER = (-1.15, -2.35)
RING = 1.2


def muster_spot(index, n):
    """The index-th of n places on the ring, the first at the north-west and
    the rest clockwise, and the heading that faces the centre."""
    angle = math.radians(135.0 - 360.0 * index / n)
    x = MUSTER[0] + RING * math.cos(angle)
    y = MUSTER[1] + RING * math.sin(angle)
    return x, y, math.atan2(MUSTER[1] - y, MUSTER[0] - x)


# The public address: a mast just off the ring, to the south-east.
PA = (MUSTER[0] + 2.3, MUSTER[1] - 1.6)

# ─── The calibration bay ────────────────────────────────────────────────────
#
# Five places side by side along the north-west room, facing north.

BAY = [(-5.8, 9.1), (-5.1, 9.1), (-4.4, 9.1), (-3.7, 9.1), (-3.0, 9.1)]
BAY_YAW = math.pi / 2
BAY_PAD = (-4.4, 9.1, 3.4, 0.9)   # centre x, y, length, depth


def bay_spot(index):
    return BAY[index][0], BAY[index][1], BAY_YAW


# ─── The robots ─────────────────────────────────────────────────────────────
#
# Agent names are the EPDDL agents. Every robot starts facing +x at its
# station, as the other demonstrations' robots do and for their reason: a
# spawn yaw other than zero turns the odometry frame against the world.

FLEET = {
    # agent: (namespace, station x, station y, colour)
    'r1': ('r1', -3.5, 5.5, (0.20, 0.45, 0.80)),
    'r2': ('r2', 1.0, 5.5, (0.90, 0.50, 0.15)),
    'r3': ('r3', 1.0, -8.0, (0.18, 0.60, 0.35)),
    'r4': ('r4', -3.5, -8.0, (0.55, 0.30, 0.70)),
    'r5': ('r5', 1.0, 1.0, (0.20, 0.65, 0.70)),
}


def robots(n=4):
    if not 2 <= n <= len(FLEET):
        raise ValueError(f'the floor has stations for 2 to {len(FLEET)} robots, not {n}')
    return {a: FLEET[a] for a in list(FLEET)[:n]}


# The lamp: lit red when the robot is faulty, dark grey otherwise.
LAMP_LIT = (1.0, 0.08, 0.05)
LAMP_DARK = (0.18, 0.18, 0.18)
LAMP_HEIGHT = 0.42

# ─── Camera shots ───────────────────────────────────────────────────────────


def look_at(eye, target):
    """x, y, z, pitch (down positive), yaw for Gazebo's user camera."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return (round(eye[0], 2), round(eye[1], 2), round(eye[2], 2),
            round(pitch, 3), round(yaw, 3))


OPENING_SHOT = look_at((6.0, -9.5, 8.5), (-1.2, -1.0, 0.0))


def muster_shot():
    """Low over the ring from the south-east, past the public address: the
    four robots facing each other, their lamps against the racks."""
    return look_at((MUSTER[0] + 3.4, MUSTER[1] - 3.6, 2.4), (MUSTER[0], MUSTER[1], 0.3))


def overhead_shot():
    """Nearly straight down on the ring, for the bells: who moves is the
    whole of what is announced, and the four lamps are in one frame."""
    return look_at((MUSTER[0] + 0.01, MUSTER[1] - 0.9, 5.2), (MUSTER[0], MUSTER[1], 0.0))


def pa_shot():
    """Past the public address onto the ring: the mast to the right with its
    lamp, lit as the supervisor speaks, in the frame, and the four robots
    beyond it. Nearer the mast, the lamp is above the frame."""
    return look_at((PA[0] + 1.5, PA[1] - 3.6, 2.9), (MUSTER[0], MUSTER[1], 1.1))


def bay_shot():
    """Down on the bay from high over the strip north of the west shelf,
    looking north. From anywhere south of the shelf's end, its 6.5 m hide
    the bay's west places; from the east, the clutter does."""
    return look_at((-4.4, 8.25, 3.4), (-4.4, 9.55, 0.0))


def hall_shot():
    """The whole floor from the south-east corner, high."""
    return look_at((6.5, -11.0, 11.0), (-1.0, 0.5, 0.0))


SHOTS = {
    'opening': OPENING_SHOT,
    'muster': muster_shot(),
    'overhead': overhead_shot(),
    'pa': pa_shot(),
    'bay': bay_shot(),
    'hall': hall_shot(),
}


def shot_text(pose):
    return 'pose ' + ' '.join(str(v) for v in pose)
