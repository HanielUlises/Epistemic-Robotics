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
The coordinated attack's floor: the pass-through floor, read for a different
question.

The hall, the racking block and the storage and dispatch floors are
pass_through_demo's, imported from its layout.py and not restated, so the two
demonstrations stand on one floor. What changes is what the bays are for.

  t1, t3   each holds a load across it, and is a stand: s1 and s2. A load
           can be lifted only from both mouths at once, one robot under each
           end, and a robot at one mouth cannot see one at the other, because
           the load is between them.
  t2       is open, and is the one sight line through the block. The beacon
           stands in it, and its two mouths are the viewpoints: from there,
           and from nowhere else on either floor, a robot sees the beacon.

The robots are south, on the storage floor, and north, on the dispatch floor.
south can read the work order, at a terminal against the west wall. Nothing
in this file is unknown to anyone: the whole floor plan, loads and beacon
included, is common knowledge. What is not is which stand the order names,
and that is not a fact about the floor.
"""

import math
import os


def _pass_through_layout():
    """pass_through_demo's layout.py, loaded under a name of its own: this file
    is also layout.py, and a plain import would find itself."""
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', '..', 'pass_through_demo', 'tools')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory('pass_through_demo'), 'tools'))
    except Exception:   # noqa: BLE001  outside a sourced workspace
        pass
    for path in candidates:
        source = os.path.join(path, 'layout.py')
        if os.path.exists(source):
            spec = importlib.util.spec_from_file_location('pass_through_layout', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError('pass_through_demo/tools/layout.py not found')


P = _pass_through_layout()   # pass_through_demo's floor

# The grid, the hall, the racking and the clutter are the pass-through floor's.
RESOLUTION = P.RESOLUTION
GRID_ORIGIN = P.GRID_ORIGIN
GRID_WIDTH = P.GRID_WIDTH
GRID_HEIGHT = P.GRID_HEIGHT
HALL_MIN_X, HALL_MAX_X = P.HALL_MIN_X, P.HALL_MAX_X
HALL_MIN_Y, HALL_MAX_Y = P.HALL_MIN_Y, P.HALL_MAX_Y
SHELF_MODELS = P.SHELF_MODELS
SHELF_YAW = P.SHELF_YAW
CLUTTER = P.CLUTTER
LOAD_MODEL = P.LOAD_MODEL
BLOCK_HALF_DEPTH = P.BLOCK_HALF_DEPTH

# ─── Stands ─────────────────────────────────────────────────────────────────
#
# EPDDL stand -> pass-through bay. The names are the EPDDL objects and carry no
# underscore, for the reason pass_through_demo gives.

STANDS = {'s1': 't1', 's2': 't3'}


def stand_box(stand):
    return P.tunnel_box(STANDS[stand])


def load_box(stand):
    return P.load_box(STANDS[stand])


def stand_x(stand):
    return P.tunnel_centre(STANDS[stand])[0]


# Which mouth of a stand each robot takes: south the storage side, north the
# dispatch side. A robot never crosses the block.
SIDE = {'south': -1.0, 'north': 1.0}


def mouth(stand, agent):
    """Where a robot waits to lift: on the stand's axis, outside its mouth."""
    return stand_x(stand), SIDE[agent] * (BLOCK_HALF_DEPTH + P.MOUTH_STANDOFF)


# How far from the load's end a robot stops when it drives in under it. The
# laser stops the creep a little short of this anyway; this is the target.
LIFT_GAP = 0.38


def under_end(stand, agent):
    """Where a robot stands to take its end: in the bay, short of the load."""
    x0, y0, x1, y1 = load_box(stand)
    y = y0 - LIFT_GAP - 0.14 if agent == 'south' else y1 + LIFT_GAP + 0.14
    return stand_x(stand), y


def facing(agent):
    """The heading into the block from a robot's side."""
    return math.pi / 2 if agent == 'south' else -math.pi / 2


# How far the load comes up when it is lifted.
LIFT_HEIGHT = 0.12

# ─── The beacon ─────────────────────────────────────────────────────────────
#
# A stack light on a post against the east side of t2, half way through the
# bay. Its lamps are at about two metres, under the racking's top beam, so it
# is seen through the bay and not over the block.

BEACON_BAY = 't2'
BEACON_POST = (P.BAY_WIDTH / 2.0 - 0.25, 0.0)
BEACON_POST_RADIUS = 0.07
BEACON_LAMP_Z = 2.1

# A stack light, one tier per stand: lit, the lower tier says s1 and the
# upper s2. What the beacon announces is which stand, so it needs a symbol for
# each, and two lamps one above the other read at a distance where a colour
# change might not.
BEACON_TIER_Z = {'s1': 1.85, 's2': 2.20}


def beacon_box():
    x, y = BEACON_POST
    r = BEACON_POST_RADIUS + 0.05
    return (x - r, y - r, x + r, y + r)


def viewpoint(agent):
    """Where a robot sees the beacon: outside its mouth of t2, on the axis."""
    x, _ = P.tunnel_centre(BEACON_BAY)
    return x, SIDE[agent] * (BLOCK_HALF_DEPTH + P.MOUTH_STANDOFF)


# ─── The work order ─────────────────────────────────────────────────────────
#
# A terminal against the west wall of the storage floor, in the aisle between
# the rows at y = -18 and y = -12. south reads the order standing in front of
# it, facing west.

TERMINAL = (-14.55, -15.0)
TERMINAL_SIZE = (0.6, 0.9)
TERMINAL_READ = (-13.45, -15.0)
TERMINAL_YAW = math.pi


def terminal_box():
    x, y = TERMINAL
    w, d = TERMINAL_SIZE
    return (x - w / 2.0, y - d / 2.0, x + w / 2.0, y + d / 2.0)


# ─── Robots ─────────────────────────────────────────────────────────────────
#
# Both start facing +x, as pass_through_demo's robots do and for its reason.
# Both carry a twelve-metre laser; neither is a scout here, but the laser is
# what stops a robot short of a load it drives in under.

ROBOTS = {
    # agent: (namespace, x, y, lidar range m, lidar samples, colour)
    'south': ('r1', -12.6, -10.5, 12.0, 720, (0.20, 0.45, 0.80)),
    'north': ('r2', 6.0, 5.6, 12.0, 720, (0.90, 0.50, 0.15)),
}
READS_ORDER = 'south'

# ─── Camera shots ───────────────────────────────────────────────────────────
#
# Gazebo's user camera takes x, y, z, pitch (down positive) and yaw. A model
# name instead of a pose is a chase of that robot.

look_at = P.look_at

OPENING_SHOT = look_at((-4.0, -16.0, 13.0), (0.0, 0.5, 0.0))


def beacon_shot():
    """Low on the axis of t2, behind south's viewpoint, looking through the bay:
    south in the foreground, the beacon on the bay's east side, north at the far
    mouth."""
    return look_at((-1.2, -7.4, 2.3), (0.5, 0.0, 1.3))


def stand_shot(stand):
    """High over the stand's axis, looking down into the bay: the load between
    its two mouths and a robot at each. From lower, the racking either side
    hides the bay, and the load hides the far robot."""
    x = stand_x(stand)
    return look_at((x, -2.8, 11.2), (x, 0.3, 0.0))


def radio_shot():
    """High over the storage floor's west side: south at the terminal low in
    frame, the block that separates it from north across the top. Held while
    a message is on the air."""
    return look_at((-10.5, -23.0, 8.0), (-13.0, -8.0, 0.0))


def terminal_shot():
    """Down the aisle at the terminal, close enough that the robot reads at
    the size of the terminal."""
    return look_at((-11.2, -13.9, 1.5), (-13.7, -15.1, 0.3))


SHOTS = {
    'opening': OPENING_SHOT,
    'radio': radio_shot(),
    'terminal': terminal_shot(),
    'beacon': beacon_shot(),
    's1': stand_shot('s1'),
    's2': stand_shot('s2'),
}


def shot_text(pose):
    return 'pose ' + ' '.join(str(v) for v in pose)


# ─── Everything a robot is told about ──────────────────────────────────────

def static_obstacles(beacon=True):
    """The pass-through floor's obstacles, and what this floor adds: the two
    loads, the terminal and, on the floor that has one, the beacon post. All
    of it common knowledge."""
    boxes = list(P.static_obstacles())
    boxes += [load_box(s) for s in STANDS]
    boxes.append(terminal_box())
    if beacon:
        boxes.append(beacon_box())
    return boxes
