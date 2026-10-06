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
The false-belief floor: the pass-through floor, read for a third question.

The hall, the racking block and the storage and dispatch floors are
pass_through_demo's, imported from its layout.py and not restated. What the
bays are for:

  t1, t3   storage bays. The crate starts in t1. Both are open at both
           mouths, so a robot on the storage floor reaches a bay from the
           south and one on the dispatch floor from the north.
  t2       unused.

The robots are the picker, which stored the crate in t1 and works the storage
floor, and the mover, which works the dispatch floor and carries the crate
through the bays' north mouths. The picker's charging dock is against the
west wall of the storage floor, between two rows of racking: from there
neither bay is in sight, which is what makes the picker oblivious to the
relocation. Everything in this file is common knowledge. Where the crate is
after the shift's second action is not, and that is not a fact about the
floor.
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

RESOLUTION = P.RESOLUTION
GRID_ORIGIN = P.GRID_ORIGIN
GRID_WIDTH = P.GRID_WIDTH
GRID_HEIGHT = P.GRID_HEIGHT
HALL_MIN_X, HALL_MAX_X = P.HALL_MIN_X, P.HALL_MAX_X
HALL_MIN_Y, HALL_MAX_Y = P.HALL_MIN_Y, P.HALL_MAX_Y
SHELF_MODELS = P.SHELF_MODELS
SHELF_YAW = P.SHELF_YAW
CLUTTER = P.CLUTTER
BLOCK_HALF_DEPTH = P.BLOCK_HALF_DEPTH

# ─── Bays and the crate ─────────────────────────────────────────────────────

BAYS = ('t1', 't3')
CRATE_START = 't1'
# A tote small enough to ride on a Waffle's top plate.
CRATE_SIZE = (0.45, 0.45, 0.32)
# The crate's centre when it rides on a robot: its bottom clears the laser.
CARRY_Z = 0.42


def bay_centre(bay):
    return P.tunnel_centre(bay)


def crate_rest(bay):
    """Where the crate stands in a bay: on its axis, half way through."""
    x, y = bay_centre(bay)
    return x, y


# The picker reaches a bay from the storage floor, the mover from the dispatch
# floor; neither crosses the block.
SIDE = {'picker': -1.0, 'mover': 1.0}


def mouth(bay, agent):
    """Where a robot stands to look into a bay or drive in: on its axis,
    outside its mouth on the robot's side."""
    x, _ = bay_centre(bay)
    return x, SIDE[agent] * (BLOCK_HALF_DEPTH + P.MOUTH_STANDOFF)


def facing(agent):
    """The heading into the block from a robot's side."""
    return math.pi / 2 if agent == 'picker' else -math.pi / 2


# How close a robot comes to the crate when it takes it, measured from the
# robot's centre to the crate's.
TAKE_GAP = 0.62
# Seen along the bay's axis from its mouth, a crate in the bay reads at about
# the standoff plus the half depth less half the crate; an empty bay reads
# through to the far floor. Anything nearer than this is the crate.
LOOK_THRESHOLD = BLOCK_HALF_DEPTH + P.MOUTH_STANDOFF + 0.4

# ─── The dock and the drop ──────────────────────────────────────────────────
#
# The picker charges against the west wall between the storage rows at y = -18
# and y = -12; from there the racking hides both bays. A fetched crate is set
# down at the drop, on the storage floor in front of the block between t1 and
# t2, where an order is packed.

DOCK = (-13.45, -15.0)
DOCK_YAW = math.pi
DOCK_PAD = (0.9, 1.1)
DROP = (-5.2, -6.2)
DROP_PAD = (1.0, 1.0)

# ─── Robots ─────────────────────────────────────────────────────────────────
#
# The picker starts at the storage mouth of t1, where it has just stored the
# crate; the mover starts on the dispatch floor.

ROBOTS = {
    # agent: (namespace, x, y, lidar range m, lidar samples, colour)
    'picker': ('r1', mouth('t1', 'picker')[0], mouth('t1', 'picker')[1], 12.0, 720,
               (0.20, 0.45, 0.80)),
    'mover': ('r2', 6.0, 5.6, 12.0, 720, (0.90, 0.50, 0.15)),
}
MOVER_REST = (6.0, 5.6)


def static_obstacles():
    """The pass-through floor's obstacles. The crate is not one: it moves, and
    a robot that meets it stops on its laser."""
    return list(P.static_obstacles())


def pad_box(centre, size):
    x, y = centre
    w, d = size
    return (x - w / 2.0, y - d / 2.0, x + w / 2.0, y + d / 2.0)


# ─── Camera shots ───────────────────────────────────────────────────────────

look_at = P.look_at

OPENING_SHOT = look_at((-2.0, -19.0, 14.0), (-1.0, 1.0, 0.0))


def bay_shot(bay):
    """High over the bay from the dispatch side, looking down its axis: the
    crate in the bay, or the bay empty."""
    x, _ = bay_centre(bay)
    return look_at((x + 0.6, 8.6, 8.4), (x, 0.0, 0.0))


def dock_shot():
    """Over the storage rows, the dock low in frame and the block behind the
    racking: the bays are out of the picker's sight."""
    x, y = DOCK
    return look_at((x + 6.0, y - 5.5, 7.0), (x + 1.0, y + 2.0, 0.0))


def drop_shot():
    x, y = DROP
    return look_at((x + 3.2, y - 4.2, 3.6), (x, y + 0.4, 0.2))


SHOTS = {
    'opening': OPENING_SHOT,
    'dock': dock_shot(),
    'drop': drop_shot(),
    't1': bay_shot('t1'),
    't3': bay_shot('t3'),
}


def shot_text(pose):
    return 'pose ' + ' '.join(str(v) for v in pose)
