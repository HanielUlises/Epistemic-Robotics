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
The stale-maps floor: the pass-through floor, with a fleet working on it.

The hall, the racking block with its three bays, and the storage and dispatch
floors are pass_through_demo's, imported from its layout.py and not restated.

At the start of the shift every robot is given the shift map: the floor plan
with the bays as they were at the start, t1 open and a load staged in each of
t2 and t3. Every cell of it is known. During the shift a forklift stages a
load in t1 and takes the load out of t3, and a robot sees a change exactly
when its station has a sight line into the whole of the bay's corridor within
its laser's range. tools/check_floor.py decides that by casting rays over the
floor plan, with the other robots standing where they stand, and requires
every station to see a bay wholly or not at all: a robot that saw part of a
change would hold a map neither the shift map nor the floor.

Eight robots, three of them haulers, which must take a load from the storage
floor to the dispatch floor through a bay their map shows open. Everything
here is common knowledge, the stations included, since the fleet's positions
are. Whether the forklift used its slots is not.
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
BLOCK_HALF_DEPTH = P.BLOCK_HALF_DEPTH
LOAD_MODEL = P.LOAD_MODEL
look_at = P.look_at

# ─── The bays, at the start of the shift and after the forklift ─────────────

BAYS = ('t1', 't2', 't3')
SHIFT_BLOCKED = ('t2', 't3')
CHANGES = {'t1': 'stage', 't3': 'clear'}


def after_forklift():
    out = set(SHIFT_BLOCKED)
    for t, kind in CHANGES.items():
        (out.add if kind == 'stage' else out.discard)(t)
    return sorted(out)


bay_centre = P.tunnel_centre
bay_box = P.tunnel_box
bay_region = P.tunnel_region
load_box = P.load_box


def bay_mouth(bay, side=-1.0):
    """On the bay's axis, outside its mouth on the south (-1) or north (+1)."""
    x, _ = bay_centre(bay)
    return x, side * (BLOCK_HALF_DEPTH + P.MOUTH_STANDOFF)


# ─── The fleet ──────────────────────────────────────────────────────────────
#
# Agent names are the EPDDL agents and the robots' namespaces. Every robot
# spawns facing +x: a spawn yaw other than zero turns its odometry frame
# against the world, and every region here is written in world coordinates.
# Every robot carries the twelve-metre mapping laser; what a robot sees is
# decided by where it stands, not by what it carries.

LIDAR_RANGE = 12.0
LIDAR_SAMPLES = 720

ROBOTS = {
    # agent: (x, y, colour)
    # South of t1, a little off its axis: sees t1, and t3 is 19 m away.
    'r1': (-8.6, -5.4, (0.20, 0.45, 0.80)),
    # A hauler at the west end of the south band, in sight of t1.
    'r2': (-12.7, -5.6, (0.85, 0.25, 0.25)),
    # North of t1, on the dispatch floor: sees t1 through its north mouth.
    'r3': (-12.4, 4.8, (0.45, 0.30, 0.70)),
    # A hauler in the storage lane, between the rows: sees no bay.
    'r4': (0.0, -14.8, (0.90, 0.60, 0.10)),
    # In the west storage aisle: sees no bay.
    'r5': (-5.6, -15.0, (0.40, 0.40, 0.40)),
    # North of t3, east of its axis, off the haulers' way out: sees all of t3
    # through its north mouth. At 12.4 m east it would miss the far corner.
    'r6': (11.8, 5.4, (0.15, 0.60, 0.55)),
    # South of t3, east of its axis, off the haulers' approach: sees t3.
    'r7': (11.8, -5.8, (0.55, 0.70, 0.20)),
    # A hauler in the east storage aisle: sees no bay.
    'r8': (5.6, -15.0, (0.85, 0.40, 0.65)),
}
HAULERS = ('r2', 'r4', 'r8')

# On the secret floor r4 is a contractor's robot: it hauls, and it must end
# the shift not believing that t1 was staged. It sees neither change.
CONTRACTORS = ('r4',)
SECRET = ('t1',)

# Where each hauler sets its load down on the dispatch floor: between the two
# clutter piles north of the block, clear of the dispatch rows.
DROPS = {
    'r2': (-1.7, 15.2),
    'r4': (0.0, 13.6),
    'r8': (1.7, 15.2),
}
DROP_RADIUS = 0.5

ROBOT_RADIUS = 0.30   # a Waffle's footprint, as an occluder of sight lines


def fleet():
    """The fleet as tools/stale_maps.py takes it, with the sight lines
    check_floor.py computes."""
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from check_floor import sight_lines   # noqa: E402
    from stale_maps import Fleet          # noqa: E402
    sees = sight_lines()
    return Fleet(list(ROBOTS), list(BAYS), SHIFT_BLOCKED, CHANGES,
                 {t: [a for a, s in sees.items() if s[t] == 'all'] for t in BAYS},
                 list(HAULERS), CONTRACTORS, SECRET)


def static_obstacles():
    """The floor plan's obstacles. The loads are not among them: they are the
    shift map's, and the shift map is what goes stale."""
    return list(P.static_obstacles())


# ─── Camera shots ───────────────────────────────────────────────────────────

OPENING_SHOT = look_at((0.0, -21.0, 15.0), (0.0, 0.5, 0.0))


def bay_shot(bay):
    """High over the storage floor, down the bay's axis: the forklift's
    change in frame, with the robots that see it."""
    x, _ = bay_centre(bay)
    return look_at((x, -10.5, 7.5), (x, 0.0, 0.0))


DISPATCH_SHOT = look_at((0.0, 4.0, 11.0), (0.0, 13.0, 0.0))

SHOTS = {
    'opening': OPENING_SHOT,
    't1': bay_shot('t1'),
    't3': bay_shot('t3'),
    'dispatch': DISPATCH_SHOT,
}


def shot_text(pose):
    return 'pose ' + ' '.join(str(v) for v in pose)


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])
