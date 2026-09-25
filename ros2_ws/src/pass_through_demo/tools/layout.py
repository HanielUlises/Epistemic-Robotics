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
The pass-through floor: one definition, from which everything else is written.

The world SDF, the floor plan the robots are given, the tunnel regions that
perception reads, the robots' start poses and the dock are all derived from
the constants here. Nothing downstream holds a coordinate of its own.

The hall is `OpenRobotics/Warehouse`, the 30 by 50 metre building the larger
warehouse demo already uses, measured from its collision mesh: the inner wall
faces are at x = +/-14.95 and y = +/-24.95, and ten pillars half a metre square
stand at x = +/-7.43, y in {-15, -7.5, 0, 7.5, 15}.

What is placed in it has one purpose. A racking block crosses the hall from
wall to wall, and the only ways from the storage floor in the south to the
dispatch floor in the north are three pass-through bays in that block: bays
left without beams, the width of one shelf unit, through which a robot can
drive. Exactly one of the three is open. The other two hold staged loads.
Which one is open is the one thing about this building the robots do not know,
and it is not something a robot can see from anywhere: a bay is 3.5 m deep and
2.6 m wide, so reading it takes a laser standing near its axis, at its mouth.

The rest of the building is common knowledge, and is given to every robot as
its floor plan. The three bays are the exception. Their interiors are left out
of the floor plan, as unknown, because their state is exactly what the plan
has to find out.
"""

import math

# ─── The hall ───────────────────────────────────────────────────────────────

HALL_MIN_X, HALL_MAX_X = -14.95, 14.95
HALL_MIN_Y, HALL_MAX_Y = -24.95, 24.95
WALL_THICKNESS = 0.05

PILLAR_X = (-7.43, 7.43)
PILLAR_Y = (-15.0, -7.5, 0.0, 7.5, 15.0)
PILLAR_SIDE = 0.5

# ─── Shelving ───────────────────────────────────────────────────────────────
#
# AWS RoboMaker's ShelfD_01 and ShelfE_01: identical collision, measured from
# the collision mesh with its node transform and its centimetre unit applied.
# Unturned, a unit is 3.918 m along x, 0.88 m along y and 2.64 m tall, and its
# collision is two solid boxes, so a laser at robot height sees a unit as a
# wall. Placed at yaw 0, a row of them runs east-west.
#
# Not the 0.88 x 2.613 x 3.918 of warehouse_xl_rmf_demo's aws_footprints.json,
# which permutes the axes. The first floor built on those figures turned every
# unit a quarter turn from where the floor plan put it: each "row" was a set
# of north-south columns with 1.7 m gaps between them, the racking block was
# not a wall, and the carrier stopped against the south end of a column the
# floor plan said was not there.

SHELF_LENGTH = 3.918
SHELF_DEPTH = 0.88
SHELF_YAW = 0.0
SHELF_MODELS = ('aws_robomaker_warehouse_ShelfD_01',
                'aws_robomaker_warehouse_ShelfE_01')

# ─── The racking block ──────────────────────────────────────────────────────
#
# Four rows back to back, wall to wall, with three pass-through bays 2.6 m
# wide: one on the hall's axis and one either side of it, two units out. The
# outermost units run 0.7 m into the walls, which closes the block against
# them; the pillars on the y = 0 line stand inside the second unit either
# side, as a building's columns stand inside real racking.

BAY_WIDTH = 2.6
BLOCK_ROWS = 4
BLOCK_HALF_DEPTH = BLOCK_ROWS * SHELF_DEPTH / 2.0  # 1.76
_SIDE_BAY_X = BAY_WIDTH / 2.0 + 2 * SHELF_LENGTH + BAY_WIDTH / 2.0   # 10.436


def block_rows_y():
    return [-BLOCK_HALF_DEPTH + SHELF_DEPTH * (i + 0.5) for i in range(BLOCK_ROWS)]


# The three pass-through bays, west to east, by the x of their axes. The names
# are the EPDDL objects and carry no underscore: grounded action names are
# joined with one, and the executor splits on it to recover arguments.
TUNNEL_BAYS = {'t1': -_SIDE_BAY_X, 't2': 0.0, 't3': _SIDE_BAY_X}


def tunnel_centre(name):
    return TUNNEL_BAYS[name], 0.0


def tunnel_box(name):
    """The bay's clear interior, wall to wall and end to end."""
    x, _ = tunnel_centre(name)
    return (x - BAY_WIDTH / 2.0, -BLOCK_HALF_DEPTH, x + BAY_WIDTH / 2.0, BLOCK_HALF_DEPTH)


def block_unit_x():
    """The x of every unit's centre along a row of the block."""
    half = BAY_WIDTH / 2.0
    inner = [half + SHELF_LENGTH * (k + 0.5) for k in range(2)]      # between t2 and t3
    outer = [_SIDE_BAY_X + half + SHELF_LENGTH / 2.0]               # beyond t3
    east = inner + outer
    return sorted([-x for x in east] + east)


# What perception reads to decide a bay: the corridor through it a robot can
# drive, which is the interior less the navigation's inflation from each side
# wall, and its full depth. A cell inside the inflation band is one no route
# can use, so whether it has been seen says nothing about whether the bay can
# be crossed; and "clear" requires every cell of the region, so a region that
# included the band would make a scout's verdict hang on the few cells a
# laser at the mouth sees only at grazing incidence. A load closes a bay by
# covering the corridor, and a centred load covers all of it.
REGION_MARGIN = 0.45


def tunnel_region(name):
    x0, y0, x1, y1 = tunnel_box(name)
    return (x0 + REGION_MARGIN, y0 + 0.10, x1 - REGION_MARGIN, y1 - 0.10)


# Where a scout stands to read a bay: on the bay's axis, a metre and a half
# short of its southern mouth, facing north into it.
MOUTH_STANDOFF = 1.5


def tunnel_mouth(name):
    x, _ = tunnel_centre(name)
    return x, -BLOCK_HALF_DEPTH - MOUTH_STANDOFF


# Where a scout waits once it has read its bay: two metres along the block,
# on the side away from the central lane, so that it is not standing on the
# carrier's approach when the bay it read is the open one.
PARKING_OFFSET = 2.0


def tunnel_parking(name):
    x, y = tunnel_mouth(name)
    return x + (PARKING_OFFSET if x >= 0 else -PARKING_OFFSET), y


# ─── The storage floor, south of the block ──────────────────────────────────
#
# Four rows of three units each side of a 3 m central lane, clear of the
# pillar lines at y = -7.5 and y = -15. The rows are what make the storage
# floor a place with sight lines: from inside it a robot sees along its own
# aisle, and not across to the block. The lane is 3 m and not 4 so that the
# aisles' outer ends leave 1.7 m to the wall: at 1.2 m a scout leaving its
# aisle there cut the corner of the row end, and its laser stopped it against
# the rack.

STORAGE_ROWS_Y = (-21.0, -18.0, -12.0, -9.0)
LANE_HALF_WIDTH = 1.5
STORAGE_UNITS_X = tuple(-(LANE_HALF_WIDTH + SHELF_LENGTH * (k + 0.5)) for k in range(3))

# Two more rows each side on the dispatch floor, either side of the dock lane.
DISPATCH_ROWS_Y = (18.5, 21.5)


def storage_shelves():
    out = []
    for y in STORAGE_ROWS_Y + DISPATCH_ROWS_Y:
        for x in STORAGE_UNITS_X:
            for side in (-1.0, 1.0):
                out.append((side * x, y))
    return out


# ─── The dispatch floor, north of the block ─────────────────────────────────
#
# Staged goods and equipment, placed so that each bay's northern mouth leads
# to the dock by a different way. AWS clutter models, with the collision
# extents measured from their meshes.

CLUTTER_FOOTPRINT = {
    'aws_robomaker_warehouse_ClutteringA_01': (2.161, 2.002),
    'aws_robomaker_warehouse_ClutteringC_01': (1.773, 2.060),
    'aws_robomaker_warehouse_ClutteringD_01': (1.017, 1.492),
    'aws_robomaker_warehouse_PalletJackB_01': (1.161, 0.540),
    'aws_robomaker_warehouse_Bucket_01': (0.941, 1.222),
    'aws_robomaker_warehouse_TrashCanC_01': (1.479, 0.909),
}

CLUTTER = (
    ('aws_robomaker_warehouse_ClutteringA_01', -10.4, 8.8, 0.0),
    ('aws_robomaker_warehouse_ClutteringC_01', -4.6, 11.8, 0.0),
    ('aws_robomaker_warehouse_ClutteringA_01', 4.6, 11.8, 0.0),
    ('aws_robomaker_warehouse_ClutteringC_01', 10.4, 8.8, 0.0),
    ('aws_robomaker_warehouse_ClutteringD_01', -12.9, 13.2, 0.0),
    ('aws_robomaker_warehouse_ClutteringD_01', 12.9, 13.2, 0.0),
    ('aws_robomaker_warehouse_PalletJackB_01', -9.0, 16.4, 0.0),
    ('aws_robomaker_warehouse_PalletJackB_01', 9.0, 16.4, 0.0),
    ('aws_robomaker_warehouse_Bucket_01', -13.9, -23.9, 0.0),
    ('aws_robomaker_warehouse_TrashCanC_01', 13.8, -24.2, 0.0),
)

# The load that closes a bay: a pallet of boxes, 2.16 m across in a bay 2.61 m
# wide, which leaves 0.23 m each side and no way through for a base 0.28 m
# wide. It stands in the middle of the bay, so that a laser at either mouth
# reads it.
LOAD_MODEL = 'aws_robomaker_warehouse_ClutteringA_01'


def load_box(name):
    x, y = tunnel_centre(name)
    w, d = CLUTTER_FOOTPRINT[LOAD_MODEL]
    return (x - w / 2.0, y - d / 2.0, x + w / 2.0, y + d / 2.0)


# ─── Robots and the dock ────────────────────────────────────────────────────
#
# Agent names are the EPDDL agents. Every robot starts facing +x, because a
# spawn yaw other than zero turns its odometry frame against the world and
# every region written in world coordinates would then be read rotated.

ROBOTS = {
    # agent: (namespace, x, y, lidar range m, lidar samples, colour)
    'west': ('r1', -12.6, -10.5, 12.0, 720, (0.20, 0.45, 0.80)),
    'east': ('r2', 12.6, -10.5, 12.0, 720, (0.90, 0.50, 0.15)),
    'carrier': ('r3', 0.0, -22.6, 3.5, 360, (0.18, 0.60, 0.35)),
}

# Which bay each scout is stationed at. The carrier is stationed at none: it is
# hauling, and a hauler is not sent to survey. The centre bay belongs to no
# one, because it opens onto the carrier's own lane.
COVERS = {'west': ('t1',), 'east': ('t3',), 'carrier': ()}

DOCK = (0.0, 21.0)
DOCK_RADIUS = 0.6

# ─── Camera shots for the recording ─────────────────────────────────────────
#
# A fixed shot down each scout's bay, from the south, on the bay's axis and
# above the 3.9 m racking, with the load in frame and the scout arriving at the
# mouth; and a wide shot of the whole block to open on, under the hall's
# 12.6 m roof. Gazebo's user camera takes a pose as x, y, z, pitch (down is
# positive) and yaw.


def look_at(eye, target):
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return (eye[0], eye[1], eye[2], round(pitch, 3), round(yaw, 3))


def mouth_shot(name):
    x, _ = tunnel_centre(name)
    return look_at((x, -9.6, 6.2), (x, -1.4, 0.3))


OPENING_SHOT = look_at((0.0, -19.0, 11.5), (0.0, 1.5, 0.0))


# ─── The grid everything is expressed on ────────────────────────────────────

RESOLUTION = 0.10
GRID_ORIGIN = (-15.5, -25.5)
GRID_WIDTH = 310      # cells, x
GRID_HEIGHT = 510     # cells, y


# ─── Boxes ──────────────────────────────────────────────────────────────────

def footprint(model, x, y, yaw):
    w, d = CLUTTER_FOOTPRINT[model]
    if abs(abs(math.fmod(yaw, math.pi)) - math.pi / 2) < 1e-3:
        w, d = d, w
    return (x - w / 2.0, y - d / 2.0, x + w / 2.0, y + d / 2.0)


def block_shelves():
    """Every unit of the racking block as (x, y)."""
    return [(x, y) for y in block_rows_y() for x in block_unit_x()]


def end_frames():
    """Boxes closing the block against the walls. None: the outer units reach
    into the walls themselves."""
    return []


def shelf_box(x, y):
    return (x - SHELF_LENGTH / 2.0, y - SHELF_DEPTH / 2.0,
            x + SHELF_LENGTH / 2.0, y + SHELF_DEPTH / 2.0)


def pillar_boxes():
    h = PILLAR_SIDE / 2.0
    return [(px - h, py - h, px + h, py + h) for px in PILLAR_X for py in PILLAR_Y]


def static_obstacles():
    """Everything a robot is told about: racking, pillars, end frames, clutter.

    Not the loads in the bays. Those are the state of the world the plan has to
    find out, and a floor plan that contained them would be a floor plan that
    already knew the answer.
    """
    boxes = [shelf_box(x, y) for x, y in block_shelves()]
    boxes += [shelf_box(x, y) for x, y in storage_shelves()]
    boxes += pillar_boxes()
    boxes += end_frames()
    boxes += [footprint(m, x, y, yaw) for m, x, y, yaw in CLUTTER]
    return boxes
