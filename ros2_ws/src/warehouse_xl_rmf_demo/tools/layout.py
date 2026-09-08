#!/usr/bin/env python3
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
One definition of the floor, from which the world and the graph are both built.

The small warehouse demo reads its zone coordinates from `warehouse.hpp`, so
that the map writer and the planner name the same places. This is the same
arrangement for the larger floor, in the language the generators are written
in: the world SDF, the navigation graph and the building map are all produced
from the constants here, and none of them is edited by hand.

The building is `OpenRobotics/Warehouse` from Gazebo Fuel, measured from its
own collision mesh rather than from its description:

  * the hall is 30 m by 50 m, x in [-15, 15] and y in [-25, 25];
  * ten pillars, half a metre square, stand at x = +/-7.43 and
    y in {-15, -7.5, 0, 7.5, 15};
  * nothing else in it collides with anything. The shelving is ours.

That last point is why this file exists. The hall is an empty box, and an
empty box has no aisles; an aisle a robot can only look down from its mouth is
the whole reason the epistemic domain has anything to be uncertain about. So
the racks are placed here, and placed clear of the pillars.
"""

# ─── The hall, from the collision mesh of the Fuel model ────────────────────

HALL_MIN_X, HALL_MAX_X = -15.0, 15.0
HALL_MIN_Y, HALL_MAX_Y = -25.0, 25.0

PILLAR_X = (-7.43, 7.43)
PILLAR_Y = (-15.0, -7.5, 0.0, 7.5, 15.0)
PILLAR_SIDE = 0.5

# ─── The service lane ───────────────────────────────────────────────────────
#
# One north-south corridor down the middle. Every aisle opens onto it and onto
# nothing else, which is what makes an aisle unobservable until a robot stands
# at its mouth.

LANE_HALF_WIDTH = 1.5

# ─── The racks ──────────────────────────────────────────────────────────────
#
# A row is two `ShelfD_01` units laid end to end, each 2.61 m long and 0.88 m
# deep, turned so their length runs east-west. Two units reach from the lane to
# x = 7.1, which clears the pillar face at 7.18 by eight centimetres.

SHELF_LENGTH = 2.61
SHELF_DEPTH = 0.88

RACK_UNIT_OFFSETS = (3.2, 5.8)      # |x| of each unit's centre
RACK_MIN_X = RACK_UNIT_OFFSETS[0] - SHELF_LENGTH / 2.0
RACK_MAX_X = RACK_UNIT_OFFSETS[-1] + SHELF_LENGTH / 2.0

# Row pitch is the shelf depth plus the aisle. 1.62 m of clear aisle takes the
# TinyRobot, whose footprint radius is 0.3, with room to turn at the mouth.
AISLE_WIDTH = 1.62
ROW_PITCH = SHELF_DEPTH + AISLE_WIDTH

ROW_COUNT = 18
ROWS_MIN_Y = -21.0

# ─── The docks ──────────────────────────────────────────────────────────────
#
# The outer bays, beyond the pillar rows, are 7.3 m of clear floor the full
# length of the building. They hold the four docks the domain names, reached
# from the ends of the service lane.

DOCK_X = 11.0
DOCK_Y = (-19.0, 19.0)


def rows():
    """The y centre of each rack row, south to north."""
    return [ROWS_MIN_Y + i * ROW_PITCH for i in range(ROW_COUNT)]


# Where the service lane ends, and so where the cross aisles to the docks run.
#
# Derived rather than chosen. The cross aisle has to clear the outermost rack
# row, and a number written here by hand goes stale the moment ROW_COUNT
# changes -- which is exactly what happened: eighteen rows put the north cross
# aisle straight through the last row of shelving, and the lane check refused
# it. Half the shelf depth, the robot's clearance and a little margin is the
# whole of it.
_RACK_END_MARGIN = SHELF_DEPTH / 2.0 + 0.45 + 0.15

LANE_MIN_Y = ROWS_MIN_Y - _RACK_END_MARGIN
LANE_MAX_Y = ROWS_MIN_Y + (ROW_COUNT - 1) * ROW_PITCH + _RACK_END_MARGIN


def aisles():
    """The y centre of each aisle, and the rows that bound it.

    An aisle lies between two consecutive rows, so `ROW_COUNT` rows give
    `ROW_COUNT - 1` of them on each side of the lane.
    """
    centres = rows()
    return [(0.5 * (centres[i] + centres[i + 1]), i, i + 1)
            for i in range(len(centres) - 1)]


def shelf_poses():
    """Every shelf unit as (x, y, yaw), both blocks."""
    out = []
    for y in rows():
        for side in (-1.0, 1.0):
            for offset in RACK_UNIT_OFFSETS:
                # Turned a quarter turn so the 2.61 m length runs east-west.
                out.append((side * offset, y, 1.5707963))
    return out


def clear_of_pillars(x, y, margin=0.45):
    """True when (x, y) is far enough from every pillar to drive through."""
    half = PILLAR_SIDE / 2.0 + margin
    return not any(abs(x - px) < half and abs(y - py) < half
                   for px in PILLAR_X for py in PILLAR_Y)


def inside_hall(x, y, margin=0.6):
    return (HALL_MIN_X + margin < x < HALL_MAX_X - margin and
            HALL_MIN_Y + margin < y < HALL_MAX_Y - margin)


def in_rack_block(x, y, margin=0.0):
    """True when (x, y) falls inside a shelf row's footprint."""
    if not (RACK_MIN_X - margin <= abs(x) <= RACK_MAX_X + margin):
        return False
    half = SHELF_DEPTH / 2.0 + margin
    return any(abs(y - ry) <= half for ry in rows())
