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
Reads a Gazebo world and says where its floor is and what stands on it.

The floor here is not ours and was not generated: it is the
`dynamic_logistics_warehouse` world, which is AWS RoboMaker's small warehouse
tiled three by three and furnished by hand. Nothing about its layout is
parametric, so the navigation graph cannot be written from a formula the way a
generated floor's can. It has to be read off the world.

What makes that tractable is that the world places models rather than raw
geometry, and every model is one of fourteen AWS assets whose collision extents
are known. `config/aws_footprints.json` holds those extents, measured from the
collision meshes themselves rather than taken from any description.

Three of the fourteen are structure and not obstacle, and getting this wrong is
the difference between a floor and a solid block:

  * `GroundB` is a floor tile, 14 by 21 metres and 12 cm thick. Nine of them
    make the building. Treated as an obstacle it fills the entire warehouse;
  * `RoofB` is overhead and a robot drives under it;
  * `WallB` is a room-sized shell whose interior is open, so its bounding box
    is not its geometry. This world happens to place none.
"""

import json
import math
import os
import re
import xml.etree.ElementTree as ET

# Structure, not obstacle. See the module docstring.
STRUCTURE = (
    'aws_robomaker_warehouse_GroundB',
    'aws_robomaker_warehouse_RoofB',
    'aws_robomaker_warehouse_WallB',
)

# Overhead: hangs above a robot rather than blocking it.
OVERHEAD = ('aws_robomaker_warehouse_Lamp',)


def base_name(model_name):
    """`aws_robomaker_warehouse_ShelfD_01_007` -> `aws_robomaker_warehouse_ShelfD`."""
    stem = model_name
    while stem and (stem[-1].isdigit() or stem[-1] == '_'):
        stem = stem[:-1]
    return stem


def footprints(path):
    with open(path) as fh:
        raw = json.load(fh)
    return {base_name(k): (v[0], v[1], v[2]) for k, v in raw.items()}


# A pose is six numbers, and in this world they are not always six tokens.
# Two of the entries in the world's `<state>` block read
# `13.8354 41.7933-0.191335 ...`: the separator between a value and the
# negative one after it is missing. Splitting on whitespace yields a token that
# is not a number, and the obvious `try: float(...) except: 0, 0` then places a
# fourteen-by-twenty-one-metre floor tile at the origin without a word. Reading
# the numbers out rather than the tokens is the only version of this that
# cannot fail quietly.
NUMBER = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')


def pose_of(element):
    """(x, y, yaw) from an element's `pose`, zeros when it has none."""
    values = [float(v) for v in NUMBER.findall(element.findtext('pose') or '')]
    if len(values) < 2:
        return 0.0, 0.0, 0.0
    return values[0], values[1], values[5] if len(values) > 5 else 0.0


def recorded_state(world):
    """Where the world's `<state>` block says each model actually is.

    A saved `<state>` overrides the poses declared on the models themselves,
    and Gazebo applies it at load. This world has one, recording 178 poses, and
    fourteen of them disagree with the declaration -- by 0.19 m for one floor
    tile, by 4 m for another, by 49 m for a third, and by 149 m for a cluttering
    prop that the declaration puts inside the building and the state puts well
    outside it.

    Reading the declaration alone therefore produces a floor plan of a
    warehouse that is not the one the simulator loads: obstacles mapped where
    the floor is clear, clear floor where an obstacle stands, and two of the
    nine slabs in the wrong place. Every symptom of that is a robot behaving
    correctly in a world the planner has the wrong map of, which is the
    hardest kind of fault to attribute.
    """
    state = world.find('state')
    if state is None:
        return {}
    return {model.get('name'): pose_of(model)
            for model in state.findall('model')}


def placements(element, ox=0.0, oy=0.0, state=None):
    """Every model at any depth, with its pose resolved against its parents'.

    Models nest. This world has a `<model name="Untitled">` that is not an
    asset at all but a group, holding a bucket and a desk placed relative to
    it, and a scan of the world's direct children sees only the group -- whose
    name is in no footprint table, so both objects are dropped and the floor
    they stand on is rasterised as free. Two obstacles disappearing quietly is
    the same defect as the pallet jack that stopped a robot dead while RMF
    reported its task underway, and it is quiet in the same way: the graph is
    produced, it is valid, and it routes through furniture.

    Only x, y and yaw are carried down. This world nests one level and places
    everything upright, so a full transform would be machinery for a case that
    does not arise; a rotated group would need one, and would be wrong here.
    """
    out = []
    for model in element.findall('model'):
        name = model.get('name') or ''
        # What the state records wins, because it is what Gazebo loads.
        if state and name in state:
            x, y, yaw = state[name]
            ax, ay = x, y          # a state pose is already absolute
        else:
            x, y, yaw = pose_of(model)
            ax, ay = ox + x, oy + y
        out.append((name, ax, ay, yaw))
        out.extend(placements(model, ax, ay, state))
    return out


def read(world_path, footprint_path):
    """Return (floor_tiles, obstacles), both lists of (min_x, min_y, max_x, max_y)."""
    sizes = footprints(footprint_path)
    root = ET.parse(world_path).getroot()
    world = root.find('world')

    floor, obstacles = [], []
    for name, x, y, yaw in placements(world, state=recorded_state(world)):
        base = base_name(name)
        if base not in sizes:
            continue

        dx, dy, _ = sizes[base]
        # Only right angles occur in this world, so a quarter turn swaps the
        # extents and anything else is treated as unturned.
        if abs(abs(yaw) % math.pi - math.pi / 2) < 0.35:
            dx, dy = dy, dx
        box = (x - dx / 2, y - dy / 2, x + dx / 2, y + dy / 2)

        if base in STRUCTURE:
            if base == 'aws_robomaker_warehouse_GroundB':
                floor.append(box)
            continue
        if base in OVERHEAD:
            continue
        obstacles.append(box)

    return floor, obstacles


def actors(world_path):
    """The walking people. They are not obstacles to plan around -- they move --
    but the graph should not be threaded through their patrol lines either."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    out = []
    for actor in world.findall('actor'):
        pts = []
        for wp in actor.findall('.//waypoint/pose'):
            v = (wp.text or '').split()
            if len(v) >= 2:
                pts.append((float(v[0]), float(v[1])))
        out.append((actor.get('name') or '?', pts))
    return out


if __name__ == '__main__':
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    world = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        here, '..', '..', '..', 'third_party',
        'dynamic_logistics_warehouse', 'worlds', 'warehouse.world')
    fp = os.path.join(here, '..', 'config', 'aws_footprints.json')
    floor, obs = read(world, fp)
    xs = [b[0] for b in floor] + [b[2] for b in floor]
    ys = [b[1] for b in floor] + [b[3] for b in floor]
    print(f'{len(floor)} floor tiles spanning '
          f'x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}] '
          f'= {max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} m')
    print(f'{len(obs)} obstacles')
    for name, pts in actors(world):
        print(f'  actor {name}: {len(pts)} waypoints')
