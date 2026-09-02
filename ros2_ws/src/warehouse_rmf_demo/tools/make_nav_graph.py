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
Writes the Open-RMF navigation graph for the AWS small warehouse.

Open-RMF normally gets its graph from a traffic-editor `.building.yaml`, which
also generates the Gazebo world. Here the world already exists and is not ours
-- it is AWS RoboMaker's small warehouse, the same one `scenarios/warehouse`
rasterises the occupancy grid from -- so the graph is written directly in that
world's own metres instead.

The coordinates are not invented. They are the constants in
`warehouse_scenario/include/warehouse_scenario/warehouse.hpp`, which is where
the rest of this repository reads the floor plan from, so the zones RMF routes
between and the zones the mu-calculus planner routes over are the same places.

    make_nav_graph.py --out maps/nav_graphs/0.yaml
"""

import argparse
import yaml

# warehouse.hpp, verbatim. The bays are the centres of kBayAisle2 and
# kBayAisle3; the docks and the corridor are the robots' start poses.
DOCK_SOUTH = (-3.50, -9.30)     # kR1Start
CORRIDOR = (-3.50, -1.30)       # kR3Start
DOCK_NORTH = (-3.50, 6.20)      # kR2Start
LANE = (2.20, -3.04)            # kServiceLaneX, between the two aisle mouths
BAY2 = (4.30, -2.14)            # centre of kBayAisle2
BAY3 = (4.30, -3.94)            # centre of kBayAisle3

WAYPOINTS = [
    ('dock_south', DOCK_SOUTH),
    ('corridor', CORRIDOR),
    ('dock_north', DOCK_NORTH),
    ('lane', LANE),
    ('bay2', BAY2),
    ('bay3', BAY3),
    # Corners, so that a lane never cuts through the rack block. dock_south is
    # already the south end of the west corridor, so the run east along the
    # south wall starts there.
    ('south_east', (2.20, -9.30)),
    ('lane_south', (2.20, -6.20)),
    ('bay2_mouth', (2.20, -2.14)),
    ('bay3_mouth', (2.20, -3.94)),
]

# The floor plan of the building, as edges. The rack block fills the east half,
# so nothing crosses it: an aisle is a pocket with one mouth on the service
# lane, and a robot that has looked into one comes back out to try the other.
EDGES = [
    ('dock_south', 'corridor'),
    ('corridor', 'dock_north'),
    ('dock_south', 'south_east'),
    ('south_east', 'lane_south'),
    ('lane_south', 'lane'),
    ('lane', 'bay2_mouth'),
    ('bay2_mouth', 'bay2'),
    ('lane', 'bay3_mouth'),
    ('bay3_mouth', 'bay3'),
]

# Where each robot parks. RMF wants a charger for every robot in the fleet.
CHARGERS = {
    'r1_charger': (-3.50, -9.90),
    'r2_charger': (-3.50, 6.80),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    named = list(WAYPOINTS) + list(CHARGERS.items())
    index = {name: i for i, (name, _) in enumerate(named)}

    vertices = []
    for name, (x, y) in named:
        properties = {'name': name}
        if name in CHARGERS:
            properties['is_charger'] = True
            properties['is_parking_spot'] = True
        vertices.append([float(x), float(y), properties])

    lanes = []
    edges = list(EDGES) + [
        ('dock_south', 'r1_charger'),
        ('dock_north', 'r2_charger'),
    ]
    for a, b in edges:
        # Both ways: nothing in this building is one-directional.
        lanes.append([index[a], index[b], {'speed_limit': 0}])
        lanes.append([index[b], index[a], {'speed_limit': 0}])

    graph = {
        'building_name': 'aws_small_warehouse',
        'levels': {'L1': {'vertices': vertices, 'lanes': lanes}},
    }

    with open(args.out, 'w') as handle:
        yaml.safe_dump(graph, handle, default_flow_style=None, sort_keys=False)

    print(f'{len(vertices)} waypoints, {len(lanes)} lanes -> {args.out}')


if __name__ == '__main__':
    main()
