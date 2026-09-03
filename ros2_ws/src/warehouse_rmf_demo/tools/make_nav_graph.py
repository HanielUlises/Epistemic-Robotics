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

The zone coordinates are not invented. They are the constants in
`warehouse_scenario/include/warehouse_scenario/warehouse.hpp`, which is where
the rest of this repository reads the floor plan from, so the zones RMF routes
between and the zones the mu-calculus planner routes over are the same places.

The lanes between them are another matter, and drawing them by eye does not
work. A first version ran a lane straight east along the south wall, which
looks reasonable on paper and passes through the shelving; the robot drove
into it and stopped, the task never completed, and nothing in RMF said why.
So every waypoint and every lane is now checked against the occupancy grid
`warehouse_scenario` rasterised from the world's own collision meshes, and a
lane that crosses anything is an error here rather than a robot halted in a
corridor an hour later.

    make_nav_graph.py --map ../warehouse_scenario/maps/aws_small_warehouse.yaml \
                      --out maps/nav_graphs/0.yaml
"""

import argparse
import math
import os

import yaml


class Floor:
    """The occupancy grid, for asking whether a robot could be somewhere."""

    def __init__(self, path):
        meta = yaml.safe_load(open(path))
        self.resolution = meta['resolution']
        self.origin_x, self.origin_y = meta['origin'][0], meta['origin'][1]
        image = os.path.join(os.path.dirname(path), meta['image'])
        with open(image, 'rb') as handle:
            if handle.readline().strip() != b'P5':
                raise SystemExit(f'{image} is not a binary PGM')
            line = handle.readline()
            while line.startswith(b'#'):
                line = handle.readline()
            self.width, self.height = map(int, line.split())
            handle.readline()
            self.pixels = handle.read()

    def free(self, x, y, margin):
        """Free, with clearance: a robot has width and RMF wants room to turn."""
        steps = int(margin / self.resolution)
        for dc in range(-steps, steps + 1):
            for dr in range(-steps, steps + 1):
                col = int((x - self.origin_x) / self.resolution) + dc
                row = self.height - 1 - int((y - self.origin_y) / self.resolution) + dr
                if not (0 <= col < self.width and 0 <= row < self.height):
                    return False
                if self.pixels[row * self.width + col] < 200:
                    return False
        return True

    def clear(self, a, b, margin):
        steps = max(2, int(math.dist(a, b) / (self.resolution)))
        return all(
            self.free(a[0] + (b[0] - a[0]) * i / steps,
                      a[1] + (b[1] - a[1]) * i / steps, margin)
            for i in range(steps + 1))

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
    # Corners, so that no lane cuts through the rack block. The crossing from
    # the west corridor to the service lane is made at y = -5.0, which is the
    # middle of the only wide band of floor that is clear the whole way across;
    # the obvious run along the south wall is not, whatever the map looks like.
    ('south_turn', (-3.50, -5.00)),
    ('lane_south', (2.20, -5.00)),
    ('bay2_mouth', (2.20, -2.14)),
    ('bay3_mouth', (2.20, -3.94)),
]

# The floor plan of the building, as edges. The rack block fills the east half,
# so nothing crosses it: an aisle is a pocket with one mouth on the service
# lane, and a robot that has looked into one comes back out to try the other.
EDGES = [
    ('dock_south', 'south_turn'),
    ('south_turn', 'corridor'),
    ('corridor', 'dock_north'),
    ('south_turn', 'lane_south'),
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
    parser.add_argument('--map', required=True,
                        help="the occupancy grid to check the graph against")
    parser.add_argument('--margin', type=float, default=0.30,
                        help='clearance a lane must have on either side')
    args = parser.parse_args()

    floor = Floor(args.map)

    named = list(WAYPOINTS) + list(CHARGERS.items())

    # Check before writing. A graph that looks right and crosses a rack costs
    # an hour of watching a robot not move.
    problems = [f'waypoint {name} at {point} is not clear'
                for name, point in named if not floor.free(*point, args.margin)]
    lookup = dict(named)
    for a, b in list(EDGES) + [('dock_south', 'r1_charger'),
                               ('dock_north', 'r2_charger')]:
        if not floor.clear(lookup[a], lookup[b], args.margin):
            problems.append(f'lane {a} -> {b} crosses something')
    if problems:
        raise SystemExit('this graph does not fit the building:\n  ' +
                         '\n  '.join(problems))
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
