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
Writes the Open-RMF building map for the AWS small warehouse.

A building map is normally drawn in traffic-editor over a floorplan image, and
its vertices are pixels that a measurement converts to metres. None of that
applies here: the building already exists as a Gazebo world nobody drew, and
the coordinates are known in metres from `warehouse.hpp`. So the map is
written in `cartesian_meters`, which is the coordinate system for exactly this
case, and the numbers go in unchanged.

The map is needed even though this warehouse has no doors, no lifts and one
level. The slotcar plugin asks the building map server which level a robot is
standing on, and without an answer it publishes no robot state at all, so RMF
never sees the fleet.

    make_building_map.py --nav-graph maps/nav_graphs/0.yaml \
                         --out maps/aws_small_warehouse.building.yaml
"""

import argparse
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nav-graph', required=True,
                        help='the graph to embed, so the two cannot drift')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    graph = yaml.safe_load(open(args.nav_graph))
    level = graph['levels']['L1']

    # traffic-editor vertices are [x, y, z, name, params]; the nav graph's are
    # [x, y, {properties}]. The y inversion the reference_image path applies is
    # not applied here, so the metres are the world's own.
    vertices = []
    for x, y, properties in level['vertices']:
        params = {}
        if properties.get('is_charger'):
            params['is_charger'] = [4, True]
        if properties.get('is_parking_spot'):
            params['is_parking_spot'] = [4, True]
        vertices.append([float(x), float(y), 0.0,
                         properties.get('name', ''), params])

    # The nav graph holds each edge twice, once per direction. traffic-editor
    # holds it once and marks it bidirectional.
    seen = set()
    lanes = []
    for start, end, properties in level['lanes']:
        key = tuple(sorted((start, end)))
        if key in seen:
            continue
        seen.add(key)
        lanes.append([start, end, {
            'bidirectional': [4, True],
            'graph_idx': [2, 0],
            'orientation': [1, ''],
            'speed_limit': [3, properties.get('speed_limit', 0)],
        }])

    building = {
        'name': 'aws_small_warehouse',
        'coordinate_system': 'cartesian_meters',
        'graphs': {},
        'lifts': {},
        'levels': {
            'L1': {
                'elevation': 0,
                'vertices': vertices,
                'lanes': lanes,
                'doors': [],
                'floors': [],
                'measurements': [],
                'models': [],
                'walls': [],
                'layers': {},
            },
        },
    }

    with open(args.out, 'w') as handle:
        yaml.safe_dump(building, handle, default_flow_style=None, sort_keys=False)

    print(f'{len(vertices)} vertices, {len(lanes)} lanes -> {args.out}')


if __name__ == '__main__':
    main()
