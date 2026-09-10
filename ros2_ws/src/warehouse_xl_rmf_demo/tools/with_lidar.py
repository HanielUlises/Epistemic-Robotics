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
Writes a copy of the laser-carrying robot model that belongs to one robot.

The single-site demo fitted a laser to exactly one robot, and said why: Gazebo
Classic names a plugin's node after the plugin, so two robots spawned from one
model file give two nodes called `/lds_driver`, and the server reports the
collision and then segfaults. With one sensing agent that restriction cost
nothing, because there was one thing to look at.

The multi-site domain scans two sites with two robots, so it costs the mission.
The fix is that the ray plugin takes a `<ros><namespace>`, and a namespace is
what a node name is unique within: `/r1/lds_driver` and `/r2/lds_driver` are
two nodes and not one name twice. The scan topic follows the namespace, which
is why the perception node is configured with a topic per robot rather than one
`/scan` for the fleet.

The alternative -- a checked-in model file per robot -- was rejected because
the three would differ by one string and drift the moment the geometry
changed. This is generated at launch, from the same model the single-site demo
spawns, so there is one description of the robot.

    with_lidar.py --model models/TurtleBot3Waffle/model_lidar.sdf \
                  --robot r1 --out /tmp/r1.sdf
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET


def namespaced(text, robot):
    """Put every ray plugin in the model into `robot`'s namespace."""
    root = ET.fromstring(text)

    lasers = [p for p in root.iter('plugin')
              if 'ray_sensor' in (p.get('filename') or '')]
    if not lasers:
        raise SystemExit(
            'no ray-sensor plugin in this model: it is not the laser-carrying '
            'one, and namespacing it would silently produce a robot that '
            'senses nothing.')

    for plugin in lasers:
        ros = plugin.find('ros')
        if ros is None:
            ros = ET.SubElement(plugin, 'ros')
        existing = ros.find('namespace')
        if existing is not None:
            ros.remove(existing)
        # First child, so the element order reads namespace-then-remapping the
        # way the plugin's own documentation writes it. gazebo_ros does not
        # care about the order; a person reading the file does.
        ns = ET.Element('namespace')
        ns.text = f'/{robot}'
        ros.insert(0, ns)

    # ElementTree drops the declaration and every comment. The comments are
    # this model's explanation of why it is built the way it is, and they
    # belong in the file that is checked in rather than in the temporary this
    # writes; the declaration is put back because a header that was there
    # before an automatic edit should be there after it.
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True,
                    help='the laser-carrying model, models/TurtleBot3Waffle/'
                         'model_lidar.sdf')
    ap.add_argument('--robot', required=True,
                    help='the robot this copy is for; becomes the namespace')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', args.robot):
        raise SystemExit(
            f'{args.robot!r} is not usable as a ROS namespace, and Gazebo will '
            f'accept it, start, and publish nothing.')

    with open(args.model) as handle:
        text = handle.read()

    with open(args.out, 'w') as handle:
        handle.write(namespaced(text, args.robot))

    print(f'{args.out}: laser in namespace /{args.robot}, '
          f'scanning on /{args.robot}/scan')
    return 0


if __name__ == '__main__':
    sys.exit(main())
