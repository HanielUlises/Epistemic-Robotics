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
One robot's model: ROBOTIS's Waffle, moved into its own namespace and given
the laser its role calls for.

The shipped SDF is written for one robot. Every plugin's `<ros>` block carries
a commented-out namespace, and the frames are `odom`, `base_footprint` and
`base_scan`, so a second copy publishes on the first one's topics and claims
the first one's transforms. Each robot therefore gets its own copy with the
namespace filled in and the frames prefixed -- the same rewrite
`warehouse_demo` applies to the Burger, for the same reason.

Three further changes, all deliberate:

  * The laser. A scout carries a mapping laser, twelve metres and 720 beams;
    the carrier keeps the stock 3.5 m safety scanner. That difference is what
    makes it the scouts' job to survey: from the mouth of a bay the carrier's
    scanner reads the first few metres of it and no more.
  * The RGB camera is removed. Nothing reads it, and under the software
    renderer the recording runs on it costs more than the rest of the robot.
  * The laser is not visualised in Gazebo; RViz shows the scans.
"""

import re

WAFFLE = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'


def robot_sdf(ns, lidar_range, samples, source=WAFFLE, colour=None):
    with open(source) as fh:
        sdf = fh.read()

    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->',
                      f'<namespace>/{ns}</namespace>')

    # Frame tags only. Rewriting the bare word `odom` would also rewrite the
    # topic, and the namespace would then be applied twice.
    for tag, value in (('odometry_frame', 'odom'),
                       ('robot_base_frame', 'base_footprint'),
                       ('frame_name', 'base_scan')):
        sdf = sdf.replace(f'<{tag}>{value}</{tag}>', f'<{tag}>{ns}/{value}</{tag}>')

    # The laser.
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<samples>)\d+(</samples>)',
                 rf'\g<1>{samples}\g<2>', sdf, count=1, flags=re.S)
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<range>.*?<max>)[\d.]+(</max>)',
                 rf'\g<1>{lidar_range}\g<2>', sdf, count=1, flags=re.S)
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<update_rate>)\d+(</update_rate>)',
                 r'\g<1>10\g<2>', sdf, count=1, flags=re.S)

    # The laser is not drawn in Gazebo. A twelve-metre fan paints the floor
    # blue for as far as the shot reaches; RViz draws the same scans as points.
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<visualize>)true(</visualize>)',
                 r'\g<1>false\g<2>', sdf, count=1, flags=re.S)

    # The camera, sensor and all.
    sdf = re.sub(r'<sensor name="camera" type="camera">.*?</sensor>', '', sdf,
                 flags=re.S)

    # Tint the chassis so the three robots can be told apart on screen. The
    # base visual already carries a material; its colours are replaced, since a
    # second <material> in one visual is not an error Gazebo reports.
    if colour is not None:
        r, g, b = colour
        sdf = re.sub(
            r'(<visual name="base_visual">.*?<ambient>)[^<]*(</ambient>\s*<diffuse>)[^<]*(</diffuse>)',
            rf'\g<1>{r} {g} {b} 1.0\g<2>{r} {g} {b} 1.0\g<3>', sdf, count=1, flags=re.S)
    return sdf


if __name__ == '__main__':
    import sys
    print(robot_sdf(sys.argv[1] if len(sys.argv) > 1 else 'r1', 12.0, 720))
