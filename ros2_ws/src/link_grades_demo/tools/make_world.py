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
From a seed, the world of one run of the link grades.

    make_world.py --seed 1 --out DIR [--noise 0.01]

The seed fixes the world: debris in the corridor for odd seeds, none for even
ones. It also perturbs both robots' poses by up to 0.08 m and 8 degrees, and
fixes how long the link stays down under the partial grade: 20 + 4 * seed
seconds, from 24 s for seed 1 to 100 s for seed 20 (tools/grades.py). Writes
DIR/world.sdf, DIR/models/lg_r1/, lg_r2/ and DIR/world.json.

Each robot is the TurtleBot3 Burger with its laser's range set to 8 m, its
rays not drawn and a coloured cap over the laser -- r1 orange, r2 blue -- and
with every ROS interface in its own namespace and every frame prefixed with
its name: /r1/scan, /r1/odom, /r1/cmd_vel, r1/odom, r1/base_footprint. The
simulator's odometry is the world pose, so each robot's odom frame is the
world frame.
"""

import argparse
import json
import math
import os
import random
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grades as R  # noqa: E402
import layout as L  # noqa: E402

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'
WALL = (0.32, 0.33, 0.35)
DEBRIS = (0.55, 0.42, 0.30)
SPOT = (0.85, 0.65, 0.10)
REGION = (0.55, 0.75, 0.95)
# A cap over each robot's laser, for whoever watches: a visual only, which no
# ray of either laser can hit, since a ray sensor sees collision shapes.
CAP = {'r1': (0.90, 0.42, 0.08), 'r2': (0.12, 0.38, 0.80)}


def box(name, cx, cy, sx, sy, sz, rgb, z=None, collide=True):
    z = sz / 2 if z is None else z
    col = ' '.join(f'{c:.3f}' for c in rgb)
    c = (f'<collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>'
         if collide else '')
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">{c}
        <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual>
      </link>
    </model>"""


def world_sdf(blocked, poses):
    parts = []
    for i, (cx, cy, sx, sy) in enumerate(L.walls()):
        parts.append(box(f'wall_{i}', cx, cy, sx, sy, 1.0, WALL))
    if blocked:
        x0, y0, x1, y1 = L.debris_box()
        parts.append(box('debris', (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.5, DEBRIS))
    # the two spots and the region to decide, marked on the floor for whoever watches the film
    for r, (x, y) in L.SPOT.items():
        parts.append(box(f'spot_{r}', x, y, 0.36, 0.36, 0.004, SPOT, z=0.002, collide=False))
    x0, y0, x1, y1 = L.REGION
    parts.append(box('region', (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.004, REGION, z=0.002,
                     collide=False))
    inc = ''
    for r, (x, y, yaw) in poses.items():
        inc += f"""
    <include>
      <uri>model://lg_{r}</uri>
      <name>{r}</name>
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose>
    </include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="link_grades">
    <gravity>0 0 -9.8</gravity>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.85 0.87 0.9 1</background>
      <shadows>false</shadows><grid>false</grid></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 15 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction></light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.93 0.93 0.92 1</ambient><diffuse>0.93 0.93 0.92 1</diffuse></material></visual>
      </link>
    </model>{''.join(parts)}{inc}
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
  </world>
</sdf>
"""


def robot_model(out, r, noise):
    dst = os.path.join(out, 'models', f'lg_{r}')
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(BURGER, dst)
    p = os.path.join(dst, 'model.sdf')
    sdf = open(p).read()
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<range>.*?<max>)[^<]+(</max>)', r'\g<1>8.0\g<2>', sdf, flags=re.S)
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<noise>.*?<stddev>)[^<]+(</stddev>)', rf'\g<1>{noise}\g<2>',
                 sdf, flags=re.S)
    sdf = sdf.replace('<visualize>true</visualize>', '<visualize>false</visualize>')
    col = ' '.join(f'{c:.2f}' for c in CAP[r])
    sdf = sdf.replace('<link name="base_link">', f'''<link name="base_link">
      <visual name="cap">
        <pose>-0.032 0 0.215 0 0 0</pose>
        <geometry><cylinder><radius>0.08</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material>
      </visual>''', 1)
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="lg_{r}">', sdf)
    # Every plugin in the robot's namespace, and every frame prefixed with its name.
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->', f'<namespace>/{r}</namespace>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{r}/base_scan</frame_name>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{r}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{r}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{r}/imu_link</frame_name>')
    open(p, 'w').write(sdf)
    c = os.path.join(dst, 'model.config')
    text = open(c).read()
    open(c, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>lg_{r}</name>', text, count=1))
    for must in ('<max>8.0</max>', f'<namespace>/{r}</namespace>', f'{r}/odom', f'{r}/base_scan',
                 '<visual name="cap">'):
        assert must in sdf, f'{r}: {must} not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    blocked = a.seed % 2 == 1

    def jitter(p):
        return (p[0] + rng.uniform(-0.08, 0.08), p[1] + rng.uniform(-0.08, 0.08),
                p[2] + rng.uniform(-math.radians(8), math.radians(8)))
    poses = {'r1': jitter(L.DOCK), 'r2': jitter(L.STATION)}
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(blocked, poses))
    for r in poses:
        robot_model(a.out, r, a.noise)
    json.dump({'seed': a.seed, 'blocked': blocked, 'outage': R.outage(a.seed), 'noise': a.noise,
               'poses': {r: [round(v, 4) for v in p] for r, p in poses.items()}},
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
