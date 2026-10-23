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
From a seed and a case, the world of one Gazebo run (tools/arena.py).

    make_world.py --seed 1 --case 2 --out DIR

Writes DIR/world.sdf, DIR/models/gc_r1/ to gc_r3/ and DIR/world.json. Each
robot is the TurtleBot3 Burger with its laser's rays not drawn, a coloured cap
over the laser (r1 blue, r2 orange, r3 green), every ROS interface in its own
namespace and every frame prefixed with its name. The tasks are drawn on the
floor, as squares whose side is the task's radius, red for the one that needs
two robots and purple for the one that needs three; nobody's laser sees them,
and a robot finds a task when it comes within its sensing range of it.
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
import arena as A  # noqa: E402

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'
WALL = (0.40, 0.41, 0.44)
CRATE = (0.62, 0.50, 0.36)
BASE = (0.25, 0.55, 0.30)
TASK = {2: (0.85, 0.20, 0.20), 3: (0.55, 0.25, 0.70), 1: (0.85, 0.55, 0.15)}


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


def world_sdf(scn, poses):
    parts = []
    W, H = A.SPEC.width, A.SPEC.height
    t = 0.15
    for i, (cx, cy, sx, sy) in enumerate([(W / 2, -t / 2, W + 2 * t, t), (W / 2, H + t / 2, W + 2 * t, t),
                                          (-t / 2, H / 2, t, H), (W + t / 2, H / 2, t, H)]):
        parts.append(box(f'wall_{i}', cx, cy, sx, sy, 0.5, WALL))
    for i, (cx, cy) in enumerate(scn['crates']):
        parts.append(box(f'crate_{i}', cx, cy, A.CRATE, A.CRATE, 0.45, CRATE))
    hx, hy = A.SPEC.home
    parts.append(box('base', hx, hy, 2.0, 1.2, 0.004, BASE, z=0.003, collide=False))
    for tk in scn['tasks']:
        x, y = tk['pos']
        parts.append(box(f"task_{tk['id']}", x, y, A.SPEC.r_task, A.SPEC.r_task, 0.006, TASK[tk['k']], z=0.004,
                         collide=False))
    inc = ''
    for r in A.ROBOTS:
        x, y, yaw = poses[r]
        inc += f"""
    <include>
      <uri>model://gc_{r}</uri>
      <name>{r}</name>
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose>
    </include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="gossip_coverage">
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
    dst = os.path.join(out, 'models', f'gc_{r}')
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(BURGER, dst)
    p = os.path.join(dst, 'model.sdf')
    sdf = open(p).read()
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<noise>.*?<stddev>)[^<]+(</stddev>)', rf'\g<1>{noise}\g<2>',
                 sdf, flags=re.S)
    sdf = sdf.replace('<visualize>true</visualize>', '<visualize>false</visualize>')
    col = ' '.join(f'{c:.2f}' for c in A.CAP[r])
    sdf = sdf.replace('<link name="base_link">', f'''<link name="base_link">
      <visual name="cap">
        <pose>-0.032 0 0.215 0 0 0</pose>
        <geometry><cylinder><radius>0.08</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material>
      </visual>''', 1)
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="gc_{r}">', sdf)
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->', f'<namespace>/{r}</namespace>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{r}/base_scan</frame_name>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{r}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{r}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{r}/imu_link</frame_name>')
    open(p, 'w').write(sdf)
    c = os.path.join(dst, 'model.config')
    text = open(c).read()
    open(c, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>gc_{r}</name>', text, count=1))
    for must in (f'<namespace>/{r}</namespace>', f'{r}/odom', f'{r}/base_scan', '<visual name="cap">'):
        assert must in sdf, f'{r}: {must} not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--case', type=int, required=True, choices=(1, 2))
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    a = ap.parse_args()
    scn = A.scenario(a.seed, a.case)
    rng = random.Random(a.seed)
    poses = {}
    for r, (x, y, yaw) in A.start_poses().items():
        poses[r] = (x + rng.uniform(-0.03, 0.03), y + rng.uniform(-0.03, 0.03),
                    yaw + rng.uniform(-math.radians(4), math.radians(4)))
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(scn, poses))
    for r in A.ROBOTS:
        robot_model(a.out, r, a.noise)
    json.dump(dict(scn, noise=a.noise, poses={r: [round(v, 4) for v in p] for r, p in poses.items()}),
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
