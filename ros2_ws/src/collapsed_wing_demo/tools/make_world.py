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
From a seed, the world of one run of the collapsed wing.

    make_world.py --seed 1 --out DIR [--noise 0.01]

The seed fixes which of the three ways into the north wing are blocked: the
seven worlds in which at least one is clear, in turn, seed 1 the first
(tools/worlds.py). It also perturbs the three robots' poses by up to 0.08 m and
8 degrees. Writes DIR/world.sdf, DIR/models/cw_s1/, cw_s2/, cw_m/ and
DIR/world.json.

Each robot is the TurtleBot3 Burger with its laser's range set to 8 m, its
rays not drawn and a coloured cap over the laser -- s1 orange, s2 blue, the
medic red -- and with every ROS interface in its own namespace and every
frame prefixed with its name: /s1/scan, /s1/odom, /s1/cmd_vel, s1/odom,
s1/base_footprint.
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
import layout as L  # noqa: E402
import worlds as WD  # noqa: E402

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'
WALL = (0.32, 0.33, 0.35)
RUBBLE = (0.50, 0.38, 0.27)
CLUTTER = (0.58, 0.58, 0.64)
SPOT = (0.85, 0.65, 0.10)
VICTIM = (0.85, 0.20, 0.20)
# A cap over each robot's laser, for whoever watches: a visual only, which no
# ray of any laser can hit, since a ray sensor sees collision shapes.
CAP = {'s1': (0.90, 0.42, 0.08), 's2': (0.12, 0.38, 0.80), 'm': (0.82, 0.10, 0.14)}


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


def world_sdf(world, poses):
    parts = []
    for i, (cx, cy, sx, sy) in enumerate(L.walls()):
        parts.append(box(f'wall_{i}', cx, cy, sx, sy, 1.2, WALL))
    for i, (cx, cy, sx, sy, sz) in enumerate(L.clutter()):
        parts.append(box(f'clutter_{i}', cx, cy, sx, sy, sz, CLUTTER))
    for d, clear in world.items():
        if not clear:
            cx, cy, sx, sy = L.RUBBLE[d]
            parts.append(box(f'rubble_{d}', cx, cy, sx, sy, L.RUBBLE_H, RUBBLE))
    # each scout's vantage and the victim, marked on the floor for whoever watches the film
    for r, (x, y) in L.VANTAGE.items():
        parts.append(box(f'vantage_{r}', x, y, 0.36, 0.36, 0.004, SPOT, z=0.002, collide=False))
    parts.append(box('victim', L.VICTIM[0], L.VICTIM[1], 0.5, 0.5, 0.004, VICTIM, z=0.002, collide=False))
    inc = ''
    for r, (x, y, yaw) in poses.items():
        inc += f"""
    <include>
      <uri>model://cw_{r}</uri>
      <name>{r}</name>
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose>
    </include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="collapsed_wing">
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
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
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
    dst = os.path.join(out, 'models', f'cw_{r}')
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
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="cw_{r}">', sdf)
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
    open(c, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>cw_{r}</name>', text, count=1))
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
    world = WD.world(a.seed)

    def jitter(p):
        return (p[0] + rng.uniform(-0.08, 0.08), p[1] + rng.uniform(-0.08, 0.08),
                p[2] + rng.uniform(-math.radians(8), math.radians(8)))
    poses = {r: jitter(L.START[r]) for r in ('s1', 's2', 'm')}
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(world, poses))
    for r in poses:
        robot_model(a.out, r, a.noise)
    json.dump({'seed': a.seed, 'clear': world, 'noise': a.noise,
               'poses': {r: [round(v, 4) for v in p] for r, p in poses.items()}},
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)

if __name__ == '__main__':
    main()
