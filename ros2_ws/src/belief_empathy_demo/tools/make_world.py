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
From a seed, the world of one run of the belief and empathy demonstration.

    make_world.py --seed 1 --out DIR [--noise 0.01] [--variant latent]

The seed fixes the world (which particle each ground vehicle tracks, so how
many faults it has), the site of the task and the times of the faults
(tools/worlds.py); with --variant latent, the seed's pattern of drive faults
and latent tool faults (worlds.scenario_latent). It also perturbs the ground vehicles' starting poses by up
to 0.05 m and 5 degrees. Writes DIR/world.sdf, DIR/models/be_g1/, be_g2/ and
DIR/world.json.

Each ground vehicle is the TurtleBot3 Burger with its laser's rays not drawn
and a coloured cap over the laser -- g1 blue, g2 orange -- and with every ROS
interface in its own namespace and every frame prefixed with its name. The
aerial vehicle is a quadrotor drawn from boxes and cylinders, with no
collision shape and gravity off, flown in the plane at layout.ALTITUDE by
gazebo_ros_planar_move on /u/cmd_vel, which also publishes /u/odom: it flies
over the obstacles and is seen by no laser.
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
WALL = (0.40, 0.41, 0.44)
CRATE = (0.62, 0.50, 0.36)
STREET = (0.80, 0.80, 0.78)
BASE = (0.25, 0.55, 0.30)
SITE = (0.85, 0.20, 0.20)
CAP = {'g1': (0.12, 0.38, 0.80), 'g2': (0.90, 0.42, 0.08)}
UAV = (0.15, 0.62, 0.38)


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


def quadrotor(pose):
    """A quadrotor of boxes and cylinders, visual only, flown by planar_move."""
    x, y, yaw = pose
    col = ' '.join(f'{c:.2f}' for c in UAV)
    arms, rotors = '', ''
    for k, a in enumerate((45, 135, 225, 315)):
        r = math.radians(a)
        arms += f"""
        <visual name="arm{k}"><pose>{0.13 * math.cos(r):.3f} {0.13 * math.sin(r):.3f} 0 0 0 {r:.3f}</pose>
          <geometry><box><size>0.26 0.03 0.02</size></box></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>"""
        rotors += f"""
        <visual name="rotor{k}"><pose>{0.24 * math.cos(r):.3f} {0.24 * math.sin(r):.3f} 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.11</radius><length>0.008</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.72 0.8</ambient><diffuse>0.7 0.7 0.72 0.8</diffuse></material></visual>"""
    return f"""
    <model name="u">
      <pose>{x:.4f} {y:.4f} {L.ALTITUDE:.3f} 0 0 {yaw:.4f}</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.02</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <visual name="body"><geometry><box><size>0.20 0.20 0.07</size></box></geometry>
          <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual>
        <visual name="nose"><pose>0.11 0 0 0 0 0</pose><geometry><box><size>0.05 0.06 0.04</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>{arms}{rotors}
      </link>
      <plugin name="move" filename="libgazebo_ros_planar_move.so">
        <ros><namespace>/u</namespace></ros>
        <update_rate>50</update_rate><publish_rate>20</publish_rate>
        <publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf>
        <odometry_frame>u/odom</odometry_frame><robot_base_frame>u/base_footprint</robot_base_frame>
      </plugin>
    </model>"""


def world_sdf(scenario, poses):
    parts = []
    x0, y0, x1, y1 = L.AREA
    t = 0.15
    for i, (cx, cy, sx, sy) in enumerate([((x0 + x1) / 2, y0 - t / 2, x1 - x0 + 2 * t, t),
                                          ((x0 + x1) / 2, y1 + t / 2, x1 - x0 + 2 * t, t),
                                          (x0 - t / 2, (y0 + y1) / 2, t, y1 - y0),
                                          (x1 + t / 2, (y0 + y1) / 2, t, y1 - y0)]):
        parts.append(box(f'wall_{i}', cx, cy, sx, sy, 0.5, WALL))
    for i, (cx, cy, sx, sy, sz) in enumerate(L.OBSTACLES):
        parts.append(box(f'crate_{i}', cx, cy, sx, sy, sz, CRATE))
    # The streets and avenues, painted on the floor for whoever watches; no laser sees paint.
    for y in L.STREETS:
        parts.append(box(f'street_{y:+.0f}', 0.0, y, 28.6, 0.08, 0.004, STREET, z=0.002, collide=False))
    for x in L.AVENUES:
        parts.append(box(f'avenue_{x:+.0f}', x, 0.0, 0.08, 12.6, 0.004, STREET, z=0.002, collide=False))
    parts.append(box('base', L.BASE[0], L.BASE[1], 2.4, 0.9, 0.004, BASE, z=0.003, collide=False))
    sx, sy = scenario['site']
    parts.append(box('task_site', sx, sy, 0.7, 0.7, 0.006, SITE, z=0.004, collide=False))
    inc = ''
    for r in L.UGVS:
        x, y, yaw = poses[r]
        inc += f"""
    <include>
      <uri>model://be_{r}</uri>
      <name>{r}</name>
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose>
    </include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="belief_empathy">
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
    </model>{''.join(parts)}{quadrotor(poses['u'])}{inc}
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
  </world>
</sdf>
"""


def robot_model(out, r, noise):
    dst = os.path.join(out, 'models', f'be_{r}')
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(BURGER, dst)
    p = os.path.join(dst, 'model.sdf')
    sdf = open(p).read()
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
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="be_{r}">', sdf)
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->', f'<namespace>/{r}</namespace>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{r}/base_scan</frame_name>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{r}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{r}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{r}/imu_link</frame_name>')
    open(p, 'w').write(sdf)
    c = os.path.join(dst, 'model.config')
    text = open(c).read()
    open(c, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>be_{r}</name>', text, count=1))
    for must in (f'<namespace>/{r}</namespace>', f'{r}/odom', f'{r}/base_scan', '<visual name="cap">'):
        assert must in sdf, f'{r}: {must} not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    ap.add_argument('--variant', default='replica', choices=('replica', 'latent'))
    a = ap.parse_args()
    rng = random.Random(a.seed)
    scenario = WD.scenario_latent(a.seed) if a.variant == 'latent' else dict(WD.scenario(a.seed), variant='replica')

    def jitter(p):
        return (p[0] + rng.uniform(-0.05, 0.05), p[1] + rng.uniform(-0.05, 0.05),
                p[2] + rng.uniform(-math.radians(5), math.radians(5)))
    poses = {r: jitter(L.START[r]) for r in L.UGVS}
    poses['u'] = L.START['u']
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(scenario, poses))
    for r in L.UGVS:
        robot_model(a.out, r, a.noise)
    json.dump(dict(scenario, noise=a.noise, poses={r: [round(v, 4) for v in p] for r, p in poses.items()}),
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
