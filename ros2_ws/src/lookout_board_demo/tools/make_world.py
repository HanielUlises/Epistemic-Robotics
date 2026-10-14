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
From a seed, the world of one run of the lookout board.

    make_world.py --seed 1 --out DIR [--noise 0.01]

The seed fixes three things and only three:

  the world      which lookout has the clear view, and whether a crate stands
                 in the zone: world (seed - 1) mod 8, lookout 1 + w // 2 open
                 and the zone blocked when w is odd, so that every eight seeds
                 meet every world once;
  the start      the nominal start perturbed by up to 0.1 m on each axis and
                 10 degrees in heading, uniformly;
  nothing else   the laser's noise is an argument, recorded with the seed.

Writes DIR/world.sdf, DIR/models/lb_burger/ and DIR/world.json. The robot is
the TurtleBot3 Burger with its laser's range set to 8 m (the LDS-01 reaches
3.5 m, too short to see across the room through a window) and its rays not
drawn; the installed model is left as it is.
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

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'


def box(name, cx, cy, sx, sy, sz, rgb, z=None):
    z = sz / 2 if z is None else z
    col = ' '.join(f'{c:.3f}' for c in rgb)
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual>
      </link>
    </model>"""


def cylinder(name, cx, cy, r, h, rgb):
    col = ' '.join(f'{c:.3f}' for c in rgb)
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {h / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse><emissive>{col} 1</emissive></material></visual>
      </link>
    </model>"""


WALL = (0.32, 0.33, 0.35)
SCREEN = (0.08, 0.13, 0.26)
CRATE = (0.70, 0.36, 0.18)
PANEL = (0.15, 0.15, 0.17)
LIT = (0.10, 0.75, 0.30)


def world_sdf(clear, blocked, pose):
    parts = []
    for i, (cx, cy, sx, sy) in enumerate(L.walls()):
        parts.append(box(f'wall_{i}', cx, cy, sx, sy, 1.0, WALL))
    for k in range(1, 5):
        if k != clear:
            x0, y0, x1, y1 = L.screen_box(k)
            parts.append(box(f'screen_{k}', (x0 + x1) / 2, (y0 + y1) / 2,
                             x1 - x0, y1 - y0, 0.9, SCREEN))
    if blocked:
        parts.append(box('crate', 0.0, 0.0, 2 * L.CRATE, 2 * L.CRATE, 0.45, CRATE))
    # The board: a dark panel against the west wall, four slot frames above
    # the laser's plane, and one post, glowing, in front of the lit slot.
    ys = L.SLOT_Y
    parts.append(box('board_panel', L.BOARD_PANEL_X + 0.03, (ys[0] + ys[-1]) / 2,
                     0.06, abs(ys[0] - ys[-1]) + 0.5, 1.2, PANEL))
    for k, y in enumerate(ys, 1):
        parts.append(box(f'slot_frame_{k}', L.BOARD_PANEL_X + 0.07, y, 0.02, 0.26, 0.26,
                         (0.6, 0.6, 0.62), z=0.95))
    parts.append(cylinder('lit_post', L.POST_X, ys[clear - 1], 0.07, 0.6, LIT))
    x, y, yaw = pose
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="lookout_board">
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
    </model>{''.join(parts)}
    <include>
      <uri>model://lb_burger</uri>
      <name>r1</name>
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose>
    </include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
  </world>
</sdf>
"""


def robot_model(out, noise):
    dst = os.path.join(out, 'models', 'lb_burger')
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(BURGER, dst)
    sdf_path = os.path.join(dst, 'model.sdf')
    sdf = open(sdf_path).read()
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<range>.*?<max>)[^<]+(</max>)',
                 r'\g<1>8.0\g<2>', sdf, flags=re.S)
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<noise>.*?<stddev>)[^<]+(</stddev>)',
                 rf'\g<1>{noise}\g<2>', sdf, flags=re.S)
    sdf = sdf.replace('<visualize>true</visualize>', '<visualize>false</visualize>')
    sdf = sdf.replace("<model name='turtlebot3_burger'>", "<model name='lb_burger'>")
    sdf = sdf.replace('<model name="turtlebot3_burger">', '<model name="lb_burger">')
    open(sdf_path, 'w').write(sdf)
    cfg = os.path.join(dst, 'model.config')
    text = open(cfg).read()
    open(cfg, 'w').write(text.replace('TurtleBot3(Burger)', 'lb_burger')
                         .replace('turtlebot3_burger', 'lb_burger'))
    assert '<max>8.0</max>' in sdf, 'the laser range was not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    w = (a.seed - 1) % 8
    clear, blocked = 1 + w // 2, bool(w % 2)
    x, y, yaw = L.START
    pose = (x + rng.uniform(-0.1, 0.1), y + rng.uniform(-0.1, 0.1),
            yaw + rng.uniform(-math.radians(10), math.radians(10)))
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(clear, blocked, pose))
    robot_model(a.out, a.noise)
    json.dump({'seed': a.seed, 'world': w, 'clear': clear, 'blocked': blocked,
               'noise': a.noise, 'x0': round(pose[0], 4), 'y0': round(pose[1], 4),
               'yaw0': round(pose[2], 4)},
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
