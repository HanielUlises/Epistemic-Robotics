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
From an act and a seed, the world of one run of a faulty sensor in the fleet.

    make_world.py --act 2 --seed 4 --out DIR [--noise 0.01]

The seed picks the night (tools/worlds.py), the porter's round and the scouts
whose laser is short-sighted, and perturbs each robot's pose by up to 0.05 m
and 5 degrees. Writes DIR/world.sdf, DIR/models/ and DIR/world.json.

The ward is tools/layout.py: walls with a dark cap, so that from above they
read as a plan; floors of vinyl, lino, wood and tile; the band's linen rooms
as solid blocks; the passages' floors tinted, one colour a door, with a disc
at the mark before each door; Open Robotics' hospital models and a few of AWS
RoboMaker's residential ones; and in each passage a fire door, its leaf
across the passage if the porter shut it and against the wall if not. The
robots are TurtleBot3 Burgers with an 8 m laser and a coloured cap. A scout's
cap is a disc, blue, green or violet; a bed-mover carries a bed, drawn and not
solid, red or amber, and a disc of its colour marks the bed's place in its
ward.
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
import furniture as F  # noqa: E402
import layout as L  # noqa: E402
import ranks as R  # noqa: E402
import worlds as WD  # noqa: E402

ROBOT = {r.name: r for act in R.FLEETS for r in R.FLEETS[act]}

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'
CAP = {'b': (0.86, 0.12, 0.14), 'g': (0.93, 0.60, 0.10), 's': (0.12, 0.40, 0.85), 't': (0.13, 0.62, 0.30),
       'u': (0.55, 0.30, 0.80)}
BED = (1.0, 0.5)                 # a bed, along and across the bed-mover: as long as a passage is wide, nearly
WALL_BODY = (0.92, 0.92, 0.89)
WALL_CAP = (0.20, 0.21, 0.24)
LINEN = (0.62, 0.66, 0.64)
TINT = {'a': (0.33, 0.70, 0.66), 'b': (0.93, 0.68, 0.25), 'c': (0.58, 0.47, 0.80), 'd': (0.90, 0.45, 0.42)}
LEAF = (0.62, 0.10, 0.10)
GOAL = (0.86, 0.30, 0.36)


def col(rgb, a=1.0):
    return ' '.join(f'{c:.3f}' for c in rgb) + f' {a}'


def box(name, cx, cy, sx, sy, sz, rgb, z=None):
    z = sz / 2 if z is None else z
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{col(rgb)}</ambient><diffuse>{col(rgb)}</diffuse></material></visual>
        <visual name="cap"><cast_shadows>false</cast_shadows><pose>0 0 {sz / 2 - 0.01:.3f} 0 0 0</pose>
          <geometry><box><size>{sx + 0.002:.4f} {sy + 0.002:.4f} 0.02</size></box></geometry>
          <material><ambient>{col(WALL_CAP)}</ambient><diffuse>{col(WALL_CAP)}</diffuse></material></visual>
      </link>
    </model>"""


def wall(name, cx, cy, sx, sy):
    h = L.WALL_H
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><box><size>{sx:.4f} {sy:.4f} {h:.3f}</size></box></geometry></collision>
        <visual name="body"><cast_shadows>false</cast_shadows><pose>0 0 {(h - 0.02) / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {h - 0.02:.3f}</size></box></geometry>
          <material><ambient>{col(WALL_BODY)}</ambient><diffuse>{col(WALL_BODY)}</diffuse></material></visual>
        <visual name="cap"><cast_shadows>false</cast_shadows><pose>0 0 {h - 0.01:.3f} 0 0 0</pose>
          <geometry><box><size>{sx + 0.002:.4f} {sy + 0.002:.4f} 0.02</size></box></geometry>
          <material><ambient>{col(WALL_CAP)}</ambient><diffuse>{col(WALL_CAP)}</diffuse></material></visual>
      </link>
    </model>"""


def floor(name, x0, y0, x1, y1, material, z):
    return f"""
    <model name="{name}"><static>true</static><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{x1 - x0:.3f} {y1 - y0:.3f}</size></plane></geometry>
          <material><script><uri>model://br_materials/materials/scripts</uri>
            <uri>model://br_materials/materials/textures</uri><name>BR/{material}</name></script></material>
        </visual>
      </link>
    </model>"""


def tint(name, x0, y0, x1, y1, rgb, z, alpha=1.0):
    return f"""
    <model name="{name}"><static>true</static><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{x1 - x0:.3f} {y1 - y0:.3f}</size></plane></geometry>
          <material><ambient>{col(rgb, alpha)}</ambient><diffuse>{col(rgb, alpha)}</diffuse></material></visual>
      </link>
    </model>"""


def disc(name, x, y, r, rgb, z=0.008):
    return f"""
    <model name="{name}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><cylinder><radius>{r}</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>{col(rgb)}</ambient><diffuse>{col(rgb)}</diffuse></material></visual>
      </link>
    </model>"""


def door(d, s):
    """A fire door's leaf: kinematic, so that it can be swung and still stop a laser."""
    cx, cy, yaw = L.leaf_swing(d, s)
    lx, ly = L.LEAF
    h = L.WALL_H - 0.05
    t = TINT[d]
    return f"""
    <model name="door_{d}"><static>false</static><pose>{cx:.4f} {cy:.4f} 0 0 0 {yaw:.4f}</pose>
      <link name="link"><kinematic>true</kinematic><gravity>false</gravity>
        <inertial><mass>5.0</mass></inertial>
        <collision name="c"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><box><size>{ly:.3f} {lx:.3f} {h:.3f}</size></box></geometry></collision>
        <visual name="leaf"><cast_shadows>false</cast_shadows><pose>0 0 {h / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{ly:.3f} {lx:.3f} {h:.3f}</size></box></geometry>
          <material><ambient>{col(LEAF)}</ambient><diffuse>{col(LEAF)}</diffuse></material></visual>
        <visual name="top"><cast_shadows>false</cast_shadows><pose>0 0 {h + 0.01:.3f} 0 0 0</pose>
          <geometry><box><size>{ly + 0.06:.3f} {lx:.3f} 0.02</size></box></geometry>
          <material><ambient>{col(t)}</ambient><diffuse>{col(t)}</diffuse></material></visual>
        <visual name="glass"><cast_shadows>false</cast_shadows><pose>0 0 {h * 0.72:.3f} 0 0 0</pose>
          <geometry><box><size>{ly + 0.004:.3f} 0.22 0.36</size></box></geometry>
          <material><ambient>0.75 0.85 0.92 1</ambient><diffuse>0.75 0.85 0.92 1</diffuse></material></visual>
      </link>
    </model>"""


def textures(out):
    """The floor materials: AWS's wood, and vinyl, lino and tile drawn here."""
    from PIL import Image, ImageDraw
    d = os.path.join(out, 'models', 'br_materials')
    tex, scripts = os.path.join(d, 'materials', 'textures'), os.path.join(d, 'materials', 'scripts')
    os.makedirs(tex, exist_ok=True)
    os.makedirs(scripts, exist_ok=True)
    wood = Image.open(os.path.join(F.HOUSE, F.AWS_PREFIX + 'FloorB_01', 'materials', 'textures', 'aws_FloorB_01.png'))
    wood.convert('RGB').resize((256, 256)).save(os.path.join(tex, 'br_wood.png'))

    def tiles(name, n, base, grout, jitter, width=3):
        img = Image.new('RGB', (256, 256), base)
        dr = ImageDraw.Draw(img)
        rng = random.Random(name)
        s = 256 // n
        for i in range(n):
            for j in range(n):
                v = rng.randint(-jitter, jitter)
                dr.rectangle([i * s + 2, j * s + 2, (i + 1) * s - 2, (j + 1) * s - 2],
                             fill=tuple(max(0, min(255, c + v)) for c in base))
        for k in range(n + 1):
            dr.line([(k * s, 0), (k * s, 256)], fill=grout, width=width)
            dr.line([(0, k * s), (256, k * s)], fill=grout, width=width)
        img.save(os.path.join(tex, f'br_{name}.png'))

    def speckle(name, base, jitter):
        img = Image.new('RGB', (256, 256), base)
        px = img.load()
        rng = random.Random(name)
        for i in range(256):
            for j in range(256):
                v = rng.randint(-jitter, jitter)
                px[i, j] = tuple(max(0, min(255, c + v)) for c in base)
        img.save(os.path.join(tex, f'br_{name}.png'))
    tiles('hall', 2, (205, 208, 204), (168, 172, 168), 5, 2)
    tiles('tile', 4, (228, 232, 234), (170, 178, 184), 4)
    speckle('lino', (196, 214, 224), 9)
    mats = ''
    for name in ('wood', 'hall', 'tile', 'lino'):
        mats += f"""material BR/{name}
{{
  technique
  {{
    pass
    {{
      ambient 0.9 0.9 0.9 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture br_{name}.png
        filtering anisotropic
        max_anisotropy 8
      }}
    }}
  }}
}}

"""
    open(os.path.join(scripts, 'br.material'), 'w').write(mats)
    open(os.path.join(d, 'model.config'), 'w').write(
        '<?xml version="1.0"?><model><name>br_materials</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf><description>floor materials</description></model>\n')
    open(os.path.join(d, 'model.sdf'), 'w').write(
        '<?xml version="1.0"?><sdf version="1.6"><model name="br_materials"><static>true</static>'
        '<link name="l"/></model></sdf>\n')


def world_sdf(poses, loop):
    parts = []
    z = 0.002
    for name, x0, y0, x1, y1, material in L.floors():
        parts.append(floor(f'floor_{name}', x0, y0, x1, y1, material, z))
        z += 0.0004
    for k, (d, x0, y0, x1, y1) in enumerate(L.passage_floor()):
        parts.append(tint(f'lobby_{d}_{k}', x0, y0, x1, y1, TINT[d], 0.007 + 0.0004 * k))
    for i, (cx, cy, sx, sy) in enumerate(L.walls()):
        parts.append(wall(f'wall_{i}', cx, cy, sx, sy))
    for i, (x0, y0, x1, y1) in enumerate(L.linen()):
        parts.append(box(f'linen_{i}', (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, L.WALL_H - 0.03, LINEN))
    for d in L.DOORS:
        parts.append(door(d, 0.0 if d in loop else 1.0))
        parts.append(disc(f'mark_{d}', L.DOOR_X[d], L.MARK_Y, 0.16, TINT[d], 0.012))
    for i, (model, x, y, yaw) in enumerate(L.FURNITURE):
        parts.append(f"""
    <include><uri>model://{F.uri(model)}</uri><name>{F.uri(model).lower()}_{i}</name><static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose></include>""")
    for r in poses:
        if ROBOT[r].role != 'mover':
            continue
        gx, gy = L.BEDS[ROBOT[r].ward]
        parts.append(disc(f'goal_{r}', gx, gy, 0.3, CAP[r]))
    inc = ''
    for r, (x, y, yaw) in poses.items():
        inc += f"""
    <include><uri>model://br_{r}</uri><name>{r}</name><pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose></include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="ward">
    <gravity>0 0 -9.8</gravity>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.62 0.62 0.64 1</ambient><background>0.12 0.14 0.19 1</background>
      <shadows>false</shadows><grid>false</grid><origin_visual>false</origin_visual></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 15 0 0 0</pose>
      <diffuse>0.80 0.80 0.80 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>-0.25 0.35 -1</direction></light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material><ambient>0.26 0.29 0.27 1</ambient><diffuse>0.26 0.29 0.27 1</diffuse></material></visual>
      </link>
    </model>{''.join(parts)}{inc}
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
  </world>
</sdf>
"""


def robot_model(out, r, noise):
    dst = os.path.join(out, 'models', f'br_{r}')
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(BURGER, dst)
    p = os.path.join(dst, 'model.sdf')
    sdf = open(p).read()
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<range>.*?<max>)[^<]+(</max>)', r'\g<1>8.0\g<2>', sdf, flags=re.S)
    sdf = re.sub(r'(<sensor name="hls_lfcd_lds".*?<noise>.*?<stddev>)[^<]+(</stddev>)', rf'\g<1>{noise}\g<2>',
                 sdf, flags=re.S)
    sdf = sdf.replace('<visualize>true</visualize>', '<visualize>false</visualize>')
    c = col(CAP[r])
    if ROBOT[r].role == 'mover':
        bl, bw = BED
        cap = f'''
      <visual name="cap">
        <pose>-0.032 0 0.215 0 0 0</pose>
        <geometry><cylinder><radius>0.09</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
      </visual>
      <visual name="bed_frame">
        <pose>-0.03 0 0.27 0 0 0</pose>
        <geometry><box><size>{bl} {bw} 0.04</size></box></geometry>
        <material><ambient>0.80 0.82 0.84 1</ambient><diffuse>0.80 0.82 0.84 1</diffuse></material>
      </visual>
      <visual name="bed_sheet">
        <pose>-0.08 0 0.31 0 0 0</pose>
        <geometry><box><size>{bl - 0.2} {bw - 0.06} 0.05</size></box></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
      </visual>
      <visual name="bed_pillow">
        <pose>{bl / 2 - 0.15:.3f} 0 0.33 0 0 0</pose>
        <geometry><box><size>0.18 {bw - 0.14} 0.05</size></box></geometry>
        <material><ambient>0.97 0.97 0.95 1</ambient><diffuse>0.97 0.97 0.95 1</diffuse></material>
      </visual>'''
    else:
        cap = f'''
      <visual name="cap">
        <pose>-0.032 0 0.215 0 0 0</pose>
        <geometry><cylinder><radius>0.09</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
      </visual>'''
    sdf = sdf.replace('<link name="base_link">', '<link name="base_link">' + cap, 1)
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="br_{r}">', sdf)
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->', f'<namespace>/{r}</namespace>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{r}/base_scan</frame_name>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{r}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{r}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{r}/imu_link</frame_name>')
    open(p, 'w').write(sdf)
    cfg = os.path.join(dst, 'model.config')
    text = open(cfg).read()
    open(cfg, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>br_{r}</name>', text, count=1))
    for must in ('<max>8.0</max>', f'<namespace>/{r}</namespace>', f'{r}/odom', f'{r}/base_scan', '<visual name="cap">'):
        assert must in sdf, f'{r}: {must} not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--act', type=int, required=True, choices=(1, 2, 3))
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    a = ap.parse_args()
    rng = random.Random(a.act * 1000 + a.seed)
    loop, short = WD.night(a.act, a.seed)

    def jitter(p):
        return (p[0] + rng.uniform(-0.05, 0.05), p[1] + rng.uniform(-0.05, 0.05),
                p[2] + rng.uniform(-math.radians(5), math.radians(5)))
    poses = {r.name: jitter(L.ROOMS[r.room]) for r in R.fleet(a.act)}
    os.makedirs(a.out, exist_ok=True)
    textures(a.out)
    for r in poses:
        robot_model(a.out, r, a.noise)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(poses, loop))
    json.dump({'act': a.act, 'seed': a.seed, 'loop': list(loop), 'short': sorted(short),
               'tag': WD.tag((loop, short)), 'noise': a.noise,
               'robots': list(poses), 'poses': {r: [round(v, 4) for v in p] for r, p in poses.items()},
               'frequency': R.frequency(a.act, (loop, short))},
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
