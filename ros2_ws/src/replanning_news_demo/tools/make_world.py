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
From an act and a seed, the world of one run of replanning on news.

    make_world.py --act 2 --seed 4 --out DIR [--noise 0.01]

The seed picks, in turn, one of the door states in which the act's news is
true (seed 1 the first, tools/worlds.py), and perturbs each robot's pose by up
to 0.05 m and 5 degrees. Writes DIR/world.sdf, DIR/models/ and DIR/world.json.

The residence is tools/layout.py: walls with a dark cap, so that from above
they read as a plan; floors of wood, carpet and tile, each a textured plane;
the closed linen rooms as solid blocks; AWS RoboMaker's residential furniture
(MIT-0); and in each passage AWS's door, cut down to the walls' height, shut
across the passage or swung open against the corridor's wall. Each robot is a
TurtleBot3 Burger with its laser's range set to 8 m and a coloured cap: the
courier red, the scout blue, the cleaner green.
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
import worlds as WD  # noqa: E402

BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger'
CAP = {'c': (0.86, 0.12, 0.14), 's': (0.12, 0.40, 0.85), 'k': (0.15, 0.62, 0.30)}
WALL_BODY = (0.93, 0.91, 0.87)
WALL_CAP = (0.20, 0.21, 0.24)
LINEN = (0.66, 0.64, 0.61)
TINT = {'w': (0.33, 0.70, 0.66), 'm': (0.93, 0.68, 0.25), 'e': (0.58, 0.47, 0.80)}
GOAL = (0.86, 0.30, 0.36)


def col(rgb, a=1.0):
    return ' '.join(f'{c:.3f}' for c in rgb) + f' {a}'


def box(name, cx, cy, sx, sy, sz, rgb, z=None, collide=True, yaw=0.0):
    z = sz / 2 if z is None else z
    c = (f'<collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>'
         if collide else '')
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">{c}
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{col(rgb)}</ambient><diffuse>{col(rgb)}</diffuse></material></visual>
      </link>
    </model>"""


def wall(name, cx, cy, sx, sy):
    """A wall: its body, and a dark cap 2 cm deep on top."""
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
    sx, sy = x1 - x0, y1 - y0
    return f"""
    <model name="{name}"><static>true</static><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>
          <material><script><uri>model://rn_materials/materials/scripts</uri>
            <uri>model://rn_materials/materials/textures</uri><name>RN/{material}</name></script></material>
        </visual>
      </link>
    </model>"""


def tint(name, x0, y0, x1, y1, rgb, z):
    return f"""
    <model name="{name}"><static>true</static><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{x1 - x0:.3f} {y1 - y0:.3f}</size></plane></geometry>
          <material><ambient>{col(rgb)}</ambient><diffuse>{col(rgb)}</diffuse></material></visual>
      </link>
    </model>"""


def disc(name, x, y, r, rgb, z=0.006):
    return f"""
    <model name="{name}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><cylinder><radius>{r}</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>{col(rgb)}</ambient><diffuse>{col(rgb)}</diffuse></material></visual>
      </link>
    </model>"""


def textures(out):
    """The floor materials: AWS's wood and carpet, and two tile patterns drawn here."""
    from PIL import Image, ImageDraw, ImageEnhance
    d = os.path.join(out, 'models', 'rn_materials')
    tex, scripts = os.path.join(d, 'materials', 'textures'), os.path.join(d, 'materials', 'scripts')
    os.makedirs(tex, exist_ok=True)
    os.makedirs(scripts, exist_ok=True)
    aws = F.MODELS
    wood = Image.open(os.path.join(aws, F.PREFIX + 'FloorB_01', 'materials', 'textures', 'aws_FloorB_01.png'))
    wood = wood.convert('RGB').resize((256, 256))
    wood.save(os.path.join(tex, 'rn_wood.png'))
    oak = ImageEnhance.Brightness(ImageEnhance.Color(wood).enhance(1.35)).enhance(0.72)
    oak.save(os.path.join(tex, 'rn_oak.png'))
    carpet = Image.open(os.path.join(aws, F.PREFIX + 'Carpet_01', 'materials', 'textures', 'aws_Carpet_01.png'))
    carpet = carpet.convert('RGB').resize((256, 256))
    r, g, b = carpet.split()
    carpet = Image.merge('RGB', (r.point(lambda v: int(v * 0.85)), g.point(lambda v: int(v * 0.95)),
                                 b.point(lambda v: min(255, int(v * 1.15)))))
    carpet.save(os.path.join(tex, 'rn_carpet.png'))

    def tiles(name, n, base, grout, jitter):
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
            dr.line([(k * s, 0), (k * s, 256)], fill=grout, width=3)
            dr.line([(0, k * s), (256, k * s)], fill=grout, width=3)
        img.save(os.path.join(tex, f'rn_{name}.png'))
    tiles('hall', 2, (196, 186, 170), (150, 140, 126), 8)
    tiles('tile', 4, (226, 230, 232), (168, 176, 182), 4)
    mats = ''
    for name in ('wood', 'oak', 'carpet', 'hall', 'tile'):
        mats += f"""material RN/{name}
{{
  technique
  {{
    pass
    {{
      ambient 0.9 0.9 0.9 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture rn_{name}.png
        filtering anisotropic
        max_anisotropy 8
      }}
    }}
  }}
}}

"""
    open(os.path.join(scripts, 'rn.material'), 'w').write(mats)
    open(os.path.join(d, 'model.config'), 'w').write(
        '<?xml version="1.0"?><model><name>rn_materials</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf><description>floor materials</description></model>\n')
    open(os.path.join(d, 'model.sdf'), 'w').write(
        '<?xml version="1.0"?><sdf version="1.6"><model name="rn_materials"><static>true</static>'
        '<link name="l"/></model></sdf>\n')


def door_model(out):
    """AWS's door, cut down to the walls' height, with a box for the lasers to hit."""
    d = os.path.join(out, 'models', 'rn_door')
    os.makedirs(d, exist_ok=True)
    k = L.WALL_H / 2.23
    open(os.path.join(d, 'model.config'), 'w').write(
        '<?xml version="1.0"?><model><name>rn_door</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf><description>a fire door</description></model>\n')
    open(os.path.join(d, 'model.sdf'), 'w').write(f"""<?xml version="1.0"?>
<sdf version="1.6"><model name="rn_door"><static>true</static>
  <link name="link">
    <collision name="c"><pose>0 0 {L.WALL_H / 2:.3f} 0 0 0</pose>
      <geometry><box><size>{L.LEAF[0]} {L.LEAF[1]} {L.WALL_H}</size></box></geometry></collision>
    <visual name="v"><cast_shadows>false</cast_shadows><geometry><mesh>
      <uri>model://{F.PREFIX}Door_01/meshes/aws_Door_01_visual.DAE</uri><scale>1 1 {k:.4f}</scale></mesh></geometry></visual>
  </link>
</model></sdf>
""")


def world_sdf(state, poses):
    parts = []
    z = 0.002
    for name, x0, y0, x1, y1, material in L.floors():
        parts.append(floor(f'floor_{name}', x0, y0, x1, y1, material, z))
        z += 0.0004
    for k, (d, x0, y0, x1, y1) in enumerate(L.passage_floor()):
        parts.append(tint(f'passage_{d}_{k}', x0, y0, x1, y1, TINT[d], 0.006 + 0.0004 * k))
    for i, (cx, cy, sx, sy) in enumerate(L.walls()):
        parts.append(wall(f'wall_{i}', cx, cy, sx, sy))
    for i, (x0, y0, x1, y1) in enumerate(L.linen()):
        parts.append(box(f'linen_{i}', (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, L.WALL_H - 0.03, LINEN))
    for d, o in state.items():
        x, y, yaw = L.leaf(d, o)
        parts.append(f"""
    <include><uri>model://rn_door</uri><name>door_{d}</name><pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose></include>""")
    for i, (model, x, y, yaw) in enumerate(L.FURNITURE):
        parts.append(f"""
    <include><uri>model://{F.PREFIX}{model}</uri><name>{model.lower()}_{i}</name><static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose></include>""")
    for d in ('w', 'e'):
        parts.append(disc(f'vantage_{d}', *L.VANTAGE[d], 0.2, TINT[d]))
    parts.append(disc('vantage_hatch', *L.VANTAGE['m'], 0.2, TINT['m']))
    parts.append(disc('goal', *L.GOAL, 0.3, GOAL))
    inc = ''
    for r, (x, y, yaw) in poses.items():
        inc += f"""
    <include><uri>model://rn_{r}</uri><name>{r}</name><pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.4f}</pose></include>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="residence">
    <gravity>0 0 -9.8</gravity>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.62 0.62 0.62 1</ambient><background>0.16 0.18 0.22 1</background>
      <shadows>false</shadows><grid>false</grid><origin_visual>false</origin_visual></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 15 0 0 0</pose>
      <diffuse>0.80 0.79 0.76 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>-0.25 0.35 -1</direction></light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material><ambient>0.30 0.33 0.30 1</ambient><diffuse>0.30 0.33 0.30 1</diffuse></material></visual>
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
    dst = os.path.join(out, 'models', f'rn_{r}')
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
    sdf = sdf.replace('<link name="base_link">', f'''<link name="base_link">
      <visual name="cap">
        <pose>-0.032 0 0.215 0 0 0</pose>
        <geometry><cylinder><radius>0.09</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
      </visual>''', 1)
    sdf = re.sub(r'<model name=["\']turtlebot3_burger["\']>', f'<model name="rn_{r}">', sdf)
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->', f'<namespace>/{r}</namespace>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{r}/base_scan</frame_name>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{r}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{r}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{r}/imu_link</frame_name>')
    open(p, 'w').write(sdf)
    cfg = os.path.join(dst, 'model.config')
    text = open(cfg).read()
    open(cfg, 'w').write(re.sub(r'<name>[^<]*</name>', f'<name>rn_{r}</name>', text, count=1))
    for must in ('<max>8.0</max>', f'<namespace>/{r}</namespace>', f'{r}/odom', f'{r}/base_scan',
                 '<visual name="cap">'):
        assert must in sdf, f'{r}: {must} not set'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--act', type=int, required=True, choices=(1, 2, 3))
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--noise', type=float, default=0.01)
    a = ap.parse_args()
    rng = random.Random(a.act * 1000 + a.seed)
    state = WD.world(a.act, a.seed)

    def jitter(p):
        return (p[0] + rng.uniform(-0.05, 0.05), p[1] + rng.uniform(-0.05, 0.05),
                p[2] + rng.uniform(-math.radians(5), math.radians(5)))
    poses = {r: jitter(L.START[r]) for r in WD.ROBOTS[a.act]}
    os.makedirs(a.out, exist_ok=True)
    textures(a.out)
    door_model(a.out)
    for r in poses:
        robot_model(a.out, r, a.noise)
    open(os.path.join(a.out, 'world.sdf'), 'w').write(world_sdf(state, poses))
    json.dump({'act': a.act, 'seed': a.seed, 'open': state, 'tag': WD.tag(state), 'noise': a.noise,
               'robots': list(poses), 'poses': {r: [round(v, 4) for v in p] for r, p in poses.items()}},
              open(os.path.join(a.out, 'world.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
