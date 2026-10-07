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
Writes the muddy robots' world: the AWS RoboMaker small warehouse, no roof,
with what this demonstration adds on its floor.

    make_world.py --aws <aws_robomaker_small_warehouse_world share> --out muster.world \\
                  --lamps <dir for the lit-lamp models>

Nothing of the AWS world is moved or removed. Added, all of it visual and
none of it colliding, so that the floor plan rasterised from the world's
collision meshes is still the floor plan:

  the muster ring   painted on the floor about the muster point, just above
                    the AWS floor's surface, which is 3.4 cm up
  the bay           a pad in the north-west room, one place per robot
  the PA            a mast with a horn and a lamp, dark until the supervisor
                    speaks; lighting it spawns a lit lamp over it, so a frame
                    with the lamp lit is a frame after the announcement
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402


def visual_box(name, x, y, z, sx, sy, sz, rgb, yaw=0.0, emissive=False):
    em = f'<emissive>{rgb[0]} {rgb[1]} {rgb[2]} 1</emissive>' if emissive else ''
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{rgb[0]} {rgb[1]} {rgb[2]} 1</ambient>
            <diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 1</diffuse>{em}</material>
        </visual>
      </link>
    </model>
"""


# The top of the AWS floor (GroundB_01's mesh, 12.4 cm, at -9.0 cm); paint
# drawn below it is hidden.
FLOOR = 0.034
PAINT = FLOOR + 0.006


def ring():
    """The muster ring: thirty-two short yellow dashes on a circle, and a
    cross at the centre."""
    out = []
    r = L.RING + 0.55
    for k in range(32):
        a = 2 * math.pi * k / 32
        out.append(visual_box(f'muster_ring_{k:02d}', L.MUSTER[0] + r * math.cos(a),
                              L.MUSTER[1] + r * math.sin(a), PAINT, 0.18, 0.05, 0.004,
                              (0.95, 0.80, 0.10), yaw=a + math.pi / 2))
    for k, yaw in enumerate((0.0, math.pi / 2)):
        out.append(visual_box(f'muster_cross_{k}', L.MUSTER[0], L.MUSTER[1], PAINT, 0.5, 0.05,
                              0.004, (0.95, 0.80, 0.10), yaw=yaw))
    return ''.join(out)


def bay():
    """The calibration bay: a blue pad, and a white line between places."""
    x, y, length, depth = L.BAY_PAD
    out = [visual_box('calibration_bay', x, y, PAINT, length, depth, 0.004, (0.15, 0.35, 0.75))]
    for k in range(len(L.BAY) - 1):
        mid = 0.5 * (L.BAY[k][0] + L.BAY[k + 1][0])
        out.append(visual_box(f'calibration_bay_line_{k}', mid, y, PAINT + 0.004, 0.04, depth, 0.004,
                              (0.95, 0.95, 0.95)))
    return ''.join(out)


PA_TOP = 2.4


def pa_mast():
    x, y = L.PA
    return f"""    <model name="public_address">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="mast">
          <pose>0 0 {PA_TOP / 2:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>{PA_TOP}</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.32 1</ambient><diffuse>0.4 0.4 0.42 1</diffuse></material>
        </visual>
        <visual name="horn">
          <pose>-0.12 0.08 {PA_TOP - 0.25:.2f} 0 1.5708 2.53</pose>
          <geometry><cylinder><radius>0.11</radius><length>0.26</length></cylinder></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
        <visual name="housing">
          <pose>0 0 {PA_TOP + 0.08:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>0.14</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.10 0.10 0.12 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def lit_lamp(rgb, radius=0.16, length=0.18):
    """A lit lamp, spawned over a dark housing when it lights: emissive, with
    no light source, which Gazebo's window would draw as a wireframe."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="lamp">
    <static>true</static>
    <link name="link">
      <visual name="lamp">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>{rgb[0]} {rgb[1]} {rgb[2]} 1</ambient>
          <diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 1</diffuse>
          <emissive>{rgb[0]} {rgb[1]} {rgb[2]} 1</emissive></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def gui_camera():
    x, y, z, pitch, yaw = L.OPENING_SHOT
    return f"""    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{x} {y} {z} 0 {pitch} {yaw}</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--aws', required=True, help='the aws_robomaker_small_warehouse_world share')
    ap.add_argument('--out', required=True)
    ap.add_argument('--lamps', required=True, help='directory for the lit-lamp models')
    args = ap.parse_args()

    source = os.path.join(args.aws, 'worlds', 'no_roof_small_warehouse',
                          'no_roof_small_warehouse.world')
    with open(source) as fh:
        world = fh.read()
    # The AWS world's own camera block goes; ours is put in its place.
    start = world.find('<gui')
    end = world.find('</gui>')
    if start >= 0 and end > start:
        world = world[:start] + world[end + len('</gui>'):]
    added = ('\n    <!-- The muddy robots: muster ring, calibration bay, public address. -->\n'
             + ring() + bay() + pa_mast() + gui_camera())
    world = world.replace('  </world>', added + '  </world>')
    with open(args.out, 'w') as fh:
        fh.write(world)

    os.makedirs(args.lamps, exist_ok=True)
    with open(os.path.join(args.lamps, 'pa_lamp.sdf'), 'w') as fh:
        fh.write(lit_lamp((1.0, 0.72, 0.10)))
    with open(os.path.join(args.lamps, 'bell_lamp.sdf'), 'w') as fh:
        fh.write(lit_lamp((0.25, 0.75, 1.0), radius=0.13, length=0.12))
    print(f'{args.out}: the AWS small warehouse with the muster ring, the bay and the PA')


if __name__ == '__main__':
    main()
