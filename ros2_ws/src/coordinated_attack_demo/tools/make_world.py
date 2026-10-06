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
Writes the coordinated attack's world.

    make_world.py --floor beacon --out /tmp/coordinated_attack_beacon.world
    make_world.py --floor radio  --out /tmp/coordinated_attack_radio.world

The two floors differ by the beacon post in t2 and by nothing else, as the two
EPDDL problems differ by `(beacon)` and by nothing else. Both stands carry a
load: the work order says which of them to lift, and that is not in the world.

The loads are named load_s1 and load_s2 and are moved, when lifted, through
gazebo_ros_state's set_entity_state, which this world loads. The lamp of the
beacon is not in the world: lighting it spawns it, so a frame in which the
lamp is lit is a frame after signal was performed.
"""

import argparse
import importlib.util
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402


def _pass_through_world():
    source = os.path.join(os.path.dirname(L.P.__file__), 'make_world.py')
    spec = importlib.util.spec_from_file_location('pass_through_world', source)
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('layout_for_world', module)
    # Its own `import layout` has to find the pass-through layout.
    saved = sys.modules.get('layout')
    sys.modules['layout'] = L.P
    try:
        spec.loader.exec_module(module)
    finally:
        if saved is not None:
            sys.modules['layout'] = saved
    return module


W = _pass_through_world()

STATE_PLUGIN = """
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
"""


def beacon_post():
    """The post and both tiers, dark. A lit tier is spawned over its housing."""
    x, y = L.BEACON_POST
    r = L.BEACON_POST_RADIUS
    h = min(L.BEACON_TIER_Z.values()) - 0.15
    top = max(L.BEACON_TIER_Z.values()) + 0.17
    tiers = ''.join(f"""        <visual name="housing_{s}">
          <pose>0 0 {z:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.30</length></cylinder></geometry>
          <material><ambient>0.08 0.16 0.10 1</ambient><diffuse>0.10 0.22 0.13 1</diffuse></material>
        </visual>
""" for s, z in sorted(L.BEACON_TIER_Z.items()))
    return f"""    <model name="beacon_post">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="post">
          <pose>0 0 {h / 2:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="post">
          <pose>0 0 {h / 2:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material>
        </visual>
{tiers}        <visual name="cap">
          <pose>0 0 {top:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.14</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def lamp_sdf(stand):
    """The lit tier for one stand, spawned over its housing when the beacon
    signals."""
    z = L.BEACON_TIER_Z[stand]
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="beacon_lamp_{stand}">
    <static>true</static>
    <link name="link">
      <visual name="lamp">
        <pose>0 0 {z:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.145</radius><length>0.31</length></cylinder></geometry>
        <material>
          <ambient>0.2 1.0 0.4 1</ambient><diffuse>0.2 1.0 0.4 1</diffuse>
          <emissive>0.15 0.95 0.35 1</emissive>
        </material>
      </visual>
      <visual name="halo">
        <pose>0 0 {z:.3f} 0 0 0</pose>
        <geometry><sphere><radius>0.42</radius></sphere></geometry>
        <material>
          <ambient>0.2 1.0 0.4 0.25</ambient><diffuse>0.2 1.0 0.4 0.25</diffuse>
          <emissive>0.1 0.6 0.2 1</emissive>
        </material>
        <transparency>0.75</transparency>
      </visual>
      <light name="glow" type="point">
        <pose>0 0 {z:.3f} 0 0 0</pose>
        <diffuse>0.3 1.0 0.45 1</diffuse>
        <specular>0.1 0.4 0.15 1</specular>
        <attenuation><range>6</range><constant>0.4</constant><linear>0.25</linear><quadratic>0.05</quadratic></attenuation>
        <cast_shadows>false</cast_shadows>
      </light>
    </link>
  </model>
</sdf>
"""


def terminal():
    x, y = L.TERMINAL
    w, d = L.TERMINAL_SIZE
    return f"""    <model name="work_order_terminal">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="body">
          <pose>0 0 0.55 0 0 0</pose>
          <geometry><box><size>{w} {d} 1.1</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.55 0 0 0</pose>
          <geometry><box><size>{w} {d} 1.1</size></box></geometry>
          <material><ambient>0.22 0.24 0.28 1</ambient><diffuse>0.28 0.30 0.34 1</diffuse></material>
        </visual>
        <visual name="screen">
          <pose>{w / 2 + 0.01:.3f} 0 1.35 0 -0.35 0</pose>
          <geometry><box><size>0.04 {d - 0.1:.2f} 0.55</size></box></geometry>
          <material>
            <ambient>0.15 0.35 0.75 1</ambient><diffuse>0.15 0.35 0.75 1</diffuse>
            <emissive>0.10 0.25 0.60 1</emissive>
          </material>
        </visual>
      </link>
    </model>
"""


def load(stand):
    x = L.stand_x(stand)
    return (f'    <include>\n'
            f'      <uri>model://{L.LOAD_MODEL}</uri>\n'
            f'      <name>load_{stand}</name>\n'
            f'      <pose>{x:.3f} 0.000 0 0 0 0</pose>\n'
            f'      <static>true</static>\n'
            f'    </include>\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--floor', choices=('beacon', 'radio'), required=True)
    ap.add_argument('--camera', default=','.join(str(v) for v in L.OPENING_SHOT),
                    help='X,Y,Z,PITCH,YAW the Gazebo view opens on')
    ap.add_argument('--out', required=True)
    ap.add_argument('--lamps', help='also write one lit lamp model per stand to this directory, as lamp_<stand>.sdf')
    args = ap.parse_args()

    header = W.HEADER.replace('<world name="pass_through">',
                              '<world name="coordinated_attack">')
    header = header.replace('Generated by tools/make_world.py. Do not edit: edit tools/layout.py.',
                            'Generated by coordinated_attack_demo/tools/make_world.py.')
    parts = [header, STATE_PLUGIN]

    shelves = L.P.block_shelves() + L.P.storage_shelves()
    parts.append(f'\n    <!-- Racking: {len(L.P.block_shelves())} units in the block, '
                 f'{len(L.P.storage_shelves())} in storage. -->\n')
    for i, (x, y) in enumerate(shelves):
        parts.append(W.include(L.SHELF_MODELS[i % 2], f'shelf_{i:03d}', x, y, L.SHELF_YAW))

    parts.append('\n    <!-- Dispatch and storage clutter. -->\n')
    for i, (model, x, y, yaw) in enumerate(L.CLUTTER):
        parts.append(W.include(model, f'clutter_{i:02d}', x, y, yaw))

    parts.append('\n    <!-- The two stands: a load across t1 and one across t3. -->\n')
    for s in sorted(L.STANDS):
        parts.append(load(s))

    parts.append('\n    <!-- The work-order terminal. -->\n')
    parts.append(terminal())

    if args.floor == 'beacon':
        parts.append('\n    <!-- The beacon in t2. Its lamp is spawned when it signals. -->\n')
        parts.append(beacon_post())

    parts.append(W.camera([float(v) for v in args.camera.split(',')]))
    parts.append(W.FOOTER)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as fh:
        fh.write(''.join(parts))
    if args.lamps:
        os.makedirs(args.lamps, exist_ok=True)
        for s in sorted(L.STANDS):
            with open(os.path.join(args.lamps, f'lamp_{s}.sdf'), 'w') as fh:
                fh.write(lamp_sdf(s))
    print(f'{args.out}: {args.floor} floor, {len(shelves)} shelves, '
          f'loads on {" ".join(sorted(L.STANDS))}')


if __name__ == '__main__':
    main()
