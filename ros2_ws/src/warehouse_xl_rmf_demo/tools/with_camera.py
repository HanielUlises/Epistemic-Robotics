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
Writes a copy of a world with the GUI camera placed where we want it.

Gazebo Classic takes its initial view from `<gui><camera>` in the world and
offers no way to set a pose afterwards, so the pose is written into a copy of
the world before it is launched.

There is no `<track_visual>` here, and the reason is worth recording because
it is the obvious thing to reach for. A `<track_visual>` naming a model can
only name one that exists when the world loads, and the robots are spawned
some forty seconds later by a timer. Written without a name it supplies the
defaults for a follow request issued afterwards, but the only tool that issues
one is `gz camera -c user_camera -f <model>`, which advertises on
`~/user_camera/cmd` and waits for a subscriber before publishing. Nothing
subscribes to that topic: it belongs to camera sensors, and the GUI camera is
not one. The command therefore blocks for ever and takes the script that
called it with it.

Following is done instead by `chase_camera`, which computes the pose it wants
and publishes it on `~/user_camera/joy_pose`, a topic the GUI camera does
subscribe to. See `src/chase_camera.cpp`.

The copy is a temporary. The world it is made from is third-party and under a
different licence to this repository, and is neither modified in place nor
redistributed.

    with_camera.py --world in.world --pose "6 -4 8 0 0.7 3.09" --out /tmp/x.world
"""

import argparse
import re


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--pose', required=True,
                    help='x y z roll pitch yaw for the user camera')
    ap.add_argument('--pallet', metavar='X,Y,YAW',
                    help='place a pallet jack. This is the thing the scan is '
                         'looking for: present in the dirty variant of the '
                         'world, absent in the clean one, and nowhere in the '
                         'navigation graph, which is identical for both.')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    text = open(args.world).read()
    gui = (f"    <gui fullscreen='0'>\n"
           f"      <camera name='user_camera'>\n"
           f"        <pose frame=''>{args.pose}</pose>\n"
           f"        <view_controller>orbit</view_controller>\n"
           f"        <projection_type>perspective</projection_type>\n"
           f"      </camera>\n"
           f"    </gui>\n")

    if re.search(r'<gui[ >]', text):
        text = re.sub(r'<gui[^>]*>.*?</gui>', gui.strip(), text, count=1, flags=re.S)
    else:
        # Immediately after the opening <world> tag, which is where a gui block
        # belongs and where Gazebo will look for it.
        text = re.sub(r'(<world[^>]*>\n)', r'\1' + gui, text, count=1)

    if args.pallet:
        px, py, pyaw = (float(v) for v in args.pallet.split(','))
        # Explicit geometry rather than an <include> of an AWS model. The
        # asset pack shipped with this world carries absolute mesh paths from
        # its author's machine, so an included model can load with no collision
        # at all: it renders and the laser passes through it. A box cannot fail
        # that way, and what the sensing action needs is something present, not
        # something ornamental.
        pallet = f"""    <model name='scan_target'>
      <static>true</static>
      <pose>{px} {py} 0.45 0 0 {pyaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>0.9 0.5 0.9</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>0.9 0.5 0.9</size></box></geometry>
          <material>
            <ambient>0.55 0.35 0.12 1</ambient>
            <diffuse>0.72 0.47 0.16 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        text = re.sub(r'(</world>)', pallet + r'\1', text, count=1)

    open(args.out, 'w').write(text)
    print(f'{args.out}: camera at {args.pose}'
          + (f', pallet at {args.pallet}' if args.pallet else ', no pallet'))


if __name__ == '__main__':
    main()
