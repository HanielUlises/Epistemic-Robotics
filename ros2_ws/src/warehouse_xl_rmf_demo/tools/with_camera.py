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

Gazebo Classic takes its initial view from `<gui><camera>` in the world, and
offers no way to set a pose afterwards: `gz camera` can follow a model, which
puts the camera on top of it and frames crates rather than robots, and nothing
else. So the pose is written into a copy of the world before it is launched.

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

    open(args.out, 'w').write(text)
    print(f'{args.out}: camera at {args.pose}')


if __name__ == '__main__':
    main()
