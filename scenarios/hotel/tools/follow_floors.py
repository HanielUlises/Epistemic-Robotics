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
Shows the floor the mission is acting on, for the duration it acts on it.

The camera looks straight down at the building, and nothing in this map models
a ceiling, so what the camera sees is whichever floor is left showing. The
world carries rmf_building_sim's `toggle_floors` plugin, whose L1/L2/L3
buttons are the only way to choose. They are buttons, so this clicks them.

Which floor to show is read from the run log rather than from a robot's
height: the film is about the epistemic action, so the view follows the agent
that is acting. `deploy` is ignored because it sends two robots to two floors
at once and neither is doing anything yet but riding.

    follow_floors.py --log run.log --display :99

Positions are where the plugin draws its buttons in a Gazebo window at the
origin, measured off a recording.
"""

import argparse
import re
import subprocess
import time

DISPATCH = re.compile(
    r'eplansys_rmf_bridge\]: '
    r'(?P<action>goto_zone|look_into|shut_valve|pick_up|drop_off): '
    r'\w+ -> [\w-]+/[\w-]+ heading for (?P<place>\w+)')

FLOOR_OF_ZONE = {
    'lobby': 'L1',
    'L2_master_suite': 'L2',
    'L3_master_suite': 'L3',
}

BUTTON = {'L1': (288, 92), 'L2': (330, 92), 'L3': (372, 92)}
ORDER = ['L1', 'L2', 'L3']

# Each button toggles one floor, and the world starts with all three drawn.
# Seen from above that means L3, whatever else is showing, so reaching a floor
# is a matter of hiding the ones stacked on top of it.
visible = {name: True for name in ORDER}


def click(floor, display):
    x, y = BUTTON[floor]
    subprocess.run(['xdotool', 'mousemove', str(x), str(y), 'click', '1'],
                   env={'DISPLAY': display, 'PATH': '/usr/bin:/bin'},
                   check=False)
    # Leave the pointer out of frame; x11grab draws it otherwise.
    subprocess.run(['xdotool', 'mousemove', '1900', '1060'],
                   env={'DISPLAY': display, 'PATH': '/usr/bin:/bin'},
                   check=False)
    time.sleep(0.4)


def show(floor, display):
    """Leave `floor` the highest one drawn."""
    wanted = ORDER.index(floor)
    for index, name in enumerate(ORDER):
        want = index <= wanted
        if visible[name] != want:
            click(name, display)
            visible[name] = want


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', required=True)
    parser.add_argument('--display', default=':99')
    parser.add_argument('--start', default='L1')
    args = parser.parse_args()

    showing = args.start
    show(showing, args.display)
    print(f'showing {showing}', flush=True)

    handle = None
    for _ in range(120):
        try:
            handle = open(args.log, errors='replace')
            break
        except FileNotFoundError:
            time.sleep(1)
    if handle is None:
        return

    handle.seek(0, 2)
    while True:
        line = handle.readline()
        if not line:
            time.sleep(0.4)
            continue
        match = DISPATCH.search(line)
        if not match:
            continue
        floor = FLOOR_OF_ZONE.get(match.group('place'))
        if floor and floor != showing:
            showing = floor
            show(showing, args.display)
            print(f'showing {showing} for {match.group("action")}', flush=True)


if __name__ == '__main__':
    main()
