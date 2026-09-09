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
The multi-site survey: three sites, two robots that look, one that is told.

Where `survey_xl_launch.py` runs the single-site survey over this floor, this
runs the domain in which contamination has a location. Exactly one of three
sites is contaminated and nobody knows which, so the policy the planner returns
is eight nodes rather than four, six levels deep rather than three, and it
sends two different robots to two different aisles.

    ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py
    ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py contaminated:=a06
    ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py plan_only:=true rmf:=false

`contaminated:=` builds the world with the pallet at that site and nowhere
else, and writes the same answer into the task map's fallback so that the
simulator's ground truth cannot disagree with itself. The three settings take
three different paths through one policy:

    contaminated:=a15   the scout finds it; four actions and the mission ends
    contaminated:=a07   the scout finds nothing, the relay finds it
    contaminated:=a06   neither finds anything, and the team concludes a06

The last is the one worth watching. No robot goes to a25, no laser is ever
pointed at it, and the team finishes knowing its state -- because exactly one
site is contaminated and the other two have been ruled out. The pallet is
standing there the whole time, unobserved, in a corner of the building forty
metres from anything the fleet did.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WEBSOCKET_PORT = 7879

CHANNEL_PREFIX = '/eplansys/channel'

# Every agent of the mission, including the one the goal excludes. `observer`
# runs a radio precisely so that its silence is recorded rather than assumed.
AGENTS = ('scout', 'relay', 'observer')

# The three sites, and where they are. These agree with the pinned waypoints in
# maps/nav_graphs/0.yaml, with the `zones` table of config/warehouse_xl_sites.json
# and with the objects of epddl/survey-sites-problem.epddl. `tools/check_sites.py`
# is what chose them and will say why: each reads open to a laser standing on
# it, and each admits the pallet on floor that no lane crosses.
SITES = {
    'a17': (1.49, 15.75),
    'a31': (15.24, 37.00),
    'a06': (30.24, -1.75),
}

# Where the pallet stands for each site: 0.80 m from the waypoint with its long
# axis pointing at it, so the nearest face is 0.35 m away whichever site is
# used. The side is the one `check_sites.py` admits, and for each of these
# three it is the only one it admits.
PALLETS = {
    'a17': (0.69, 15.75, 0.0),
    'a31': (14.44, 37.00, 0.0),
    'a06': (31.04, -1.75, 0.0),
}

# Which robots do the looking, and which agent each of them is. The bridge pins
# every task to the robot the planner's agent names, so these have to agree
# with the `agents` table of the task map: the robot that senses is the agent
# the model credits with knowing.
SENSING = {'r1': 'relay', 'r2': 'scout'}


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('warehouse_xl_rmf_demo')

    contaminated = LaunchConfiguration('contaminated').perform(context)
    if contaminated not in SITES and contaminated != 'none':
        raise RuntimeError(
            f'contaminated:={contaminated} is not one of '
            f'{sorted(SITES)} or "none". It is which site the pallet stands '
            f'at, and the domain says exactly one of them is contaminated.')

    domain = os.path.join(here, 'epddl', 'survey-sites.epddl')
    problem = os.path.join(here, 'epddl', 'survey-sites-problem.epddl')
    mapping = os.path.join(here, 'pddl', 'survey-sites-mapping.json')
    model = os.path.join(here, 'pddl', 'survey-sites.pddl')

    # The EPDDL problem is the authority on which sites exist. This file only
    # knows where they are, and a disagreement between the two is not
    # detectable at run time by anything that would say so: the mission
    # declares one set of objects, the planner solves over another, and the
    # executor spins on an over-all condition naming a site that is not an
    # object of the problem. It says so fifty times a second and never says
    # why. Checking here turns that into a refusal to start.
    declared = set(re.findall(
        r'\(\s*:objects\s+([^)]*)\)', open(problem).read()))
    named = set()
    for group in declared:
        pending = []
        words = group.split()
        while words:
            word = words.pop(0)
            if word == '-':
                kind = words.pop(0) if words else ''
                if kind == 'site':
                    named.update(pending)
                pending = []
            else:
                pending.append(word)
    if named != set(SITES):
        raise RuntimeError(
            f'{problem} declares the sites {sorted(named)} and this launch '
            f'places pallets at {sorted(SITES)}. One of the two has been '
            f'renamed and the other has not.')

    with open(os.path.join(here, 'params', 'survey_sites.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey_sites.yaml', delete=False)
    filled.write(params)
    filled.close()

    # The task map's `outcomes` is the simulator's ground truth: what a scan
    # would report if there were no sensor to report it. Writing it here from
    # the same argument that places the pallet is what stops the fallback and
    # the world from disagreeing -- a disagreement that would show up as a
    # policy branching correctly on a floor that says otherwise, which is a
    # very quiet way to be wrong.
    import json
    with open(os.path.join(here, 'config', 'warehouse_xl_sites.json')) as handle:
        task_map = json.load(handle)
    task_map['actions']['scan']['outcomes'] = {
        site: ('e-scan-dirty' if site == contaminated else 'e-scan-clean')
        for site in SITES
    }

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_warehouse_xl_sites.json', delete=False)
    json.dump(task_map, map_file, indent=2)
    map_file.close()

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            'epistemic_state': 'True',
        }.items())

    bridge = Node(
        package='eplansys_rmf_bridge',
        executable='rmf_action_node',
        name='eplansys_rmf_bridge',
        output='screen',
        parameters=[{
            'task_map': map_file.name,
            'websocket_port': WEBSOCKET_PORT,
            # Generous, and it has to be. The scout's errand is charger_2 to
            # a15, which is the length and then the width of the building; the
            # single-site demo's 180 s was already the small floor's figure and
            # too short for a thirty-metre transit.
            'task_timeout': 1200.0,
            'channel_prefix': CHANNEL_PREFIX,
        }])

    radios = [
        Node(
            package='eplansys_rmf_demo',
            executable='radio',
            name=f'radio_{agent}',
            output='screen',
            parameters=[{
                'agent': agent,
                'channel_prefix': CHANNEL_PREFIX,
            }])
        for agent in AGENTS
    ]

    # The sensing actions' eyes: one node watching every robot that can look at
    # every site it can be sent to. It is told nothing about the policy, and
    # publishes when a robot it is watching settles at a site it is watching.
    perception = Node(
        package='warehouse_xl_rmf_demo',
        executable='site_perception.py',
        name='site_perception',
        output='screen',
        parameters=[{
            'sites': [f'{name}:{x}:{y}' for name, (x, y) in sorted(SITES.items())],
            'robots': [f'{robot}:/{robot}/scan' for robot in sorted(SENSING)],
            'observation_topic': '/eplansys/observation',
            'site_radius': 0.40,
            'settle': 3,
            'threshold': 0.70,
        }])

    mission = Node(
        package='warehouse_xl_rmf_demo',
        executable='survey_sites_mission',
        name='survey_sites_mission',
        output='screen',
        parameters=[{
            'plan_only': LaunchConfiguration('plan_only'),
            'policy_out': LaunchConfiguration('policy_out'),
            'epddl_problem': problem,
        }])

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            here, 'launch', 'warehouse_xl_fleet.launch.py')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'server_uri': f'ws://localhost:{WEBSOCKET_PORT}',
            # Both agents that scan need a laser, which the single-site fleet
            # gave to one robot only.
            'sensing_robots': ','.join(sorted(SENSING)),
        }.items())

    # What the check asks, derived from the sites rather than written out. The
    # single-site mission's formulas are the node's defaults and name an atom
    # -- `contaminated` -- that this domain does not have: plank grounds
    # `(contaminated ?s)` into one atom per site, `contaminated_a17` and its
    # fellows. Left at the defaults the check reports three formulas UNCHECKED
    # and the mission as failed, on a run that in fact came out exactly as the
    # goal asked.
    check = Node(
        package='eplansys_rmf_demo',
        executable='mission_check',
        name='mission_check',
        output='screen',
        parameters=[{
            'channel_prefix': CHANNEL_PREFIX,
            'must_hold': [f'(Kw {agent} contaminated_{site})'
                          for agent in ('scout', 'relay')
                          for site in sorted(SITES)],
            'must_not_hold': [f'(Kw observer contaminated_{site})'
                              for site in sorted(SITES)],
            # The relay only. It is spoken to on every branch -- the scout's
            # announcement is the first speech act whichever way the first scan
            # goes -- whereas the scout is spoken to on two branches of three.
            # Naming both would make the check a statement about which branch
            # was expected, and a run that took the short one would report a
            # failure for having been short.
            'heard_something': ['relay'],
            'heard_nothing': ['observer'],
        }])

    checking = LaunchConfiguration('check').perform(context).lower() in ('true', '1')

    staged = []
    if checking:
        staged.append(
            RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[check])))

    if LaunchConfiguration('shutdown').perform(context).lower() in ('true', '1'):
        staged.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=check if checking else mission,
                    on_exit=[EmitEvent(event=Shutdown())])))

    return [fleet, plansys2, bridge, perception, *radios, mission, *staged]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'contaminated', default_value='a06',
            description='Which site the pallet stands at: a07, a15, a25, or '
                        'none. The default is the site nobody visits, which '
                        'is the case the domain exists to demonstrate.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo headless and leave rviz out.'),
        DeclareLaunchArgument(
            'policy_out', default_value='',
            description='Write the policy the planner returned to this file, '
                        'so tools/plot_sites.py draws the policy rather than '
                        'a picture of one.'),
        DeclareLaunchArgument(
            'plan_only', default_value='false',
            description='Print the policy and stop. Pair with rmf:=false to '
                        'measure the search without starting a simulator.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('warehouse_xl_rmf_demo'),
                'config', 'warehouse_xl.rviz'),
            description='RViz configuration, passed through to the fleet.'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join('/tmp', 'warehouse_xl_sites.world'),
            description='The world to run. tools/make_site_world.sh builds one '
                        'per value of contaminated:, and they differ by where '
                        'the single pallet stands and by nothing else.'),
        DeclareLaunchArgument(
            'shutdown', default_value='true',
            description='Bring the simulation down when the mission ends. '
                        'false to leave it up, which is what recording needs.'),
        DeclareLaunchArgument(
            'check', default_value='true',
            description='Ask the epistemic state, once the robots have '
                        'stopped, whether the goal actually came out.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the warehouse fleet too. false when it is '
                        'already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
