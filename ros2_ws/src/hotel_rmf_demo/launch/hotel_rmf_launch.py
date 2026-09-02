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
The hotel incident, over an Open-RMF fleet.

    ros2 launch hotel_rmf_demo hotel_rmf_launch.py
    ros2 launch hotel_rmf_demo hotel_rmf_launch.py leak:=l3_suite
    ros2 launch hotel_rmf_demo hotel_rmf_launch.py rmf:=false

One command brings up the hotel world, its three fleets, the planning system
and the bridge. `leak:` chooses which suite is actually flooded, and the two
values take the policy down its two different branches with the robots really
driving each one.

The mission is `epddl-workspace/hotel-incident/instances/problem_2.epddl`:
find the leak, shut it off, tell the porter, and do not let the guest learn
which suite it was in. What the planner returns splits the two robots across
the two lifts before it knows the answer, and what happens after the
inspection depends on what the inspection found.
"""

import os
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
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SUITES = ('l2_suite', 'l3_suite')
WEBSOCKET_PORT = 7879


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('hotel_rmf_demo')

    leak = LaunchConfiguration('leak').perform(context)
    if leak not in SUITES:
        raise RuntimeError(
            f'leak:={leak} is not one of {list(SUITES)}. It is which suite the '
            'inspector turns out to find flooded.')

    domain = os.path.join(here, 'epddl', 'hotel-incident.epddl')
    problem = os.path.join(here, 'epddl', 'instances', 'problem_2.epddl')
    mapping = os.path.join(here, 'pddl', 'hotel-mapping.json')
    model = os.path.join(here, 'pddl', 'hotel.pddl')

    with open(os.path.join(here, 'params', 'hotel.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_hotel.yaml', delete=False)
    filled.write(params)
    filled.close()

    # Which suite is flooded is ground truth the simulator holds and the fleet
    # does not. The checked-in map states one arrangement; this rewrites it so
    # that the same demo can be run for either, without the value that decides
    # the branch being buried in a config file.
    with open(os.path.join(here, 'config', 'hotel_rmf.json')) as handle:
        task_map = handle.read()
    if leak == 'l3_suite':
        task_map = (task_map
                    .replace('"l2_suite": "e-inspect-wet"', '"l2_suite": "@WET@"')
                    .replace('"l3_suite": "e-inspect-dry"',
                             '"l3_suite": "e-inspect-wet"')
                    .replace('"@WET@"', '"e-inspect-dry"'))

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_hotel_rmf.json', delete=False)
    map_file.write(task_map)
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
            'task_timeout': 900.0,
        }])

    mission = Node(
        package='hotel_rmf_demo',
        executable='hotel_mission',
        name='hotel_mission',
        output='screen')

    fleet = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            here, 'launch', 'hotel_fleet.launch.xml')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={
            'use_rmf_panel': 'false',
            'headless': LaunchConfiguration('headless'),
            'server_uri': f'ws://localhost:{WEBSOCKET_PORT}',
        }.items())

    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [fleet, plansys2, bridge, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leak', default_value='l2_suite',
            description='Which suite is actually flooded: l2_suite or l3_suite.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo headless and leave rviz out.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the hotel fleets too. false when they are '
                        'already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
