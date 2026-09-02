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
The warehouse mission, over an Open-RMF fleet, in the AWS small warehouse.

    ros2 launch warehouse_rmf_demo warehouse_rmf_launch.py
    ros2 launch warehouse_rmf_demo warehouse_rmf_launch.py pallet:=bay3

This is `epddl-workspace/robot-warehouse` executed unchanged. The domain, the
instance and the policy are the ones `validate.sh` checks and
`ros2 launch warehouse_demo warehouse_demo_launch.py` already runs; what
changes is who drives the robots. There they are TurtleBots under Nav2, and
here they are RMF's, in the same building.

`pallet:` chooses which aisle the pallet actually stands in. Nobody in the
domain is told, and the two values take the policy down its two branches:
`bay2` is settled by the first look, `bay3` costs r1 the walk back out to the
lane and into the other aisle.
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BAYS = ('bay2', 'bay3')
WEBSOCKET_PORT = 7879


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('warehouse_rmf_demo')
    classical = get_package_share_directory('warehouse_demo')

    pallet = LaunchConfiguration('pallet').perform(context)
    if pallet not in BAYS:
        raise RuntimeError(
            f'pallet:={pallet} is not one of {list(BAYS)}. It is which aisle '
            'the pallet actually stands in.')

    domain = os.path.join(here, 'epddl', 'warehouse-domain.epddl')
    problem = os.path.join(here, 'epddl', 'warehouse-problem.epddl')
    mapping = os.path.join(here, 'pddl', 'warehouse-mapping.json')

    # The classical half is warehouse_demo's, unchanged. What a robot
    # physically does is the same whether Nav2 or RMF is doing it.
    model = os.path.join(classical, 'pddl', 'warehouse.pddl')

    with open(os.path.join(here, 'params', 'warehouse.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_warehouse.yaml', delete=False)
    filled.write(params)
    filled.close()

    # Which aisle holds the pallet is ground truth the simulator has and the
    # fleet does not. The checked-in map states one arrangement; this rewrites
    # it so the same demo runs for either without the value that decides the
    # branch being buried in a config file.
    with open(os.path.join(here, 'config', 'warehouse_rmf.json')) as handle:
        task_map = handle.read()
    if pallet == 'bay3':
        task_map = (task_map
                    .replace('"bay2": "e-inspect-found"', '"bay2": "@FOUND@"')
                    .replace('"bay3": "e-inspect-empty"', '"bay3": "e-inspect-found"')
                    .replace('"@FOUND@"', '"e-inspect-empty"'))

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_warehouse_rmf.json', delete=False)
    map_file.write(task_map)
    map_file.close()

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(here, 'launch', 'warehouse_fleet.launch.py')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'server_uri': f'ws://localhost:{WEBSOCKET_PORT}',
        }.items())

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
        package='warehouse_rmf_demo',
        executable='warehouse_mission',
        name='warehouse_mission',
        output='screen')

    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [fleet, plansys2, bridge, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pallet', default_value='bay2',
            description='Which aisle the pallet actually stands in: bay2 or bay3.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo without a window.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the warehouse fleet too. false when it is '
                        'already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
