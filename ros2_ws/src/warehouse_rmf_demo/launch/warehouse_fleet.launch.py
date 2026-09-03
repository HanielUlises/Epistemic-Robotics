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
Open-RMF over the AWS RoboMaker small warehouse.

The rmf_demos worlds are generated from a traffic-editor `.building.yaml`,
which also writes the Gazebo world. This one is not ours to generate: it is the
warehouse the RoboticsAcademy exercise runs in and the one
`scenarios/warehouse` rasterises its occupancy grid from. So the world is
launched unmodified, RMF's robots are spawned into it, and the navigation graph
is written directly in that world's metres by `tools/make_nav_graph.py`.

What that leaves out of the usual rmf_demos bringup is deliberate. There is no
dispatcher, because the bridge pins every task to a named robot and a
`robot_task_request` is handled by that robot's own task manager without a bid;
and there are no door or lift supervisors, because this building has neither.

The building map server stays, though the warehouse has one level and nothing
to open. The slotcar plugin asks it which level a robot is standing on, and
with no answer it publishes no robot state, so RMF never sees the fleet at
all.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Where each robot is put in the world. These agree with the start waypoints in
# config/warehouse_fleet.yaml on purpose: RMF is told the robot is at a
# waypoint and Gazebo puts it at a pose, and if the two disagree the first
# command is issued from somewhere the robot is not.
SPAWN = [
    ('r1', -3.50, -9.30, 0.0),
    # Beside the receiving dock rather than on it: see the note in
    # config/warehouse_fleet.yaml about zones holding several agents and
    # waypoints holding one.
    ('r2', -3.50, 7.70, 0.0),
]


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('warehouse_rmf_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')

    world = os.path.join(
        aws, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    nav_graph = os.path.join(here, 'maps', 'nav_graphs', '0.yaml')
    building_map = os.path.join(here, 'maps', 'aws_small_warehouse.building.yaml')
    fleet_config = os.path.join(here, 'config', 'warehouse_fleet.yaml')

    server_uri = LaunchConfiguration('server_uri').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'

    plugins = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    gazebo = ExecuteProcess(
        cmd=(['gzserver'] if headless else ['gazebo']) + ['--verbose'] +
            plugins + [world],
        output='screen')

    # The robots are rmf_demos' own TinyRobot, whose slotcar plugin is what
    # RMF drives. Nothing else in the world is touched.
    spawns = [
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            name=f'spawn_{name}', output='screen',
            arguments=[
                '-entity', name,
                '-file', os.path.join(
                    get_package_share_directory('rmf_demos_assets'),
                    'models', 'TinyRobot', 'model.sdf'),
                '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw),
            ])
        for name, x, y, yaw in SPAWN
    ]

    building_map_server = Node(
        package='rmf_building_map_tools', executable='building_map_server',
        name='building_map_server', output='both',
        arguments=[building_map],
        parameters=[{'use_sim_time': True}])

    schedule = Node(
        package='rmf_traffic_ros2', executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule_primary', output='both',
        parameters=[{'use_sim_time': True}])

    blockade = Node(
        package='rmf_traffic_ros2', executable='rmf_traffic_blockade',
        output='both', parameters=[{'use_sim_time': True}])

    manager = Node(
        package='rmf_demos_fleet_adapter', executable='fleet_manager',
        name='warehouse_fleet_manager', output='both',
        arguments=['--config_file', fleet_config, '--nav_graph', nav_graph],
        parameters=[{'use_sim_time': True}])

    adapter = Node(
        package='rmf_demos_fleet_adapter', executable='fleet_adapter',
        name='warehouse_fleet_adapter', output='both',
        arguments=['-c', fleet_config, '-n', nav_graph, '-sim'],
        parameters=[{'use_sim_time': True, 'server_uri': server_uri}])

    return ([gazebo] + spawns +
            [building_map_server, schedule, blockade, manager, adapter])


def generate_launch_description():
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    rmf_models = os.path.join(
        get_package_share_directory('rmf_demos_assets'), 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'server_uri', default_value='',
            description='Websocket the fleet adapter publishes task states to. '
                        'On Humble it is the only place they go.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo without a window.'),

        # The warehouse's own models, and RMF's robots, on one path.
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(aws, 'models') + ':' + rmf_models + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '')),

        # The AWS world refers to its collision meshes as `file://models/...`,
        # which resolves against GAZEBO_RESOURCE_PATH and not the model path.
        # Without the package share here every rack loads with no collision
        # geometry, and a robot drives straight through the shelving.
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            aws + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')),

        # libslotcar.so is what RMF actually drives, and it does not live
        # anywhere Gazebo looks by default.
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            '/opt/ros/humble/lib/rmf_robot_sim_gz_classic_plugins:' +
            os.environ.get('GAZEBO_PLUGIN_PATH', '')),

        OpaqueFunction(function=launch_setup),
    ])
