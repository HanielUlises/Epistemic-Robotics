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
Open-RMF over the larger warehouse.

The world, the navigation graph and the building map all come out of
`tools/layout.py`, so none of them can drift from the others. The building is
`OpenRobotics/Warehouse` from Fuel, thirty by fifty metres, and the shelving
inside it is AWS RoboMaker's -- the hall on its own is an empty box, and an
aisle a robot can only see into from its mouth is the point of the floor.

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
    TimerAction,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Where each robot is put in the world. These agree with the start waypoints
# in config/warehouse_xl_fleet.yaml on purpose: RMF is told the robot is at a
# waypoint and Gazebo puts it at a pose, and if the two disagree the first
# command is issued from somewhere the robot is not.
# The floor of this world is nine raised slabs, each a collision mesh about
# 12 cm thick, and there is no ground plane beneath them. A robot spawned at
# z = 0 arrives inside the slab and wedges: it reports a pose, accepts a path
# and does not move, which from the fleet adapter's side is indistinguishable
# from a robot that is never commanded. Measured on the first attempt, r1 came
# to rest at z = -0.067 with its wheels inside the floor.
#
# Spawning clear of the slab and letting it drop is the fix. A third of a metre
# is enough for the tallest slab and small enough that the drop is not visible
# in a recording.
# The agent the domain has scan. It is the one fitted with a laser.
SENSING_ROBOT = 'r1'

SPAWN_Z = '0.35'

# Seconds to let gzserver load the world before asking it to spawn anything.
SPAWN_DELAY = 45.0

SPAWN = [
    ('r1', -3.35, -9.25, 0.0),
    ('r2', 22.90, -9.25, 0.0),
    ('r3', 11.65, -5.50, 0.0),
]


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('warehouse_xl_rmf_demo')

    world = LaunchConfiguration('world').perform(context)
    nav_graph = os.path.join(here, 'maps', 'nav_graphs', '0.yaml')
    building_map = os.path.join(here, 'maps', 'warehouse_xl.building.yaml')
    fleet_config = os.path.join(here, 'config', 'warehouse_xl_fleet.yaml')

    server_uri = LaunchConfiguration('server_uri').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'

    plugins = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    gazebo = ExecuteProcess(
        cmd=(['gzserver'] if headless else ['gazebo']) + ['--verbose'] +
            plugins + [world],
        output='screen')

    # The robots are TurtleBot3 Waffles carrying RMF's slotcar plugin, which is
    # what RMF actually drives. A stock TurtleBot3 will not do: it ships a
    # differential-drive plugin and its own controllers, which is a different
    # way of being driven and fights this one. models/TurtleBot3Waffle is
    # ROBOTIS's geometry on the skeleton slotcar expects.
    spawns = [
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            name=f'spawn_{name}', output='screen',
            arguments=[
                '-entity', name,
                '-file', os.path.join(
                    get_package_share_directory('warehouse_xl_rmf_demo'),
                    'models', 'TurtleBot3Waffle',
                    # Only the sensing agent carries a laser. Gazebo names a
                    # plugin's node after the plugin, so three robots from one
                    # model file give three nodes called /lds_driver, and the
                    # server reports the collision and then segfaults.
                    'model_lidar.sdf' if name == SENSING_ROBOT else 'model.sdf'),
                '-x', str(x), '-y', str(y), '-z', SPAWN_Z, '-Y', str(yaw),
            ])
        for name, x, y, yaw in SPAWN
    ]

    # RViz and the schedule markers, so a recording shows the graph the fleet
    # routes over beside the world it drives through.
    visualization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rmf_visualization'),
            'visualization.launch.xml')),
        launch_arguments={
            'use_sim_time': 'true',
            'map_name': 'L1',
            'headless': LaunchConfiguration('headless'),
            # The stock rmf.rviz saves a top-down camera over the office demo,
            # centred twenty metres from anything in this world, which renders
            # as an empty grey panel. Only the focal point of a saved view is
            # honoured on load; its angle and scale are not, so the framing
            # here comes from the focal point alone.
            'viz_config_file': os.path.join(
                get_package_share_directory('warehouse_xl_rmf_demo'),
                'config', 'warehouse_xl.rviz'),
        }.items())

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
        name='warehouse_xl_fleet_manager', output='both',
        arguments=['--config_file', fleet_config, '--nav_graph', nav_graph],
        parameters=[{'use_sim_time': True}])

    adapter = Node(
        package='rmf_demos_fleet_adapter', executable='fleet_adapter',
        name='warehouse_xl_fleet_adapter', output='both',
        arguments=['-c', fleet_config, '-n', nav_graph, '-sim'],
        parameters=[{'use_sim_time': True, 'server_uri': server_uri}])

    # The spawns are held back rather than issued with the world. This world
    # is a megabyte of SDF, 169 placed models and nine actors, and gzserver
    # takes tens of seconds to have a spawn service worth calling: issued
    # immediately, all three spawns fail with "Spawn service failed" and the
    # fleet comes up with no robots in it. The delay is the difference between
    # a demonstration and an empty warehouse.
    delayed_spawns = TimerAction(period=SPAWN_DELAY, actions=spawns)

    return ([gazebo, delayed_spawns] +
            [building_map_server, schedule, blockade, manager, adapter,
             visualization])


def generate_launch_description():
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    here = get_package_share_directory('warehouse_xl_rmf_demo')
    rmf_models = os.path.join(
        get_package_share_directory('rmf_demos_assets'), 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'server_uri', default_value='',
            description='Websocket the fleet adapter publishes task states to. '
                        'On Humble it is the only place they go.'),
        # The floor is `dynamic_logistics_warehouse`, which is GPL-2.0 and so
        # is fetched rather than vendored into this Apache-2.0 repository:
        # tools/fetch_third_party.sh clones it beside the workspace.
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                os.path.expanduser('~'), 'Projects', 'Epistemic-Robotics',
                'ros2_ws', 'third_party', 'dynamic_logistics_warehouse',
                'worlds', 'warehouse.world'),
            description='Gazebo world to drive the fleet in.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo without a window.'),

        # The warehouse's own models, and RMF's robots, on one path.
        # The hall from Fuel, the AWS shelving, and RMF's robots, on one path.
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(here, 'models') + ':' +
            os.path.join(aws, 'models') + ':' +
            os.path.join(
                os.path.expanduser('~'), 'Projects', 'Epistemic-Robotics',
                'ros2_ws', 'third_party', 'dynamic_logistics_warehouse',
                'models') + ':' + rmf_models + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '')),

        # The AWS world refers to its collision meshes as `file://models/...`,
        # which resolves against GAZEBO_RESOURCE_PATH and not the model path.
        # Without the package share here every rack loads with no collision
        # geometry, and a robot drives straight through the shelving.
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            aws + ':' + os.path.join(
                os.path.expanduser('~'), 'Projects', 'Epistemic-Robotics',
                'ros2_ws', 'third_party', 'dynamic_logistics_warehouse') +
            ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')),

        # rmf_demos' fleet_manager imports fastapi, and the fastapi that
        # Jammy packages is 0.63, which is written against pydantic 1. A
        # pydantic 2 in ~/.local shadows the apt pydantic 1.8 for every
        # interpreter on the machine, and fastapi then fails to import with
        #
        #   PydanticUserError: Field 'type_' defined on a base class was
        #   overridden by a non-annotated attribute
        #
        # which stops the fleet manager and so stops every robot. Ignoring the
        # user site restores the pair apt installed together. This is set here
        # rather than fixed in the environment because the environment is not
        # ours to change and nothing RMF launches comes from ~/.local.
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),

        # libslotcar.so is what RMF actually drives, and it does not live
        # anywhere Gazebo looks by default.
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            '/opt/ros/humble/lib/rmf_robot_sim_gz_classic_plugins:' +
            os.environ.get('GAZEBO_PLUGIN_PATH', '')),

        OpaqueFunction(function=launch_setup),
    ])
