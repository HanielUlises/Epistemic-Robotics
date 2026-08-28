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
ePlanSys over the warehouse.

What runs, and why each piece is there:

  gazebo           the AWS RoboMaker small warehouse -- the world the
                   RoboticsAcademy exercise itself runs on -- and a TurtleBot3
                   with a laser spawned at shipping. The grid the planner
                   reasons over is rasterised from this world's own collision
                   meshes, so a rack it plans around is the rack the laser
                   sees
  slam_toolbox     builds /map from that laser -- which is what makes the demo
                   honest: a bay the robot has not looked into is unobserved in
                   the grid and undecided in the epistemic model at once
  plansys2         the six-node epistemic bringup: planner through perception
  mu_path_planner  the least fixed point over that map. It decides whether a
                   zone may be driven to at all: while the floor in between is
                   unmeasured the answer is no, and the drive goes and looks
                   instead of setting off
  nav2             does the driving, once something above it has said where.
                   Its costmaps, its local planner and above all its recovery
                   behaviours -- clear, back up, spin -- are what a hand-rolled
                   pursuit loop has no answer for
  action nodes     one per action of the classical domain: drive, look, handle
  mission          seeds the problem, asks for the policy, runs it

The parameters file is written with the EPDDL paths substituted in, because
they are absolute and belong to whoever checked the repository out.
"""

import os
import re
import tempfile

from ament_index_python.packages import (get_package_prefix,
                                          get_package_share_directory)

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_start():
    """Where warehouse_scenario says r1 stands, read from its own header.

    The drive action turns a world box into a map box by subtracting this pose.
    Writing it here as well would be two definitions of one number, and the
    failure when they drift is not a crash -- it is a robot driving to the
    wrong place, which is the expensive kind of wrong.
    """
    header = os.path.join(
        get_package_prefix('warehouse_scenario'),
        'include', 'warehouse_scenario', 'warehouse.hpp')
    with open(header) as handle:
        found = re.search(
            r'kR1Start\s*\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}',
            handle.read())
    if not found:
        raise RuntimeError('no kR1Start in ' + header)
    return found.group(1), found.group(2)


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_demo')

    epddl = os.path.join(pkg, 'epddl')
    domain = os.path.join(epddl, 'warehouse-domain.epddl')
    problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
    mapping = os.path.join(pkg, 'pddl', 'warehouse-mapping.json')
    model = os.path.join(pkg, 'pddl', 'warehouse.pddl')

    with open(os.path.join(pkg, 'params', 'warehouse_demo.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('ACTION_MAPPING', mapping))
    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_warehouse_demo.yaml', delete=False)
    filled.write(params)
    filled.close()

    # The exercise's own world, unmodified. warehouse_scenario's floor plan is
    # rasterised from these very meshes, so the two cannot drift apart.
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    world = os.path.join(
        aws, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')

    # The warehouse's models, and the TurtleBot's, which lives with its own
    # package rather than with either of these.
    models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models + ':' + os.path.join(aws, 'models') + ':' + aws + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''))

    # The AWS models point at their meshes with `file://models/<name>/...`,
    # which is a path relative to a resource root rather than a model:// URI.
    # Without the package share on GAZEBO_RESOURCE_PATH every collision mesh
    # fails to load, and the warehouse comes up as a floor with nothing on it
    # -- a laser that returns nothing, and a demonstration of nothing.
    resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        aws + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', ''))

    gui = LaunchConfiguration('gui')
    plugins = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose'] + plugins + [world],
        output='screen', condition=IfCondition(gui))
    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '--verbose'] + plugins + [world],
        output='screen', condition=UnlessCondition(gui))

    # Gazebo's diff drive plugin publishes odom -> base_footprint and nothing
    # else; the laser reports in base_scan. Without the robot's own static
    # joints SLAM drops every scan it is handed and never produces a map.
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf')
    with open(urdf) as handle:
        robot_description = handle.read().replace('${namespace}', '')

    # The AWS world carries no robot, so the burger is spawned into it, at the
    # shipping floor the scenario calls r1's start.
    #
    # The yaw is zero, and it has to be. Everything downstream -- the zone
    # boxes, the snapshots -- is written in this world's frame, and SLAM fixes
    # the map frame to wherever the robot happened to be standing when it
    # started, orientation included. The drive action turns a world box into a
    # map box by subtracting this pose, which is a translation and nothing
    # else; spawn the robot facing any other way and the map frame is rotated
    # against the world while the zone boxes are not, so every zone lands
    # somewhere the robot has no business driving to and it turns on the spot
    # looking for it. Facing +x costs nothing and keeps the two frames
    # parallel.
    # The pallet, in one of the two bays. Which one is the fact the fleet is
    # uncertain about, and perception is what settles it -- so this pose is the
    # ground truth the demo is *not* told.
    pallet = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_pallet',
        output='screen',
        arguments=['-entity', 'pallet',
                   '-file', os.path.join(pkg, 'worlds', 'pallet.sdf'),
                   # Far enough into the bay that the robot has somewhere to
                   # stand and look at it from, rather than arriving with its
                   # nose against it.
                   '-x', '4.50', '-y', '-2.14', '-z', '0.25'])

    start_x, start_y = robot_start()
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_burger',
        output='screen',
        arguments=['-entity', 'burger',
                   '-file', os.path.join(
                       models, 'turtlebot3_burger', 'model.sdf'),
                   '-x', start_x, '-y', start_y, '-z', '0.01',
                   '-Y', '0.0'])

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 8.0,
            # A sensing action stands the robot still and looks; with the
            # default travel thresholds the mapper stops integrating exactly
            # while it does, and the bay it was dispatched to settle is never
            # observed at all.
            'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0,
            'map_update_interval': 1.0,
        }])

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            'epistemic_state': 'True',
            'epistemic_perception': 'True',
        }.items())

    # Nav2, composed by hand: nav2_bringup is not installed here, and the
    # servers are the whole of what it would have started anyway.
    #
    # No AMCL and no map_server. slam_toolbox is building the map and
    # publishing map -> odom as it goes; a second localiser fighting it over
    # the same transform is how a robot ends up believing it is outside the
    # building.
    nav2_params = os.path.join(pkg, 'params', 'nav2.yaml')

    # The floor plan, served to Nav2 on a topic of its own.
    #
    # This is the split the demo turns on. A warehouse robot is given the
    # building; what it does not have is a record of what it has actually
    # looked at. So Nav2 plans over `/floorplan` -- the rasterised AWS world,
    # the same one warehouse_scenario carries -- and drives straight to a zone
    # instead of feeling its way there, while `/map` stays what slam_toolbox
    # has genuinely measured and stays what mu_path_planner answers "is there
    # a route over floor anyone has seen?" against.
    #
    # Before this, navigation and epistemics were reading the same grid, so a
    # robot that had not measured the far end of the building could not drive
    # there either, and spent the run exploring instead of doing the mission.
    floorplan = os.path.join(
        get_package_share_directory('warehouse_scenario'), 'maps',
        'aws_small_warehouse.yaml')
    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server',
             name='floorplan_server', output='screen',
             parameters=[{'use_sim_time': True,
                          'yaml_filename': floorplan,
                          'topic_name': 'floorplan',
                          'frame_id': 'map'}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[nav2_params],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    # The manager goes up after the servers it manages, not beside them.
    #
    # Started together, it creates its service clients and begins the bringup
    # about a hundred milliseconds later -- before controller_server has
    # advertised its lifecycle services at all. The change_state call fails
    # instantly, the manager aborts the whole bringup, and every server logs a
    # clean configure just afterwards, so the logs show nothing wrong with any
    # of them.
    nav2_manager = [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                          # Wall clock, deliberately, and the only node here
                          # that does not use the simulator's. Bringing five
                          # servers through their lifecycle is not something
                          # that happens in simulated time, and its timeouts
                          # should not stretch or shrink with how fast the
                          # simulator is managing to run.
                          'use_sim_time': False,
                          'autostart': True,
                          # The default is four seconds, and on a cold start
                          # this machine spends longer than that just loading
                          # the warehouse meshes. The manager then declares a
                          # server failed while it is still configuring and
                          # aborts the whole of Nav2 -- with the servers
                          # themselves reporting no error at all, because
                          # nothing was wrong with them.
                          'bond_timeout': 30.0,
                          'attempt_respawn_reconnection': True,
                          'node_names': ['floorplan_server',
                                         'controller_server',
                                         'planner_server',
                                         'behavior_server',
                                         'bt_navigator',
                                         'velocity_smoother']}]),
    ]

    # Started once the simulator and the mapper have had a moment. Nav2's
    # costmaps want a map and a transform tree, and bringing them up into an
    # empty one is the other half of the race above.
    nav2_delayed = TimerAction(period=20.0, actions=nav2_nodes)
    nav2_managed = TimerAction(period=32.0, actions=nav2_manager)

    mu_planner = Node(
        package='mu_path_planner', executable='mu_path_planner_node',
        name='mu_path_planner', output='screen',
        parameters=[{'use_sim_time': True,
                     'free_below': 25,
                     'occupied_above': 65,
                     # 1.5 m over 0.05 m cells, the same reach the offline
                     # scenario resolves its queries with.
                     'sensor_range_cells': 30,
                     # A burger is 0.21 m across over 0.05 m cells, so its
                     # own radius is two cells. Six is well over that, and
                     # deliberately: it also shuts the gaps the building
                     # leaves that are technically wider than the robot and
                     # not worth threading -- 0.43 m between the pallet jack
                     # and the south wall, 0.15 m behind the racks. A route
                     # that goes round costs seconds; one that goes through
                     # ends with the robot wedged. The aisles are 0.92 m and
                     # stay open. kInflationCells in drive_action_node.cpp
                     # has to match: the drive picks where to stand and the
                     # planner decides whether it can be stood in.
                     'inflation_cells': 6}])

    def action(executable, name, action_name, extra=None):
        parameters = [{'action_name': action_name, 'use_sim_time': True}]
        if extra:
            parameters.append(extra)
        return Node(
            package='warehouse_demo', executable=executable, name=name,
            output='screen', parameters=parameters)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo client and rviz, or neither'),
        model_path, resource_path, gazebo_gui, gazebo_headless, spawn, pallet,
        state_publisher, slam, nav2_delayed, nav2_managed, plansys2, mu_planner,
        action('drive_action_node', 'drive_action', 'goto_zone'),
        action('look_action_node', 'look_action', 'look_into'),
        action('handle_action_node', 'pick_action', 'pick_up',
               {'verb': 'lifting the pallet', 'dwell': 4.0}),
        action('handle_action_node', 'drop_action', 'drop_off',
               {'verb': 'unloading at the dock', 'dwell': 4.0}),
        Node(package='warehouse_demo', executable='markers_node',
             name='warehouse_markers', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='warehouse_demo', executable='mission_node',
             name='warehouse_mission', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(gui),
             arguments=['-d', os.path.join(pkg, 'config', 'warehouse_demo.rviz')],
             parameters=[{'use_sim_time': True}],
             output='screen'),
    ])
