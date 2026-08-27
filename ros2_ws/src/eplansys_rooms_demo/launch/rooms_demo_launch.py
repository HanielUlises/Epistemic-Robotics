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
ePlanSys over the six-room building.

What runs, and why each piece is there:

  gazebo         the building and a TurtleBot3 with a laser
  slam_toolbox   builds /map from that laser -- so a room the robot has not
                 entered is genuinely unobserved, rather than unobserved by
                 arrangement. This is what makes the demo honest: the epistemic
                 state's uncertainty and the map's unknown cells are the same
                 fact, seen from two sides
  plansys2       the six-node epistemic bringup, planner through perception
  action nodes   one per action of the policy: two that drive, two that look

The parameters file is written with the EPDDL paths substituted in, because
they are absolute and belong to whoever checked the repository out.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('eplansys_rooms_demo')

    # Installed from epddl-workspace/building-rooms, which remains the domain's
    # only source. Absolute, because both the planner and the epistemic state
    # are given the pair by path.
    epddl = os.path.join(pkg, 'epddl')
    domain = os.path.join(epddl, 'building-rooms.epddl')
    problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
    mapping = os.path.join(pkg, 'pddl', 'building-rooms-mapping.json')
    model = os.path.join(pkg, 'pddl', 'building-rooms.pddl')

    # The shipped parameters name the EPDDL by placeholder; fill them in.
    with open(os.path.join(pkg, 'params', 'rooms_demo.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('ACTION_MAPPING', mapping))
    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_rooms_demo.yaml', delete=False)
    filled.write(params)
    filled.close()

    world = os.path.join(pkg, 'worlds', 'building_rooms_r1.sdf')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', world],
        output='screen')

    # Gazebo's diff drive plugin publishes odom -> base_footprint, and nothing
    # else. The laser reports in base_scan, so without the robot's own static
    # joints SLAM drops every scan it is handed -- "Message Filter dropping
    # message: frame 'base_scan'" -- and never produces a map.
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf')
    with open(urdf) as handle:
        robot_description = handle.read()

    # The shipped description carries an unexpanded ${namespace} on every link,
    # which the TurtleBot's own launch files substitute. Published as-is, the
    # frames come out named "${namespace}base_footprint" and the laser's frame
    # connects to nothing, so SLAM fills its queue and drops every scan. This
    # demo runs one robot in the global namespace.
    robot_description = robot_description.replace('${namespace}', '')

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            # Wide enough to see through a doorway and down a room, which is
            # what the sensing actions depend on.
            'max_laser_range': 8.0,
            # A sensing action stands the robot still, and slam_toolbox
            # processes a scan only once the base has moved this far. Left at
            # the default, the map stops growing exactly when the robot is
            # trying to look at something, and the region it was sent to settle
            # is never observed at all.
            # Zero, so a scan is integrated whether or not the base has moved.
            # A sensing action stands the robot still and looks; with a
            # non-zero threshold the mapper stops exactly while it does, and
            # the region the action was dispatched to settle is never observed
            # at all. This is what a sensing action needs from a mapper.
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

    # One node per PDDL action, not per policy step: the executor dispatches on
    # the action name, and which door a particular goto_door means is in its
    # arguments.
    def action(executable, name, action_name, extra=None):
        parameters = [{'action_name': action_name, 'use_sim_time': True}]
        if extra:
            parameters.append(extra)
        return Node(
            package='eplansys_rooms_demo',
            executable=executable,
            name=name,
            output='screen',
            parameters=parameters)

    # The routes, as flat [x1,y1,x2,y2,...] in the map frame. The node's own
    # defaults say the same thing; naming them here is what lets a different
    # building be driven without rebuilding.
    doors = {
        'route_door3': [-6.0, 6.0, -6.0, 1.5, -2.0, 1.5, 1.0, 0.45, 3.0, 0.45, 4.9, 0.45, 5.7, 1.3],
        'route_door6': [4.9, 0.45, 3.0, 0.45, 3.0, -0.45, 4.9, -0.45, 6.2, -1.7],
    }

    return LaunchDescription([
        gazebo,
        state_publisher,
        slam,
        plansys2,
        action('drive_action_node', 'drive_action', 'goto_door', doors),
        action('look_action_node', 'look_action', 'look_into'),
        Node(
            package='eplansys_rooms_demo', executable='caption_node',
            name='caption', output='screen',
            parameters=[{'use_sim_time': True, 'x': 0.0, 'top': 9.5}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'rooms_demo.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
