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
The pass-through mission: two scouts, a carrier, and three bays of which one
is open.

    ros2 launch pass_through_demo pass_through_launch.py              # open:=t2
    ros2 launch pass_through_demo pass_through_launch.py open:=t3
    ros2 launch pass_through_demo pass_through_launch.py gui:=false rviz:=false

What runs, per robot:

  gazebo           a Waffle in its own namespace, spawned from a copy of the
                   model with its laser set to its role
  slam_toolbox     that robot's own map, from that robot's own laser, and
                   nothing else: remapped off /map so three mappers do not
                   write one map, which would make every observation public
  knowledge_map    the SLAM map on the floor plan's grid, and the same fused
                   with every map the robot has been sent

and once for the mission:

  plansys2        ePlanSys, monolithic: the EPDDL grounded by plank, the policy
                  found by Aletheia, executed by the epistemic behaviour tree,
                  and the model advanced by the epistemic state
  survey_action   one per scout: drive to a bay, read it off the scout's own map
  share_action    one per announcement kind: fuse the sender's map into the
                  receiver's, and check the map against the announcement
  cross_action    the carrier: a route that exists only through bays the
                  carrier knows to be open
  knowledge_view  the model and the maps, drawn for RViz
  mission         asks for the policy and runs it

`open:=` builds the world with loads in the other two bays. The three worlds
take three different leaves of one policy; open:=t2 is the one in which the
carrier crosses a bay no robot has looked into.
"""

import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def flat_boxes(boxes):
    return [float(v) for b in boxes for v in b]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('pass_through_demo')
    sys.path.insert(0, os.path.join(share, 'tools'))
    import layout as L   # noqa: E402
    from robot_sdf import robot_sdf   # noqa: E402

    open_bay = LaunchConfiguration('open').perform(context)
    if open_bay not in L.TUNNEL_BAYS:
        raise RuntimeError(f'open:={open_bay} is not one of {sorted(L.TUNNEL_BAYS)}')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    # The world, built now from layout.py, so the run cannot be of a world that
    # has drifted from the floor plan the robots are given.
    world = LaunchConfiguration('world').perform(context) or os.path.join(
        tempfile.gettempdir(), f'pass_through_{open_bay}.world')
    subprocess.run([sys.executable, os.path.join(share, 'tools', 'make_world.py'),
                    '--open', open_bay, '--out', world], check=True)

    floorplan = os.path.join(share, 'maps', 'floorplan.yaml')
    domain = os.path.join(share, 'epddl', 'pass-through.epddl')
    problem = os.path.join(share, 'epddl', 'pass-through-problem.epddl')
    mapping = os.path.join(share, 'pddl', 'pass-through-mapping.json')
    model = os.path.join(share, 'pddl', 'pass-through.pddl')

    with open(os.path.join(share, 'params', 'pass_through.yaml')) as fh:
        params = (fh.read().replace('EPDDL_DOMAIN', domain)
                  .replace('EPDDL_PROBLEM', problem).replace('MAPPING_FILE', mapping))
    filled = tempfile.NamedTemporaryFile('w', suffix='_pass_through.yaml', delete=False)
    filled.write(params)
    filled.close()

    bays = sorted(L.TUNNEL_BAYS)
    bay_boxes = flat_boxes([L.tunnel_box(t) for t in bays])
    bay_regions = flat_boxes([L.tunnel_region(t) for t in bays])
    bay_mouths = [float(v) for t in bays for v in L.tunnel_mouth(t)]
    bay_parking = [float(v) for t in bays for v in L.tunnel_parking(t)]
    agents = list(L.ROBOTS)
    namespaces = [L.ROBOTS[a][0] for a in agents]

    # The regions perception reads are the bay interiors less the margin; the
    # boxes the safe set lifts are the whole interior. Two lists, because they
    # answer two questions: "is this bay decided" and "may I drive here".
    common = {'floorplan': floorplan, 'bays': bays}

    gazebo = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen')]
    if gui:
        gazebo.append(ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen'))

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf) as fh:
        urdf_text = fh.read()

    per_robot = []
    for k, agent in enumerate(agents):
        ns, x, y, rng, samples, colour = L.ROBOTS[agent]
        sdf = tempfile.NamedTemporaryFile('w', suffix=f'_{ns}.sdf', delete=False)
        sdf.write(robot_sdf(ns, rng, samples, colour=colour))
        sdf.close()

        # One at a time: /spawn_entity is a single service, and a robot spawned
        # while the world is still loading its meshes can come up without its
        # plugins.
        at = 8.0 + 4.0 * k
        per_robot.append(TimerAction(period=at, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{ns}',
                 output='screen',
                 arguments=['-entity', ns, '-file', sdf.name,
                            '-x', str(x), '-y', str(y), '-z', '0.01']),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': True,
                              'robot_description': urdf_text.replace('${namespace}', ns + '/')}]),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', namespace=ns, output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'odom_frame': f'{ns}/odom',
                     'map_frame': 'map',
                     'base_frame': f'{ns}/base_footprint',
                     'scan_topic': f'/{ns}/scan',
                     'mode': 'mapping',
                     # The floor plan's resolution, so a SLAM cell and a plan
                     # cell are the same size and resampling is a translation.
                     'resolution': L.RESOLUTION,
                     'max_laser_range': float(rng),
                     # A scout reads a bay standing still. With the default
                     # travel thresholds the mapper stops integrating exactly
                     # then, and the bay it was sent to read is never observed.
                     'minimum_travel_distance': 0.0,
                     'minimum_travel_heading': 0.0,
                     'map_update_interval': 1.0,
                     'transform_publish_period': 0.05,
                 }],
                 remappings=[('/map', f'/{ns}/map'),
                             ('/map_metadata', f'/{ns}/map_metadata')]),
            Node(package='pass_through_demo', executable='knowledge_map',
                 name='knowledge_map', namespace=ns, output='screen',
                 parameters=[{**common, 'agent': agent, 'bay_boxes': bay_regions}]),
        ]))

    performers = []
    for agent, bays_covered in L.COVERS.items():
        if not bays_covered:
            continue
        ns = L.ROBOTS[agent][0]
        performers.append(Node(
            package='pass_through_demo', executable='survey_action',
            output='screen', arguments=['--agent', agent],
            parameters=[{**common, 'ns': ns, 'bay_boxes': bay_boxes, 'bay_regions': bay_regions,
                         'bay_mouths': bay_mouths, 'bay_parking': bay_parking,
                         'action_name': 'survey',
                         'specialized_arguments': [agent, ''], 'rate': 10.0}]))
    for kind in ('open', 'shut'):
        performers.append(Node(
            package='pass_through_demo', executable='share_action',
            output='screen', arguments=['--kind', kind],
            parameters=[{'agents': agents, 'namespaces': namespaces,
                         'action_name': f'tell_{kind}', 'rate': 10.0,
                         'transfer_seconds': 4.0}]))
    carrier = [a for a in agents if not L.COVERS[a]][0]
    performers.append(Node(
        package='pass_through_demo', executable='cross_action',
        output='screen', arguments=['--agent', carrier],
        parameters=[{**common, 'ns': L.ROBOTS[carrier][0], 'bay_boxes': bay_boxes,
                     'dock': [L.DOCK[0], L.DOCK[1], L.DOCK_RADIUS],
                     'action_name': 'cross', 'specialized_arguments': [carrier, ''],
                     'rate': 10.0}]))

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': model, 'params_file': filled.name,
                          'epistemic_state': 'True'}.items())

    colours = [c for a in agents for c in L.ROBOTS[a][5]]
    view = Node(
        package='pass_through_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'floorplan': floorplan, 'agents': agents, 'namespaces': namespaces,
                     'colours': colours, 'bays': bays, 'bay_boxes': bay_boxes,
                     'scouts': [a for a in agents if L.COVERS[a]]}])

    director = Node(
        package='pass_through_demo', executable='camera_director.py', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{'agents': agents, 'namespaces': namespaces,
                     'follow_file': LaunchConfiguration('follow_file'),
                     'initial': 'pose ' + ' '.join(str(v) for v in L.OPENING_SHOT),
                     'shots': [f'{a}=pose ' + ' '.join(str(v) for v in L.mouth_shot(L.COVERS[a][0]))
                               for a in agents if L.COVERS[a]]}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}])

    mission = Node(
        package='pass_through_demo', executable='pass_through_mission', output='screen',
        parameters=[{'epddl_problem': problem,
                     'policy_out': LaunchConfiguration('policy_out'),
                     'plan_only': LaunchConfiguration('plan_only'),
                     'hold': float(LaunchConfiguration('hold').perform(context))}])

    staged = []
    if LaunchConfiguration('shutdown').perform(context).lower() == 'true':
        staged.append(RegisterEventHandler(OnProcessExit(
            target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))

    # The mission waits for the planning system itself; it is held back only
    # until the robots and their mappers are up, so the policy does not start
    # before there is anything to drive.
    return (gazebo + per_robot + [plansys2, view, director, rviz] +
            [TimerAction(period=25.0, actions=performers)] +
            [TimerAction(period=float(LaunchConfiguration('start_after').perform(context)),
                         actions=[mission])] + staged)


def generate_launch_description():
    share = get_package_share_directory('pass_through_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    xl = get_package_share_directory('warehouse_xl_rmf_demo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('open', default_value='t2',
                              description='The one bay left open: t1, t2 or t3.'),
        DeclareLaunchArgument('world', default_value='',
                              description='Use this world instead of building one.'),
        DeclareLaunchArgument('gui', default_value='true', description='gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Write the acting robot to follow_file.'),
        DeclareLaunchArgument('follow_file', default_value='/tmp/pass_through_follow'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'pass_through.rviz')),
        DeclareLaunchArgument('policy_out', default_value=''),
        DeclareLaunchArgument('plan_only', default_value='false'),
        DeclareLaunchArgument('hold', default_value='0.0',
                              description='Seconds the mission waits before planning.'),
        DeclareLaunchArgument('start_after', default_value='40.0',
                              description='Seconds before the mission node starts.'),
        DeclareLaunchArgument('shutdown', default_value='false',
                              description='Bring everything down when the mission ends.'),

        # The hall is the Fuel model vendored in warehouse_xl_rmf_demo, the
        # racking and clutter are AWS RoboMaker's, the robots ROBOTIS's.
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join([
            os.path.join(xl, 'models'), os.path.join(aws, 'models'), os.path.join(tb3, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', '')])),
        # The AWS models name their meshes `file://models/...`, which resolves
        # against the resource path; without the package share here every rack
        # loads without collision and the lasers see through them.
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', ':'.join([
            aws, '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        OpaqueFunction(function=setup),
    ])
