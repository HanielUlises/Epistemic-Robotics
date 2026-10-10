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
The stale-maps shift: eight robots, a forklift, and one fleet's way of
sharing maps.

    ros2 launch stale_map_demo stale_maps_launch.py                    # fleet:=epistemic
    ros2 launch stale_map_demo stale_maps_launch.py fleet:=fuse
    ros2 launch stale_map_demo stale_maps_launch.py fleet:=overwrite
    ros2 launch stale_map_demo stale_maps_launch.py fleet:=recency
    ros2 launch stale_map_demo stale_maps_launch.py gui:=false rviz:=false

What runs, per robot:

  gazebo        a Waffle in its own namespace, with a twelve-metre laser
  living_map    the shift map, kept current by that robot's laser, and merged
                with the maps it is sent by the fleet's rule

and once:

  crew          the forklift, the map exchanges and the hauls, for every fleet
  view          every robot's map of the bays, against the floor, for RViz
  mission       runs the fleet, and judges it against what tools/fleets.py
                says it will do

and on the epistemic fleet only:

  plansys2      ePlanSys: the radio floor's EPDDL grounded by plank, the
                policy found by Aletheia, executed by the epistemic behaviour
                tree, and the model advanced by the epistemic state
  relay_action  one per action, handing it to the crew

The four fleets differ in how a robot merges a map it is sent (the living
maps' rule) and in which maps are sent (the policy, or every map of every bay
to every robot). Nothing else differs between them.
"""

import json
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

# See pass_through_launch.py: the action nodes are built against the
# cascade_lifecycle /opt/ros ships, and must load that one.
SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'

RULES = {'epistemic': 'recency', 'fuse': 'confidence', 'overwrite': 'overwrite',
         'recency': 'recency'}


def matching_cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def flat(boxes):
    return [float(v) for b in boxes for v in b]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('stale_map_demo')
    sys.path.insert(0, os.path.join(share, 'tools'))
    import layout as L   # noqa: E402
    pt_tools = os.path.join(get_package_share_directory('pass_through_demo'), 'tools')
    sys.path.insert(0, pt_tools)
    from robot_sdf import robot_sdf   # noqa: E402

    fleet = LaunchConfiguration('fleet').perform(context)
    if fleet not in RULES:
        raise RuntimeError(f'fleet:={fleet} is not one of {sorted(RULES)}')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    epistemic = fleet == 'epistemic'

    world = os.path.join(tempfile.gettempdir(), 'stale_maps.world')
    subprocess.run([sys.executable, os.path.join(share, 'tools', 'make_world.py'),
                    '--out', world], check=True)

    floorplan = os.path.join(share, 'maps', 'floorplan.yaml')
    shift_map = os.path.join(share, 'maps', 'shift_map.yaml')
    with open(os.path.join(share, 'config', 'fleets.json')) as fh:
        expected = json.load(fh)[fleet]
    with open(os.path.join(share, 'epddl', 'fleet.json')) as fh:
        sees = json.load(fh)['sees']

    agents = list(L.ROBOTS)
    bays = list(L.BAYS)
    bay_boxes = flat([L.bay_box(t) for t in bays])
    bay_regions = flat([L.bay_region(t) for t in bays])
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    load_sdf = os.path.join(aws, 'models', L.LOAD_MODEL, 'model.sdf')
    changes = [f'{kind} {t}' for t, kind in sorted(L.CHANGES.items(), key=lambda kv: kv[0] != 't3')]

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
        x, y, colour = L.ROBOTS[agent]
        sdf = tempfile.NamedTemporaryFile('w', suffix=f'_{agent}.sdf', delete=False)
        sdf.write(robot_sdf(agent, L.LIDAR_RANGE, L.LIDAR_SAMPLES, colour=colour))
        sdf.close()
        # One at a time: /spawn_entity is a single service, and a robot
        # spawned while the world is still loading can come up without its
        # plugins.
        per_robot.append(TimerAction(period=8.0 + 3.0 * k, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{agent}',
                 output='screen',
                 arguments=['-entity', agent, '-file', sdf.name,
                            '-x', str(x), '-y', str(y), '-z', '0.01']),
            # The odometry is in the world frame, so the map frame is the
            # odometry frame and the transform between them is the identity.
            # It is published only so RViz can place the scans.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'map_to_{agent}', output='log',
                 arguments=['--frame-id', 'map', '--child-frame-id', f'{agent}/odom']),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=agent, output='log',
                 parameters=[{'use_sim_time': True,
                              'robot_description': urdf_text.replace('${namespace}', agent + '/')}]),
            Node(package='stale_map_demo', executable='living_map', name='living_map',
                 namespace=agent, output='screen',
                 parameters=[{'use_sim_time': True, 'agent': agent, 'shift_map': shift_map,
                              'bays': bays, 'bay_regions': bay_regions,
                              'rule': RULES[fleet], 'others': agents,
                              'lidar_range': float(L.LIDAR_RANGE)}]),
        ]))

    crew = Node(
        package='stale_map_demo', executable='crew', output='screen',
        parameters=[{'use_sim_time': True, 'agents': agents, 'haulers': list(L.HAULERS),
                     'drops': [float(v) for h in L.HAULERS for v in L.DROPS[h]],
                     'drop_radius': L.DROP_RADIUS, 'floorplan': floorplan,
                     'bays': bays, 'bay_boxes': bay_boxes,
                     'bay_centres': [float(v) for t in bays for v in L.bay_centre(t)],
                     'load_sdf': load_sdf,
                     'observer_bays': list(L.CHANGES),
                     'observer_lists': [' '.join(sees[t]) for t in L.CHANGES],
                     'transfer_seconds': 4.0 if epistemic else 0.3}])

    view = Node(
        package='stale_map_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'floorplan': floorplan, 'agents': agents, 'haulers': list(L.HAULERS),
                     'colours': [c for a in agents for c in L.ROBOTS[a][2]],
                     'bays': bays, 'load_boxes': flat([L.load_box(t) for t in bays]),
                     'shift_blocked': list(L.SHIFT_BLOCKED), 'fleet': fleet}])

    shots = [f'forklift {t}=' + L.shot_text(L.bay_shot(t)) for t in L.CHANGES]
    director = Node(
        package='stale_map_demo', executable='camera_director.py', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{'agents': agents, 'namespaces': agents,
                     'follow_file': LaunchConfiguration('follow_file'),
                     'initial': L.shot_text(L.OPENING_SHOT), 'shots': shots}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}])

    problem = os.path.join(share, 'epddl', 'radio.epddl')
    mission = Node(
        package='stale_map_demo', executable='stale_maps_mission', output='screen',
        arguments=['--ros-args', '-p', 'plan_solver_timeout:=150.0'],
        parameters=[{'fleet': fleet, 'agents': agents, 'haulers': list(L.HAULERS),
                     'bays': bays, 'blocked_after': L.after_forklift(),
                     'changes': changes,
                     'expected_delivered': expected['delivered'] or [''],
                     'expected_sends': int(expected['sends']),
                     'policy_out': LaunchConfiguration('policy_out'),
                     'hold': float(LaunchConfiguration('hold').perform(context))}])

    planning = []
    if epistemic:
        domain = os.path.join(share, 'epddl', 'stale-maps.epddl')
        maps_library = os.path.join(share, 'epddl', 'maps.epddl')
        intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'),
                                    'libraries', 'intermediate.epddl')
        mapping = os.path.join(share, 'pddl', 'stale-maps-mapping.json')
        model = os.path.join(share, 'pddl', 'stale-maps.pddl')
        with open(os.path.join(share, 'params', 'stale_maps.yaml')) as fh:
            params = (fh.read().replace('EPDDL_DOMAIN', domain).replace('EPDDL_PROBLEM', problem)
                      .replace('MAPPING_FILE', mapping)
                      .replace('INTERMEDIATE_LIBRARY', intermediate)
                      .replace('MAPS_LIBRARY', maps_library))
        filled = tempfile.NamedTemporaryFile('w', suffix='_stale_maps.yaml', delete=False)
        filled.write(params)
        filled.close()
        planning.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('plansys2_bringup'),
                'launch', 'plansys2_bringup_launch_monolithic.py')),
            launch_arguments={'model_file': model, 'params_file': filled.name,
                              'epistemic_state': 'True'}.items()))
        planning.append(TimerAction(period=20.0, actions=[
            Node(package='stale_map_demo', executable='relay_action',
                 name=f'{action}_relay', additional_env=matching_cascade(), output='screen',
                 arguments=['--action', action],
                 parameters=[{'action_name': action, 'rate': 10.0}])
            for action in ('stage', 'clear', 'send_open', 'send_blocked', 'cross')]))

    staged = []
    if LaunchConfiguration('shutdown').perform(context).lower() == 'true':
        staged.append(RegisterEventHandler(OnProcessExit(
            target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))

    start = float(LaunchConfiguration('start_after').perform(context))
    return (gazebo + per_robot + planning +
            [TimerAction(period=8.0 + 3.0 * len(agents) + 2.0, actions=[crew]), view, director, rviz,
             TimerAction(period=start, actions=[mission])] + staged)


def generate_launch_description():
    share = get_package_share_directory('stale_map_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    xl = get_package_share_directory('warehouse_xl_rmf_demo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value='epistemic',
                              description='epistemic, fuse, overwrite or recency.'),
        DeclareLaunchArgument('gui', default_value='true', description='gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Write the acting robot to follow_file.'),
        DeclareLaunchArgument('follow_file', default_value='/tmp/stale_maps_follow'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'stale_maps.rviz')),
        DeclareLaunchArgument('policy_out', default_value=''),
        DeclareLaunchArgument('hold', default_value='0.0',
                              description='Seconds the mission waits before the forklift.'),
        DeclareLaunchArgument('start_after', default_value='45.0',
                              description='Seconds before the mission node starts.'),
        DeclareLaunchArgument('shutdown', default_value='false',
                              description='Bring everything down when the mission ends.'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join([
            os.path.join(xl, 'models'), os.path.join(aws, 'models'), os.path.join(tb3, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', ':'.join([
            aws, '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        OpaqueFunction(function=setup),
    ])
