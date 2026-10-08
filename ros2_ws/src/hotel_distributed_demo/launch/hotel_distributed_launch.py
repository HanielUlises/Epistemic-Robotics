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
The hotel's distributed leak, over the Open-RMF hotel.

    ros2 launch hotel_distributed_demo hotel_distributed_launch.py
    ros2 launch hotel_distributed_demo hotel_distributed_launch.py fleet:=filter
    ros2 launch hotel_distributed_demo hotel_distributed_launch.py fleet:=siloed leak:=L2_room15

One command brings up the stock rmf_demos hotel with its three fleets and two
lifts, the planning system, the bridge to Open-RMF, the crew, the knowledge
view and the mission.

  fleet   epistemic, broadcast, filter or siloed: which problem the planner is
          given, so which channels the robots may use and which goal the
          policy is for. Every fleet is judged by the whole goal.
  leak    the room that is leaking: L2_room1, L2_room15, L3_room1 or
          L3_room15. It is ground truth the simulator holds and no robot does:
          the bridge reports from it what the cleaner says of the column, what
          the concierge says of the floor, and what an inspection finds.
  gui     gzclient, looking straight down at the building.
"""

import json
import os
import re
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FLEETS = ('epistemic', 'broadcast', 'filter', 'siloed')
ROOMS = ('L2_room1', 'L2_room15', 'L3_room1', 'L3_room15')
WEBSOCKET_PORT = 7879


def task_map(template, leak):
    """The bridge's task map, with the ground truth of this leak written in:
    which room an inspection finds wet, which floor the concierge says and
    which column the cleaner says."""
    with open(template) as fh:
        data = json.load(fh)
    floor, column = leak.split('_', 1)
    actions = data['actions']
    actions['look_into']['outcomes'] = {
        z.lower(): ('e-wet' if z == leak else 'e-dry') for z in ROOMS}
    actions['say_floor']['default_outcome'] = f'e-floor-{floor}'
    actions['say_column']['default_outcome'] = f'e-col-{column}'
    actions['page_floor']['default_outcome'] = f'e-page-floor-{floor}'
    actions['page_column']['default_outcome'] = f'e-page-col-{column}'
    out = tempfile.NamedTemporaryFile(mode='w', suffix='_hotel_task_map.json', delete=False)
    json.dump(data, out, indent=2)
    out.close()
    return out.name


def launch_setup(context, *args, **kwargs):
    here = get_package_share_directory('hotel_distributed_demo')
    fleet = LaunchConfiguration('fleet').perform(context)
    leak = LaunchConfiguration('leak').perform(context)
    if fleet not in FLEETS:
        raise RuntimeError(f'fleet:={fleet} is not one of {list(FLEETS)}')
    if leak not in ROOMS:
        raise RuntimeError(f'leak:={leak} is not one of {list(ROOMS)}')

    epddl = os.path.join(here, 'epddl')
    domain = os.path.join(epddl, 'hotel-distributed.epddl')
    problem = os.path.join(epddl, f'{fleet}.epddl')
    earshot = os.path.join(epddl, 'earshot.epddl')
    intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'),
                                'libraries', 'intermediate.epddl')
    mapping = os.path.join(here, 'pddl', 'mapping.json')
    model = os.path.join(here, 'pddl', 'hotel-distributed.pddl')

    with open(os.path.join(here, 'params', 'hotel_distributed.yaml')) as fh:
        params = fh.read()
    params = (params.replace('EPDDL_DOMAIN', domain).replace('EPDDL_PROBLEM', problem)
              .replace('INTERMEDIATE_LIBRARY', intermediate)
              .replace('EARSHOT_LIBRARY', earshot).replace('MAPPING_FILE', mapping))
    filled = tempfile.NamedTemporaryFile(mode='w', suffix='_hotel_distributed.yaml',
                                         delete=False)
    filled.write(params)
    filled.close()

    hotel = get_package_share_directory('hotel_rmf_demo')
    maps = get_package_share_directory('rmf_demos_maps')
    fleets = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(hotel, 'launch', 'hotel_fleet.launch.xml')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={'use_rmf_panel': 'false', 'headless': 'true',
                          'server_uri': f'ws://localhost:{WEBSOCKET_PORT}'}.items())

    # gzclient on its own, so that the film has Gazebo without RViz: the
    # hotel's launch draws both or neither. Same world and paths as there.
    gz = '11'
    world = os.path.join(hotel, 'worlds', 'hotel.world')
    model_path = ':'.join([os.path.join(maps, 'maps', 'hotel', 'models'),
                           os.path.join(get_package_share_directory('rmf_demos_assets'),
                                        'models'),
                           f'/usr/share/gazebo-{gz}/models'])
    resource_path = ':'.join([get_package_share_directory('rmf_demos_assets'),
                              f'/usr/share/gazebo-{gz}'])
    plugin_path = ':'.join([
        os.path.join(get_package_prefix('rmf_robot_sim_gz_classic_plugins'), 'lib',
                     'rmf_robot_sim_gz_classic_plugins'),
        os.path.join(get_package_prefix('rmf_building_sim_gz_classic_plugins'), 'lib',
                     'rmf_building_sim_gz_classic_plugins'),
        f'/usr/share/gazebo-{gz}'])
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose', world], output='both',
        condition=IfCondition(LaunchConfiguration('gui')),
        additional_env={'GAZEBO_MODEL_PATH': model_path,
                        'GAZEBO_RESOURCE_PATH': resource_path,
                        'GAZEBO_PLUGIN_PATH': plugin_path})

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': model, 'params_file': filled.name,
                          'epistemic_state': 'True'}.items())

    bridge = Node(
        package='eplansys_rmf_bridge', executable='rmf_action_node',
        name='eplansys_rmf_bridge', output='screen',
        parameters=[{'task_map': task_map(os.path.join(here, 'config', 'task_map.json'), leak),
                     'websocket_port': WEBSOCKET_PORT,
                     # A backstop against a task that has stopped, not a budget:
                     # wall-clock, while the fleet runs on sim time, and a
                     # recording on a software renderer slows the simulation.
                     'task_timeout': 3600.0}])

    graphs = os.path.join(maps, 'maps', 'hotel', 'nav_graphs')
    crew = Node(
        package='hotel_distributed_demo', executable='hotel_crew.py', output='screen',
        parameters=[{'leak': leak, 'nav_graphs': graphs, 'use_sim_time': True,
                     'scene': LaunchConfiguration('scene')}])
    view = Node(
        package='hotel_distributed_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'leak': leak, 'fleet': fleet}])

    # The mission's PlannerClient is a node of its own in the mission's
    # process and waits plan_solver_timeout for an answer, 15 s unless told.
    # The epistemic fleet's search takes minutes, so the client is given the
    # planner's budget and a margin.
    budget = float(re.search(r'plan_solver_timeout:\s*([\d.]+)', params).group(1))
    mission = Node(
        package='hotel_distributed_demo', executable='hotel_distributed_mission',
        output='screen',
        arguments=['--ros-args', '-p', f'plan_solver_timeout:={budget + 15.0}'],
        parameters=[{'fleet': fleet, 'leak': leak,
                     'policy_out': LaunchConfiguration('policy_out'),
                     'hold': float(LaunchConfiguration('hold').perform(context))}])

    staged = []
    if LaunchConfiguration('shutdown').perform(context).lower() == 'true':
        staged.append(RegisterEventHandler(OnProcessExit(
            target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))

    return ([fleets, plansys2, bridge, crew, view,
             TimerAction(period=8.0, actions=[gzclient])] +
            [TimerAction(period=float(LaunchConfiguration('start_after').perform(context)),
                         actions=[mission])] + staged)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value='epistemic',
                              description='epistemic, broadcast, filter or siloed'),
        DeclareLaunchArgument('leak', default_value='L3_room1',
                              description='L2_room1, L2_room15, L3_room1 or L3_room15'),
        DeclareLaunchArgument('gui', default_value='true', description='gzclient'),
        DeclareLaunchArgument('rmf', default_value='true',
                              description='Launch the hotel and its fleets. false when they '
                                          'are already running.'),
        DeclareLaunchArgument('scene', default_value='true',
                              description='Draw the guest and the water in Gazebo.'),
        DeclareLaunchArgument('policy_out', default_value=''),
        DeclareLaunchArgument('hold', default_value='0.0',
                              description='Seconds the mission waits before the setup.'),
        DeclareLaunchArgument('start_after', default_value='30.0',
                              description='Seconds before the mission node starts.'),
        DeclareLaunchArgument('shutdown', default_value='true',
                              description='Bring everything down when the mission ends.'),
        OpaqueFunction(function=launch_setup),
    ])
