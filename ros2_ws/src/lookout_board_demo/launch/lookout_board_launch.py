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
One run of the lookout board, headless.

    ros2 launch lookout_board_demo lookout_board_launch.py dir:=RUN formulation:=epistemic

RUN must hold the world tools/make_world.py wrote for the run's seed. The
formulation is epistemic, replan or conformant (see scripts/mission.py). The
parameters, written to RUN/params.yaml, are built here from tools/layout.py,
so that the perception's regions, the drive node's roadmap and the sensing
node's targets are the same geometry the world was built from.

The launch shuts down when the mission ends.
"""

import json
import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('lookout_board_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def perception():
    """The regions the epistemic state is told about: each slot of the board
    answers read-k, each window check-k, and the zone, once per lookout,
    look-k. A lit slot holds a post, so it reads blocked; a screened window
    reads blocked; the zone reads blocked when the crate is in it."""
    p = {'map_topic': '/map', 'free_below': 25, 'occupied_above': 65,
         'applicability_retries': 120, 'regions': []}
    for k in range(1, 5):
        p['regions'] += [f'slot{k}', f'window{k}', f'zone{k}']
        p[f'slot{k}'] = {'boxes': list(L.slot_region(k)), 'atom': f'clear-{k}',
                         'atom_true_when_clear': False, 'sensing_action': f'read-{k}_r1',
                         'outcome_when_clear': f'e-read{k}-dark',
                         'outcome_when_blocked': f'e-read{k}-lit'}
        p[f'window{k}'] = {'boxes': list(L.window_region(k)), 'atom': f'clear-{k}',
                           'atom_true_when_clear': True, 'sensing_action': f'check-{k}_r1',
                           'outcome_when_clear': f'e-check{k}-clear',
                           'outcome_when_blocked': f'e-check{k}-screened'}
        p[f'zone{k}'] = {'boxes': list(L.zone_region()), 'atom': 'blocked-z',
                         'atom_true_when_clear': False, 'sensing_action': f'look-{k}_r1',
                         'outcome_when_clear': f'e-look{k}-free',
                         'outcome_when_blocked': f'e-look{k}-blocked'}
    return p


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in ('epistemic', 'replan', 'conformant'), formulation
    epistemic = formulation == 'epistemic'
    epddl = os.path.join(SHARE, 'epddl')
    domain = os.path.join(epddl, 'lookout-board.epddl')
    problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
    pddl = os.path.join(SHARE, 'pddl', 'lookout-board.pddl')
    mapping = os.path.join(SHARE, 'pddl', 'lookout-board-mapping.json')

    params = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        # The domain expert checks the domain with a POPF of its own, which
        # writes its scratch files to /tmp unless told otherwise. Runs started
        # side by side then read each other's half-written check_domain.pddl
        # and one fails to start with "PDDL syntax error". Its own directory too.
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 60.0}},
        'epistemic_state': {'ros__parameters': {'epddl_domain': domain, 'epddl_problem': problem}},
        'epistemic_perception': {'ros__parameters': perception()},
    }
    if epistemic:
        params['planner'] = {'ros__parameters': {
            'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['EPISTEMIC'],
            'EPISTEMIC': {'plugin': 'plansys2/EpistemicPlanSolver', 'epddl_domain': domain,
                          'epddl_problem': problem, 'plank_command': '', 'task_file': '',
                          'heuristic': '', 'strategy': '', 'policy_file': '',
                          'action_mapping': mapping, 'conditional_plan': 'policy'}}}
        params['executor'] = {'ros__parameters': {
            'bt_builder_plugin': 'EpistemicBTBuilder',
            'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}}
    else:
        # POPF writes its domain, problem and plan in the run's own directory:
        # by default it uses /tmp/<namespace>/, which simultaneous runs share.
        params['planner'] = {'ros__parameters': {
            'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}}
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))

    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    gz_path = models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': gz_path}, output='screen')

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_burger.urdf')
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': open(urdf).read().replace('${namespace}', '')}])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True, 'odom_frame': 'odom', 'map_frame': 'map',
                     'base_frame': 'base_footprint', 'scan_topic': '/scan', 'mode': 'mapping',
                     'resolution': 0.05, 'max_laser_range': 8.0,
                     'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.0,
                     'map_update_interval': 1.0}])

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': pddl, 'params_file': filled,
                          'epistemic_state': 'True' if epistemic else 'False',
                          'epistemic_perception': 'True' if epistemic else 'False'}.items())

    nodes, _ = L.roadmap()
    edges = [f'{u}:{v}' for u, v in L.roadmap()[1]]
    drive = Node(
        package='lookout_board_demo', executable='drive_action_node', name='drive_action',
        output='screen', additional_env=cascade(),
        parameters=[{'action_name': 'goto', 'use_sim_time': True,
                     'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
                     'roadmap_y': [p[1] for p in nodes.values()], 'roadmap_edges': edges}])

    mode = 'epistemic' if epistemic else 'classical'

    def sense(action, targets):
        p = {'action_name': action, 'use_sim_time': True, 'mode': mode,
             'targets': list(targets)}
        for t, (face, knows, region, window) in targets.items():
            p[f'face_{t}'] = list(face)
            p[f'knows_{t}'] = knows
            p[f'region_{t}'] = list(region)
            if window:
                p[f'window_{t}'] = list(window)
        return Node(package='lookout_board_demo', executable='sense_action_node',
                    name=f'{action}_action', output='screen', additional_env=cascade(),
                    parameters=[p])

    zone_c = (0.0, 0.0)
    slots = {f's{k}': ((L.POST_X, L.SLOT_Y[k - 1]), f'(Kw r1 clear-{k})', L.slot_region(k), None)
             for k in range(1, 5)}
    views = {f'l{k}': (zone_c, f'(Kw r1 clear-{k})', L.window_region(k), None)
             for k in range(1, 5)}
    looks = {f'l{k}': (zone_c, '(Kw r1 blocked-z)', L.zone_region(), L.window_region(k))
             for k in range(1, 5)}

    recorder = Node(package='lookout_board_demo', executable='recorder.py', name='lookout_recorder',
                    output='screen', arguments=['--dir', run])
    mission = Node(package='lookout_board_demo', executable='mission.py', name='lookout_mission',
                   output='screen',
                   arguments=['--dir', run, '--formulation', formulation,
                              '--limit', LaunchConfiguration('limit').perform(context),
                              '--ros-args', '-p', 'plan_solver_timeout:=70.0'])
    json.dump({'formulation': formulation}, open(os.path.join(run, 'launch.json'), 'w'))
    return [gazebo, state_publisher, slam, plansys2, drive,
            sense('read_slot', slots), sense('check_view', views), sense('look_zone', looks),
            recorder, mission,
            RegisterEventHandler(OnProcessExit(
                target_action=mission,
                on_exit=[EmitEvent(event=Shutdown(reason='mission over'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        DeclareLaunchArgument('limit', default_value='600.0'),
        OpaqueFunction(function=build),
    ])
