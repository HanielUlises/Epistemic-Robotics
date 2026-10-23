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
One run of the gossip demonstration, headless.

    ros2 launch gossip_coverage_demo gc_launch.py dir:=RUN formulation:=epistemic

RUN holds the world make_world.py wrote. What runs:

  gzserver      the arena, its crates and three Burgers
  plansys2      under the epistemic formulation only: the epistemic state,
                the planner with Aletheia (plansys2/AletheiaPlanSolver), and the
                epistemic executor with parallel dispatch, over the vocabulary
                of pddl/; one relay process per robot serves its actions
  mission.py    everything else, and the formulation; the launch ends when
                it does
"""

import json
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, \
    OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('gossip_coverage_demo')
SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
ROBOTS = ('r1', 'r2', 'r3')
RELAYED = ['call', 'await', 'leave', 'go_site_0', 'go_site_1', 'go_site_2', 'do_task_1', 'do_task_2', 'do_task_3']


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    world = json.load(open(os.path.join(run, 'world.json')))
    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')},
        output='screen')]
    if formulation == 'epistemic':
        relays = {}
        for r in ROBOTS:
            for a in RELAYED:
                relays[f'relay_{a}_{r}'] = {'ros__parameters': {
                    'action_name': a, 'specialized_arguments': [r], 'use_sim_time': True}}
        relay_params = os.path.join(run, 'relays.yaml')
        open(relay_params, 'w').write(yaml.safe_dump(relays, sort_keys=False))
        for r in ROBOTS:
            # no name= here: launch would remap every node of the process to it
            actions.append(Node(package='gossip_coverage_demo', executable='relay_node',
                                output='screen', additional_env=cascade(),
                                arguments=[r] + RELAYED + ['--ros-args', '--params-file', relay_params]))
        bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                               'plansys2_bringup_launch_monolithic.py')
        params = {
            'plansys2': {'ros__parameters': {'use_real_time': False}},
            'planner_client': {'ros__parameters': {'plan_solver_timeout': 60.0}},
            'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['ALETHEIA'],
                'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver', 'command': PLANNER,
                             'arguments': '--strategy replan --subjective auto --consistent-beliefs',
                             'output_dir': run,
                             'action_mapping': os.path.join(SHARE, 'pddl', 'gc-mapping.json'),
                             'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # independent actions of different robots dispatched together, as
            # schedule_policy arranges them
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {}}}
        filled = os.path.join(run, 'params.yaml')
        open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'gc-vocabulary.pddl'),
                              'params_file': filled, 'epistemic_state': 'True',
                              'epistemic_perception': 'False'}.items()))
    mission = Node(package='gossip_coverage_demo', executable='mission.py', name='gc_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation,
                              '--ros-args', '-p', 'plan_solver_timeout:=60.0'])
    json.dump({'formulation': formulation, 'seed': world['seed'], 'case': world['case']},
              open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        OpaqueFunction(function=build),
    ])
