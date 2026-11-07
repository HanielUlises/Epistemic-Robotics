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
One run of belief revision in a changing ward, headless.

    ros2 launch belief_revision_demo revision_launch.py dir:=RUN act:=2 formulation:=explanation

RUN holds the world make_world.py wrote. What runs:

  gzserver      the ward, its doors open at dusk, the act's couriers and the porter
  porter_node   walks the porter at one, and at two when the policy asks; swings the doors
  door_node     one per courier, deciding doors with its own laser from a lobby's bend
  drive_node    per courier, one for (drive r from to) and one for (lobby r l from to)
  wait_node     per courier, one for looks and one for reports; and the porter's round
  plansys2      the bringup: the AletheiaPlanSolver with tools/plan.py as its planner,
                the epistemic executor and its epistemic state
  recorder.py   actions, poses, the radio and the doors, to RUN
  mission.py    plans and executes; the launch ends when it does
"""

import json
import os
import subprocess
import sys

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, \
    OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('belief_revision_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import furniture as F  # noqa: E402
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
LIBRARY = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
LOOK_WAIT = 30.0


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    act = int(LaunchConfiguration('act').perform(context))
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in W.FORMULATIONS, formulation
    world = json.load(open(os.path.join(run, 'world.json')))
    robots = W.ROBOTS[act]

    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    models = ':'.join([os.path.join(run, 'models'), tb3, F.HOSPITAL, F.HOUSE, os.environ.get('GAZEBO_MODEL_PATH', '')])
    plugins = ':'.join([os.path.join(get_package_prefix('belief_revision_demo'), 'lib'),
                        os.environ.get('GAZEBO_PLUGIN_PATH', '')])
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models, 'GAZEBO_PLUGIN_PATH': plugins}, output='screen')]

    nodes, south, north, cross = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()],
               'edges': [f'{u} {v}' for u, v in south + north + cross]}

    for r in robots:
        others = [o for o in robots if o != r]
        actions.append(Node(package='belief_revision_demo', executable='door_node.py', name=f'doors_{r}',
                            output='screen', arguments=['--robot', r]))
        for name in ('drive', 'lobby'):
            actions.append(Node(package='belief_revision_demo', executable='drive_node', name=f'{name}_{r}',
                                output='screen', additional_env=cascade(),
                                parameters=[dict(roadmap, action_name=name, use_sim_time=True, robot=r,
                                                 others=others or [''], gives_way=(r == 'c' and bool(others)),
                                                 specialized_arguments=[r] + [''] * (2 if name == 'drive' else 3))]))
        for kind in ('look', 'tell'):
            actions.append(Node(package='belief_revision_demo', executable='wait_node', name=f'{kind}_{r}',
                                output='screen', additional_env=cascade(),
                                parameters=[dict(action_name=kind, use_sim_time=True, robot=r, kind=kind,
                                                 wait=LOOK_WAIT,
                                                 specialized_arguments=[r] + [''] * (2 if kind == 'look' else 1))]))
    if act == 3:
        actions.append(Node(package='belief_revision_demo', executable='wait_node', name='round_p',
                            output='screen', additional_env=cascade(),
                            parameters=[dict(action_name='round', use_sim_time=True, robot='p', kind='round',
                                             specialized_arguments=['p'])]))

    epddl = os.path.join(SHARE, 'epddl')
    instance = f'act{act}-{formulation}'
    subprocess.run([PLANK, 'export', '-d', os.path.join(epddl, 'belief-revision.epddl'), '-p',
                    os.path.join(epddl, 'instances', f'{instance}.epddl'), '-l', LIBRARY,
                    '-o', os.path.join(run, 'task')], check=True, capture_output=True)
    task = os.path.join(run, 'task', f'{instance}.json')
    params = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 120.0}},
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
        'planner': {'ros__parameters': {
            'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['ALETHEIA'],
            'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver',
                         'command': os.path.join(SHARE, 'tools', 'plan.py'),
                         'arguments': f'--act {act} --formulation {formulation}', 'output_dir': run,
                         'task_file': task, 'action_mapping': os.path.join(SHARE, 'pddl', 'revision-mapping.json'),
                         'conditional_plan': 'policy'}}},
        'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                         'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
        # independent actions of different robots dispatched together
        'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
        'epistemic_state': {'ros__parameters': {'task_file': task}}}
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'revision-vocabulary.pddl'),
                          'params_file': filled, 'epistemic_state': 'True',
                          'epistemic_perception': 'False'}.items()))
    porter = Node(package='belief_revision_demo', executable='porter_node.py', name='porter', output='screen',
                  arguments=['--dir', run])
    mission = Node(package='belief_revision_demo', executable='mission.py', name='revision_mission', output='screen',
                   arguments=['--dir', run, '--act', str(act), '--formulation', formulation])
    recorder = Node(package='belief_revision_demo', executable='recorder.py', name='revision_recorder',
                    output='screen', arguments=['--dir', run, '--robots', *robots])
    json.dump({'act': act, 'formulation': formulation, 'seed': world['seed'], 'loop': world['loop'],
               'plan': world['plan'], 'tag': world['tag']}, open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, porter, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('act', default_value='1'),
        DeclareLaunchArgument('formulation', default_value='explanation'),
        OpaqueFunction(function=build),
    ])
