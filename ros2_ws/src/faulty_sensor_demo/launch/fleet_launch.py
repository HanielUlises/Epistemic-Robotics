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
One run of a faulty sensor in the fleet, headless.

    ros2 launch faulty_sensor_demo fleet_launch.py dir:=RUN act:=2 formulation:=sensors

RUN holds the world make_world.py wrote. What runs:

  gzserver      the ward, two of its doors shut, and the act's robots
  door_node     one per robot, reading doors with its own laser from a mouth or a
                mark; short-sighted for the scouts the night says
  drive_node    per robot, one for (drive r from to), one for (passage r p from to)
                and, for a bed-mover, one for (backout r p from to)
  wait_node     per robot, one for readings and one for what is said
  plansys2      the bringup: the AletheiaPlanSolver with tools/plan.py as its planner,
                the epistemic executor and its epistemic state
  recorder.py   actions, poses and the radio, to RUN
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

SHARE = get_package_share_directory('faulty_sensor_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import furniture as F  # noqa: E402
import layout as L  # noqa: E402
import model as MD  # noqa: E402
import ranks as R  # noqa: E402
import worlds as W  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
LIBRARY = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
LOOK_WAIT = 30.0


def yields(r, robots):
    """Who r gives way to: every robot after it in the fleet's order, so that of any
    two that meet one waits."""
    return robots[robots.index(r) + 1:]


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    act = int(LaunchConfiguration('act').perform(context))
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in W.FORMULATIONS, formulation
    world = json.load(open(os.path.join(run, 'world.json')))
    robots = W.robots(act)

    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    models = ':'.join([os.path.join(run, 'models'), tb3, F.HOSPITAL, F.HOUSE, os.environ.get('GAZEBO_MODEL_PATH', '')])
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models}, output='screen')]

    role = {r.name: r.role for r in R.fleet(act)}
    for r in robots:
        nodes, south, north, cross = L.roadmap(r)
        roadmap = {'roadmap_names': [f'{n}_{r}' for n in nodes], 'roadmap_x': [p[0] for p in nodes.values()],
                   'roadmap_y': [p[1] for p in nodes.values()],
                   'edges': [f'{u}_{r} {v}_{r}' for u, v in south + north + cross]}
        others = [o for o in robots if o != r]
        actions.append(Node(package='faulty_sensor_demo', executable='door_node.py', name=f'doors_{r}',
                            output='screen', arguments=['--robot', r] + (['--short'] if r in world['short'] else [])))
        drives = [('drive', 2, False), ('passage', 3, False)] + ([('backout', 3, True)] if role[r] == 'mover' else [])
        for name, extra, reverse in drives:
            actions.append(Node(package='faulty_sensor_demo', executable='drive_node', name=f'{name}_{r}',
                                output='screen', additional_env=cascade(),
                                parameters=[dict(roadmap, action_name=name, use_sim_time=True, robot=r,
                                                 others=others or [''], gives_way=bool(yields(r, robots)),
                                                 yield_to=yields(r, robots) or [''], reverse=reverse,
                                                 specialized_arguments=[r] + [''] * extra)]))
        for kind in ('look', 'say'):
            actions.append(Node(package='faulty_sensor_demo', executable='wait_node', name=f'{kind}_{r}',
                                output='screen', additional_env=cascade(),
                                parameters=[dict(action_name=kind, use_sim_time=True, robot=r, kind=kind,
                                                 wait=LOOK_WAIT, specialized_arguments=[r, '', ''])]))

    epddl = os.path.join(SHARE, 'epddl')
    kind = R.MODEL[formulation]
    instance = f'act{act}-{kind}'
    subprocess.run([PLANK, 'export', '-d', os.path.join(epddl, f'{MD.DOMAIN[kind]}.epddl'), '-p',
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
                         'task_file': task, 'action_mapping': os.path.join(SHARE, 'pddl', f'fleet-mapping-act{act}.json'),
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
        launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'fleet-vocabulary.pddl'),
                          'params_file': filled, 'epistemic_state': 'True',
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='faulty_sensor_demo', executable='mission.py', name='fleet_mission', output='screen',
                   arguments=['--dir', run, '--act', str(act), '--formulation', formulation])
    recorder = Node(package='faulty_sensor_demo', executable='recorder.py', name='fleet_recorder',
                    output='screen', arguments=['--dir', run, '--robots', *robots])
    json.dump({'act': act, 'formulation': formulation, 'seed': world['seed'], 'loop': world['loop'],
               'short': world['short'], 'tag': world['tag']}, open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('act', default_value='1'),
        DeclareLaunchArgument('formulation', default_value='sensors'),
        OpaqueFunction(function=build),
    ])
