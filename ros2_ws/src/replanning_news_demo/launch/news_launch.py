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
One run of replanning on news, headless.

    ros2 launch replanning_news_demo news_launch.py dir:=RUN act:=2 formulation:=replan

RUN holds the world make_world.py wrote. What runs:

  gzserver      the residence, its doors as the run's world has them, and the
                act's robots: the courier, the scout from the second act, the
                cleaner in the third
  door_node     one per robot that looks, deciding doors with its own laser
  drive_node    one per drive and robot: to a passage's mouth or the hatch,
                through a door, along the corridor to the resident
  wait_node     one per look and message
  plansys2      the bringup: the in-process epistemic solver and the epistemic
                executor, or POPF and the classical executor
  news.py       says the news at worlds.NEWS_AT, and announces it under replan
  recorder.py   actions, poses, the radio and the belief, to RUN
  mission.py    plans, executes, plans again; the launch ends when it does
"""

import json
import os
import subprocess
import sys

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

SHARE = get_package_share_directory('replanning_news_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import furniture as F  # noqa: E402
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
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
    epistemic = formulation != 'classical'
    world = json.load(open(os.path.join(run, 'world.json')))
    robots = W.ROBOTS[act]

    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    models = ':'.join([os.path.join(run, 'models'), tb3, F.MODELS, os.environ.get('GAZEBO_MODEL_PATH', '')])
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models}, output='screen')]

    nodes, south, north = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()]}
    south_edges = [f'{u} {v}' for u, v in south]
    north_edges = [f'{u} {v}' for u, v in north]

    def drive(name, robot, **kw):
        return Node(package='replanning_news_demo', executable='drive_node', name=f'{name}_{robot}',
                    output='screen', additional_env=cascade(),
                    parameters=[dict(roadmap, action_name=name, use_sim_time=True, robot=robot,
                                     specialized_arguments=[robot], **kw)])

    def wait(name, robot, **kw):
        return Node(package='replanning_news_demo', executable='wait_node', name=f'{name}_{robot}',
                    output='screen', additional_env=cascade(),
                    parameters=[dict(action_name=name, use_sim_time=True, robot=robot, wait=LOOK_WAIT,
                                     specialized_arguments=[robot], **kw)])

    for r in robots:
        if r in ('c', 's'):
            actions.append(Node(package='replanning_news_demo', executable='door_node.py', name=f'doors_{r}',
                                output='screen', arguments=['--robot', r]))
    if epistemic:
        for r in robots:
            if r in ('c', 's'):
                for v in ('vw', 'vm', 've'):
                    actions.append(drive(f'goto_{v}', r, target=v, edges=south_edges))
                for d in (('w', 'e') if r == 'c' else ('w', 'm', 'e')):
                    actions.append(wait(f'look_{d}', r, kind='look', door=d))
            if r in ('s', 'k'):
                for d in ('w', 'm', 'e'):
                    memory = ('open' if world['open'][d] else 'shut') if (r == 'k' and d == 'e') else ''
                    actions.append(wait(f'tell_{d}', r, kind='tell', door=d, memory=memory))
        for d in ('w', 'm', 'e'):
            actions.append(drive(f'cross_{d}', 'c', route_fixed=L.CROSS[d]))
        actions.append(drive('deliver', 'c', target='r', edges=north_edges))
    else:
        for v in ('vw', 'vm', 've'):
            for a in P.SOUTH:
                if a != v:
                    actions.append(drive(f'goto_{v}_from_{a}', 'c', target=v, edges=south_edges))
        for d in ('w', 'm', 'e'):
            actions.append(drive(f'cross_{d}', 'c', route_fixed=L.CROSS[d], stop_when_blocked=True))
            actions.append(drive(f'deliver_from_{d}', 'c', target='r', edges=north_edges))

    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    common = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 60.0}},
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
    }
    if epistemic:
        epddl = os.path.join(SHARE, 'epddl')
        subprocess.run([PLANK, 'export', '-d', os.path.join(epddl, 'replanning-news.epddl'), '-p',
                        os.path.join(epddl, 'instances', f'act{act}.epddl'), '-l', LIBRARY,
                        '-o', os.path.join(run, 'task')], check=True, capture_output=True)
        task = os.path.join(run, 'task', f'act{act}.json')
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['EPISTEMIC'],
                'EPISTEMIC': {'plugin': 'plansys2/EpistemicPlanSolver', 'task_file': task,
                              'heuristic': '', 'strategy': 'aostar', 'policy_file': '',
                              'action_mapping': os.path.join(SHARE, 'pddl', 'news-mapping.json'),
                              'conditional_plan': 'policy', 'consistent_beliefs': True}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # independent actions of different robots dispatched together
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {'task_file': task}}})
        state = 'True'
        model = os.path.join(SHARE, 'pddl', 'news-vocabulary.pddl')
    else:
        params = dict(common, **{
            'planner': {'ros__parameters': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}}})
        state = 'False'
        model = os.path.join(SHARE, 'pddl', 'news-classical.pddl')
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': model, 'params_file': filled, 'epistemic_state': state,
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='replanning_news_demo', executable='mission.py', name='news_mission', output='screen',
                   arguments=['--dir', run, '--act', str(act), '--formulation', formulation])
    news = Node(package='replanning_news_demo', executable='news.py', name='news', output='screen',
                arguments=['--dir', run, '--act', str(act), '--formulation', formulation])
    recorder = Node(package='replanning_news_demo', executable='recorder.py', name='news_recorder', output='screen',
                    arguments=['--dir', run, '--robots', *robots])
    json.dump({'act': act, 'formulation': formulation, 'seed': world['seed'], 'open': world['open'],
               'tag': world['tag'], 'start_at': W.START_AT, 'news_at': W.NEWS_AT},
              open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, news, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('act', default_value='1'),
        DeclareLaunchArgument('formulation', default_value='replan'),
        OpaqueFunction(function=build),
    ])
