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
One run of the belief and empathy demonstration, headless.

    ros2 launch belief_empathy_demo be_launch.py dir:=RUN formulation:=epistemic

RUN holds the world make_world.py wrote. What runs:

  gzserver      the area, its obstacles, the task's site, two Burgers and the
                quadrotor
  world_node    the radio, with layout.R of range (unlimited under the ideal
                formulation), and the task: found, and done
  ugv_node      one per ground vehicle: coverage at its particle, faults,
                answering hails, driving to the site when asked
  uav_node      the aerial vehicle: coverage, seeking particles, hailing,
                flying to the site
  plansys2      under the epistemic formulation only: Aletheia as the planner,
                over the instance for the half of the area the site is in (of
                belief-empathy-latent.epddl, when RUN/world.json is of the
                variant with latent tool faults), and
                the epistemic executor, with one relay_node per action and
                robot; the other formulations need no planner
  recorder.py   actions, poses, the radio and the events, to RUN
  mission.py    the formulation; the launch ends when it does
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

SHARE = get_package_share_directory('belief_empathy_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
import worlds as W  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    world = json.load(open(os.path.join(run, 'world.json')))
    latent = world.get('variant') == 'latent'
    assert formulation in (W.LATENT_FORMULATIONS if latent else W.FORMULATIONS), formulation

    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')},
        output='screen')]
    radio = 1e9 if formulation == 'ideal' else L.R
    actions.append(Node(package='belief_empathy_demo', executable='world_node.py', name='be_world', output='screen',
                        arguments=['--dir', run, '--range', str(radio)]))
    for g in L.UGVS:
        actions.append(Node(package='belief_empathy_demo', executable='ugv_node.py', name=f'ugv_{g}',
                            output='screen', arguments=['--robot', g, '--dir', run, '--formulation', formulation]))
    actions.append(Node(package='belief_empathy_demo', executable='uav_node.py', name='uav_u', output='screen',
                        arguments=['--dir', run, '--formulation', formulation]))

    instance = f"problem_{'latent_' if latent else ''}{world['side']}"
    domain = 'belief-empathy-latent.epddl' if latent else 'belief-empathy.epddl'
    if formulation in ('epistemic', 'timed'):
        # One relay per action and robot: the aerial vehicle's actions, and a
        # drive to the site for each ground vehicle.
        def relay(name, robot):
            return Node(package='belief_empathy_demo', executable='relay_node', name=f'{name}_{robot}',
                        output='screen', additional_env=cascade(),
                        parameters=[{'action_name': name, 'use_sim_time': True, 'robot': robot,
                                     'specialized_arguments': [robot]}])
        for name in P.VOCAB:
            if name == 'go_site':
                actions += [relay(name, g) for g in L.UGVS]
            else:
                actions.append(relay(name, 'u'))
        epddl = os.path.join(SHARE, 'epddl')
        intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                                    'intermediate.epddl')
        subprocess.run([PLANK, 'export', '-d', os.path.join(epddl, domain), '-p',
                        os.path.join(epddl, 'instances', f'{instance}.epddl'), '-l', intermediate,
                        os.path.join(epddl, 'hailing.epddl'), '-o', os.path.join(run, 'task')],
                       check=True, capture_output=True)
        task = os.path.join(run, 'task', f'{instance}.json')
        # The timed formulation plans through the same plugin with the timed
        # AND-OR search in place of Aletheia: it is told the site, since it
        # times every move to it, and reads the task's skeleton, computed
        # beforehand, from the package. The plugin validates its policy as it
        # does Aletheia's. TIMED_FIXED=1 runs instead the policies the first
        # latent batch ran, kept in policies/.
        if formulation == 'timed':
            x, y = world['site']
            if os.environ.get('TIMED_FIXED') == '1':
                command = os.path.join(SHARE, 'tools', 'fixed_policy.sh')
                arguments = '--fixed ' + os.path.join(SHARE, 'policies', f'policy_latent_site{x:+.0f}_{y:.0f}.json')
            else:
                command = os.path.join(SHARE, 'tools', 'timed_policy.py')
                arguments = f"--site {x} {y} --skeletons {os.path.join(SHARE, 'policies', 'skeletons')}"
                if not latent:
                    # the replica's ground vehicles answer a drive on arrival
                    arguments += ' --drive-blocks'
        else:
            command, arguments = PLANNER, '--conditional --consistent-beliefs'
        bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                               'plansys2_bringup_launch_monolithic.py')
        params = {
            'plansys2': {'ros__parameters': {'use_real_time': False}},
            'planner_client': {'ros__parameters': {'plan_solver_timeout': 120.0}},
            'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['ALETHEIA'],
                'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver', 'command': command,
                             'arguments': arguments, 'output_dir': run,
                             'task_file': task,
                             'action_mapping': os.path.join(SHARE, 'pddl', 'be-mapping.json'),
                             'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # independent actions of different robots dispatched together
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {'task_file': task}}}
        filled = os.path.join(run, 'params.yaml')
        open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'be-vocabulary.pddl'),
                              'params_file': filled, 'epistemic_state': 'True',
                              'epistemic_perception': 'False'}.items()))
    mission = Node(package='belief_empathy_demo', executable='mission.py', name='be_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation, '--instance', instance])
    recorder = Node(package='belief_empathy_demo', executable='recorder.py', name='be_recorder', output='screen',
                    arguments=['--dir', run])
    json.dump({'formulation': formulation, 'variant': world.get('variant', 'replica'), 'seed': world['seed'],
               'world': world['world'], 'site': world['site'],
               'start_at': W.START_AT, 'radio': radio if radio < 1e8 else None, 'instance': instance},
              open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        OpaqueFunction(function=build),
    ])
