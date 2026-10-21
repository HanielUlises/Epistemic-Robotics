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
One run of the collapsed wing, headless.

    ros2 launch collapsed_wing_demo wing_launch.py dir:=RUN formulation:=epistemic

RUN holds the world make_world.py wrote. What runs:

  gzserver        the building, the rubble of the run's world, and three Burgers
  slam_toolbox    one per robot, building that robot's own map from its own laser
                  in its own frame, s1/map, published on /<robot>/map and nowhere
                  else: a map one robot builds is not another's
  doorway_node    one per robot, deciding each doorway on that robot's map
  drive_node      one per drive: each scout's to its vantage, and the medic's by
                  each way and back to the hall; under the optimistic formulation
                  the medic's crossings stop and fail on rubble
  wait_node       one per inspection, check and radio message
  plansys2        the bringup: Aletheia and the epistemic executor for the policy,
                  or POPF for the classical formulations
  recorder.py     actions, poses and the radio, to RUN
  mission.py      plans, executes, replans; the launch ends when it does
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

SHARE = get_package_share_directory('collapsed_wing_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
import worlds as W  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
ROBOTS = ('s1', 's2', 'm')
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
LOOK_WAIT = 30.0


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def slam(r):
    return Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', namespace=r,
                output='screen',
                parameters=[{'use_sim_time': True, 'odom_frame': f'{r}/odom', 'map_frame': f'{r}/map',
                             'base_frame': f'{r}/base_footprint', 'scan_topic': f'/{r}/scan', 'mode': 'mapping',
                             'resolution': 0.05, 'max_laser_range': 8.0,
                             # a scout standing at its vantage must keep mapping while it looks
                             'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.0,
                             'map_update_interval': 1.0, 'transform_publish_period': 0.05}],
                # slam_toolbox publishes its map on the absolute /map: each robot's is moved into its own namespace
                remappings=[('/map', f'/{r}/map'), ('/map_metadata', f'/{r}/map_metadata')])


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in W.FORMULATIONS, formulation
    epistemic = formulation == 'epistemic'
    world = json.load(open(os.path.join(run, 'world.json')))

    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')},
        output='screen')]
    urdf = open(os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf',
                             'turtlebot3_burger.urdf')).read().replace('${namespace}', '')
    for r in ROBOTS:
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=r,
                 name='robot_state_publisher', output='screen',
                 parameters=[{'use_sim_time': True, 'robot_description': urdf, 'frame_prefix': f'{r}/'}]),
            slam(r),
            Node(package='collapsed_wing_demo', executable='doorway_node.py', name=f'doorways_{r}', output='screen',
                 arguments=['--robot', r, '--dir', run]),
        ]

    # The drives.
    nodes = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()]}

    def drive(name, robot, **kw):
        return Node(package='collapsed_wing_demo', executable='drive_node', name=f'{name}_{robot}', output='screen',
                    additional_env=cascade(),
                    parameters=[dict(roadmap, action_name=name, use_sim_time=True, robot=robot,
                                     mode='epistemic' if epistemic else 'classical', outcome='',
                                     specialized_arguments=[robot], **kw)])
    for d, s in P.SIDE.items():
        actions.append(drive(f'goto_{d}', s, route_fixed=L.ROUTE[s][1:]))
    for d in P.WAYS:
        route = L.ROUTE[f'm_{d}']
        # From the start, or from the hall after backing out of a blocked way.
        actions.append(drive(f'cross_{d}', 'm', places=['m0', 'hall'], places_x=[nodes['m0'][0], nodes['hall'][0]],
                             places_y=[nodes['m0'][1], nodes['hall'][1]], route_from_m0=route[1:],
                             route_from_hall=route[2:], stop_when_blocked=formulation == 'optimistic'))
        actions.append(drive(f'retreat_{d}', 'm', route_fixed=P.RETREAT[d]))

    # Looking and speaking.
    def wait(name, robot, **kw):
        return Node(package='collapsed_wing_demo', executable='wait_node', name=f'{name}_{robot}', output='screen',
                    additional_env=cascade(),
                    parameters=[dict(action_name=name, use_sim_time=True, robot=robot, wait=LOOK_WAIT,
                                     specialized_arguments=[robot], **kw)])
    for d, s in P.SIDE.items():
        actions.append(wait(f'inspect_{d}', s, kind='inspect', door=d, outcome=f'e-inspect-{d}-{{v}}'))
        actions.append(wait(f'check_{d}', s, kind='check', door=d))
    for d in P.WAYS:
        for s in ('s1', 's2'):
            actions.append(wait(f'tell_{d}', s, kind='tell', door=d, outcome=f'e-tell-{d}-{{v}}'))

    # The planner and the executor.
    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    common = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 120.0}},
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
    }
    if epistemic:
        epddl = os.path.join(SHARE, 'epddl')
        intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                                    'intermediate.epddl')
        subprocess.run([PLANK, 'export', '-d', os.path.join(epddl, 'collapsed-wing.epddl'), '-p',
                        os.path.join(epddl, 'instances', 'problem_1.epddl'), '-l', intermediate,
                        '-o', os.path.join(run, 'task')], check=True, capture_output=True)
        task = os.path.join(run, 'task', 'problem_1.json')
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['ALETHEIA'],
                'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver', 'command': PLANNER,
                             'arguments': '--conditional --consistent-beliefs', 'output_dir': run,
                             'task_file': task,
                             'action_mapping': os.path.join(SHARE, 'pddl', 'wing-mapping.json'),
                             'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # independent actions of different robots dispatched together
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {'task_file': task}}})
        state = 'True'
        model = os.path.join(SHARE, 'pddl', 'wing-vocabulary.pddl')
    else:
        params = dict(common, **{
            'planner': {'ros__parameters': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}}})
        state = 'False'
        model = os.path.join(SHARE, 'pddl', f'wing-{formulation}.pddl')
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': model, 'params_file': filled, 'epistemic_state': state,
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='collapsed_wing_demo', executable='mission.py', name='wing_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation])
    recorder = Node(package='collapsed_wing_demo', executable='recorder.py', name='wing_recorder', output='screen',
                    arguments=['--dir', run])
    json.dump({'formulation': formulation, 'seed': world['seed'], 'clear': world['clear'], 'start_at': W.START_AT,
               'look_wait': LOOK_WAIT}, open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        OpaqueFunction(function=build),
    ])
