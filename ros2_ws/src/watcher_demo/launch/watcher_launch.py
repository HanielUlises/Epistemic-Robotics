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
One run of the watcher, headless.

    ros2 launch watcher_demo watcher_launch.py dir:=RUN formulation:=epistemic

RUN must hold the world tools/make_world.py wrote for the run's seed.

epistemic  PlanSys2 with ePlanSys's planner over the EPDDL task, the epistemic
           executor, and the carrier's action nodes;
direct     PlanSys2 with POPF over the classical domain, the same action
discreet   nodes, the carrier planning alone (scripts/mission.py).

Under every formulation two observers run, one on the ally r2 and one on the
watcher r3, each recording what its own laser saw of the carrier. They are
told nothing by the planner or the executor. The launch shuts down when the
mission has ended.
"""

import json
import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('watcher_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
ROBOTS = ('r1', 'r2', 'r3')


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def shrink(box, m):
    return (box[0] + m, box[1] + m, box[2] - m, box[3] - m)


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in ('epistemic', 'direct', 'discreet'), formulation
    epistemic = formulation == 'epistemic'
    epddl = os.path.join(SHARE, 'epddl')
    domain = os.path.join(epddl, 'watcher.epddl')
    problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
    # Naming any library means naming every one the domain declares.
    libraries = [os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                              'intermediate.epddl'), os.path.join(epddl, 'sightlines.epddl')]

    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')},
        output='screen')]
    urdf = open(os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf',
                             'turtlebot3_burger.urdf')).read().replace('${namespace}', '')
    for r in ROBOTS:
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=r, name='robot_state_publisher', output='screen',
                            parameters=[{'use_sim_time': True, 'robot_description': urdf,
                                         'frame_prefix': f'{r}/'}]))
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name=f'map_to_{r}_odom', output='screen',
                            arguments=['--frame-id', 'map', '--child-frame-id', f'{r}/odom']))

    nodes, _ = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()]}
    mode = 'epistemic' if epistemic else 'classical'

    def drive(action, outcome='', **routes):
        p = dict(roadmap, action_name=action, use_sim_time=True, robot='r1', mode=mode, outcome=outcome,
                 specialized_arguments=['r1'], **routes)
        prefix = os.environ.get('WATCHER_DRIVE_PREFIX') or None
        return Node(package='watcher_demo', executable='drive_node', name=f'{action}_r1', output='screen',
                    additional_env=cascade(), parameters=[p], prefix=prefix)

    def sense(action, kind, ra=(), rb=(), la='a', lb='b', outcome='', **extra):
        p = dict(action_name=action, use_sim_time=True, robot='r1', mode=mode, kind=kind, label_a=la, label_b=lb,
                 outcome=outcome, specialized_arguments=['r1'], **extra)
        if ra:
            p.update(region_a=list(ra), region_b=list(rb))
        return Node(package='watcher_demo', executable='sense_node', name=f'{action}_r1', output='screen',
                    additional_env=cascade(), parameters=[p])

    # Under the policy an action reports the event of its model that occurred;
    # in the classical runs, what the mission needs to read.
    ev = (lambda s, c='': s) if epistemic else (lambda s, c='': c)
    gate_routes = {f'route_{b}': L.route_to_gate(b)[1:] + L.route_gate_in(b)[1:] for b in ('x', 'y')}
    back_routes = {f'route_{s}{b}': L.route_back(s, b)[1:] for s in ('w', 'e') for b in ('x', 'y')}
    actions += [
        # Labels a parameter file cannot mistake for anything else: YAML reads
        # a bare y as true.
        sense('read_order', 'read_order', L.order_slot_region('x'), L.order_slot_region('y'), 'order-x',
              'order-y', ev('e-{v}', '{v}')),
        sense('read_post', 'read_post', L.duty_slot_region('w'), L.duty_slot_region('e'), 'peek-west',
              'peek-east', ev('e-{v}', '{v}')),
        # The gate is held for three seconds, so that whoever can see it sees
        # the carrier standing in it.
        drive('to_gate', ev('e-gate-{b}'), pause_places=['gate_w', 'gate_e'], pause=3.0, **gate_routes),
        sense('signal', 'signal', outcome=ev('e-say-{b}-{s}'), pause=4.0),
        drive('back_route', ev('e-back-{b}'), side_from_pose=True, **back_routes),
    ]
    if epistemic:
        for s in ('w', 'e'):
            actions.append(drive(f'to_window_{s}', route_fixed=L.route_to_window(s)[1:]))
    else:
        actions.append(drive('to_window', key_from_arg=True,
                             **{f'route_{s}': L.route_to_window(s)[1:] for s in ('w', 'e')}))

    regions = {'gate_x': L.gate_region('x'), 'gate_y': L.gate_region('y'),
               'window_w': L.window_region('w'), 'window_e': L.window_region('e'),
               'hall_w': shrink(L.hall_region('w'), 0.2), 'hall_e': shrink(L.hall_region('e'), 0.2),
               'bay_x': shrink(L.bay_region('x'), 0.2), 'bay_y': shrink(L.bay_region('y'), 0.2)}
    for r in ('r2', 'r3'):
        actions.append(Node(package='watcher_demo', executable='observer_node', name=f'observer_{r}',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'robot': r, 'carrier': 'r1', 'dir': run,
                                         'names': list(regions),
                                         'boxes': [v for b in regions.values() for v in b]}]))

    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    common = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 60.0}},
        # The domain expert checks the domain with a POPF of its own, which
        # writes to /tmp unless told otherwise; runs side by side would share it.
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
    }
    if epistemic:
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['EPISTEMIC'],
                'EPISTEMIC': {'plugin': 'plansys2/EpistemicPlanSolver', 'epddl_domain': domain,
                              'epddl_problem': problem, 'epddl_libraries': list(libraries),
                              'plank_command': '', 'task_file': '', 'heuristic': '', 'strategy': '',
                              'policy_file': '', 'action_mapping': os.path.join(SHARE, 'pddl', 'watcher-mapping.json'),
                              'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            'epistemic_state': {'ros__parameters': {'epddl_domain': domain, 'epddl_problem': problem,
                                                    'epddl_libraries': list(libraries)}},
        })
        model, state = os.path.join(SHARE, 'pddl', 'watcher-vocabulary.pddl'), 'True'
    else:
        params = dict(common, **{
            'planner': {'ros__parameters': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}},
        })
        model, state = os.path.join(SHARE, 'pddl', 'watcher-classical.pddl'), 'False'
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': model, 'params_file': filled, 'epistemic_state': state,
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='watcher_demo', executable='mission.py', name='watcher_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation])
    recorder = Node(package='watcher_demo', executable='recorder.py', name='watcher_recorder', output='screen',
                    arguments=['--dir', run])
    json.dump({'formulation': formulation}, open(os.path.join(run, 'launch.json'), 'w'))
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        OpaqueFunction(function=build),
    ])
