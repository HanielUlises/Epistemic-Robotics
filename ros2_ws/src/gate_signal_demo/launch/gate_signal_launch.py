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
One run of the gate signal, headless.

    ros2 launch gate_signal_demo gate_signal_launch.py dir:=RUN formulation:=epistemic

RUN must hold the world tools/make_world.py wrote for the run's seed.

epistemic    one PlanSys2, ePlanSys's planner over the EPDDL task, the
             epistemic executor dispatching independent actions together, and
             the action nodes of both robots, each taking only its own robot's
             actions;
optimistic   two PlanSys2 instances, /r1 and /r2, each robot planning for
cautious     itself with POPF over the classical domain and knowing nothing of
             the other; the formulation names r2's strategy (scripts/mission.py).

The launch shuts down when every mission has ended.
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

SHARE = get_package_share_directory('gate_signal_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    assert formulation in ('epistemic', 'optimistic', 'cautious'), formulation
    epistemic = formulation == 'epistemic'
    epddl = os.path.join(SHARE, 'epddl')
    domain = os.path.join(epddl, 'gate-signal.epddl')
    problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
    # The domain declares two libraries, ePlanSys's intermediate one and its
    # own, which adds the semi-private ontic action; naming any library means
    # naming every one.
    libraries = [os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                              'intermediate.epddl'), os.path.join(epddl, 'gate.epddl')]

    models = os.path.join(run, 'models')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    actions = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', os.path.join(run, 'world.sdf')],
        additional_env={'GAZEBO_MODEL_PATH': models + ':' + tb3 + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')},
        output='screen')]
    urdf = open(os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf',
                             'turtlebot3_burger.urdf')).read().replace('${namespace}', '')
    for r in ('r1', 'r2'):
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=r, name='robot_state_publisher', output='screen',
                            parameters=[{'use_sim_time': True, 'robot_description': urdf,
                                         'frame_prefix': f'{r}/'}]))
        # Each robot's odometry is the world pose, so its odom frame is the map frame.
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name=f'map_to_{r}_odom', output='screen',
                            arguments=['--frame-id', 'map', '--child-frame-id', f'{r}/odom']))

    nodes, edges = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()]}
    mode = 'epistemic' if epistemic else 'classical'

    def ns(r):
        return '' if epistemic else r

    def drive(r, action, outcome='', bay_from='order'):
        p = dict(roadmap, action_name=action, use_sim_time=True, robot=r, mode=mode, outcome=outcome,
                 bay_from=bay_from, specialized_arguments=[r],
                 route_x=P.route(action, 'x')[1:], route_y=P.route(action, 'y')[1:],
                 gate_x=list(L.gate('x')), gate_y=list(L.gate('y')))
        return Node(package='gate_signal_demo', executable='drive_node', namespace=ns(r),
                    name=f'{action}_{r}', output='screen', additional_env=cascade(), parameters=[p])

    def sense(r, action, kind, rx, ry, outcome='', **extra):
        p = dict(action_name=action, use_sim_time=True, robot=r, kind=kind, region_x=list(rx), region_y=list(ry),
                 outcome=outcome, specialized_arguments=[r], **extra)
        return Node(package='gate_signal_demo', executable='sense_node', namespace=ns(r),
                    name=f'{action}_{r}', output='screen', additional_env=cascade(), parameters=[p])

    def shrink(box, m):
        return (box[0] + m, box[1] + m, box[2] - m, box[3] - m)

    # Under the policy an action reports the event of its model that occurred;
    # in the classical runs, what the mission needs to read: the bay of the
    # order, and whether the bay looked over holds anything.
    ev = (lambda s, c='': s) if epistemic else (lambda s, c='': c)
    look_x, look_y = shrink(L.bay_region('x'), 0.3), shrink(L.bay_region('y'), 0.3)
    actions += [
        sense('r1', 'read_order', 'read_order', L.order_slot_region('x'), L.order_slot_region('y'),
              ev('e-order-{b}', '{b}'), dwell=20.0),
        drive('r1', 'to_gate', ev('e-gate-{b}')),
        drive('r1', 'back_route', ev('e-back-{b}')),
        drive('r1', 'enter_bay', ev('e-enter-{b}'), bay_from='gate'),
        sense('r1', 'unload', 'unload', L.bay_region('x'), L.bay_region('y'), ev('e-drop-carrier-{b}')),
        drive('r2', 'go'),
        sense('r2', 'unload', 'unload', L.bay_region('x'), L.bay_region('y')),
    ]
    if epistemic:
        actions.append(sense('r2', 'watch', 'watch', L.gate_region('x'), L.gate_region('y'), 'e-seen-{b}',
                             dwell=120.0))
    else:
        actions += [drive('r2', 'cross'),
                    sense('r2', 'look_bay', 'look_bay', look_x, look_y, '{s}', look_for=3.0)]

    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    if epistemic:
        params = {
            'plansys2': {'ros__parameters': {'use_real_time': False}},
            'planner_client': {'ros__parameters': {'plan_solver_timeout': 60.0}},
            # The domain expert checks the domain with a POPF of its own, which
            # writes to /tmp unless told otherwise; runs side by side would share it.
            'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['EPISTEMIC'],
                'EPISTEMIC': {'plugin': 'plansys2/EpistemicPlanSolver', 'epddl_domain': domain,
                              'epddl_problem': problem, 'epddl_libraries': list(libraries),
                              'plank_command': '', 'task_file': '', 'heuristic': '', 'strategy': '',
                              'policy_file': '', 'action_mapping': os.path.join(SHARE, 'pddl', 'gate-signal-mapping.json'),
                              'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # Independent runs of the policy, one robot's and the other's, together.
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {'epddl_domain': domain, 'epddl_problem': problem,
                                                    'epddl_libraries': list(libraries)}},
        }
        filled = os.path.join(run, 'params.yaml')
        open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'gate-signal-vocabulary.pddl'),
                              'params_file': filled, 'epistemic_state': 'True',
                              'epistemic_perception': 'False'}.items()))
        missions = [Node(package='gate_signal_demo', executable='mission.py', name='gate_mission',
                         output='screen', arguments=['--dir', run, '--formulation', formulation])]
    else:
        missions = []
        for r in ('r1', 'r2'):
            params = {f'/{r}/{n}': {'ros__parameters': v} for n, v in {
                'plansys2': {'use_real_time': False},
                'planner_client': {'plan_solver_timeout': 60.0},
                'domain_expert': {'POPF': {'output_dir': os.path.join(run, r)}},
                'planner': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': os.path.join(run, r)}},
            }.items()}
            os.makedirs(os.path.join(run, r), exist_ok=True)
            filled = os.path.join(run, f'params_{r}.yaml')
            open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup),
                launch_arguments={'model_file': os.path.join(SHARE, 'pddl', 'gate-signal-classical.pddl'),
                                  'params_file': filled, 'namespace': r, 'epistemic_state': 'False',
                                  'epistemic_perception': 'False'}.items()))
            missions.append(Node(package='gate_signal_demo', executable='mission.py', namespace=r,
                                 name='gate_mission', output='screen',
                                 arguments=['--dir', run, '--formulation', formulation, '--robot', r,
                                            '--deadline', LaunchConfiguration('deadline').perform(context)]))
    recorder = Node(package='gate_signal_demo', executable='recorder.py', name='gate_recorder', output='screen',
                    arguments=['--dir', run])
    json.dump({'formulation': formulation}, open(os.path.join(run, 'launch.json'), 'w'))
    left = {'n': len(missions)}

    def one_ended(event, context):
        left['n'] -= 1
        return [EmitEvent(event=Shutdown(reason='missions over'))] if left['n'] <= 0 else []
    handlers = [RegisterEventHandler(OnProcessExit(target_action=m, on_exit=one_ended)) for m in missions]
    return actions + [recorder] + missions + handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        DeclareLaunchArgument('deadline', default_value='50.0'),
        OpaqueFunction(function=build),
    ])
