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
One run of the watcher that moves, headless.

    ros2 launch watcher_moves_demo watcher_launch.py dir:=RUN formulation:=opaque strategy:=chase

RUN must hold the world tools/make_world.py wrote for the run's seed.

v1        the watcher's published policy: watcher.epddl, which reads the duty
          board once, planned in the executor by ePlanSys's planner;
shortest  watcher-mobile.epddl with the gates, the watcher allowed one change of
          post: the shortest policy, planned by Aletheia with every action
          branching on what its actor observes (--subjective auto);
opaque    the same domain without the gates: the policy that keeps the order
          from a watcher that knows it (epddl-workspace/watcher-mobile/tools);
direct    POPF over the classical domain, as in watcher_demo;
discreet  the same, the gates ruled out.

strategy is the watcher's: stay at its post, or chase, changing posts the
moment the carrier starts its first drive (scripts/watcher_node.py). Under
every formulation the ally's and the watcher's observers record what their
own lasers saw of the carrier. The launch shuts down when the mission has.
"""

import json
import os
import subprocess
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

SHARE = get_package_share_directory('watcher_moves_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import layout as L  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
ROBOTS = ('r1', 'r2', 'r3')
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
SUBJECTIVE = os.environ.get('SUBJECTIVE_PLANNER',
                            os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
FORMS = ('v1', 'shortest', 'opaque', 'direct', 'discreet')


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def shrink(box, m):
    return (box[0] + m, box[1] + m, box[2] - m, box[3] - m)


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    strategy = LaunchConfiguration('strategy').perform(context)
    assert formulation in FORMS, formulation
    epistemic = formulation in ('v1', 'shortest', 'opaque')
    mobile = formulation in ('shortest', 'opaque')
    intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                                'intermediate.epddl')
    domain = problem = None
    libraries = []
    if formulation == 'v1':
        epddl = os.path.join(SHARE, 'epddl', 'v1')
        domain, problem = os.path.join(epddl, 'watcher.epddl'), os.path.join(epddl, 'instances', 'problem_1.epddl')
        libraries = [intermediate, os.path.join(epddl, 'sightlines.epddl')]
    elif mobile:
        epddl = os.path.join(SHARE, 'epddl', 'gates' if formulation == 'shortest' else 'opaque')
        domain = os.path.join(epddl, 'watcher-mobile.epddl')
        problem = os.path.join(epddl, 'instances', 'problem_1.epddl')
        libraries = [intermediate, os.path.join(epddl, 'sightlines-mobile.epddl')]

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
    places = {'places': list(L.CARRIER_PLACES), 'places_x': [nodes[p][0] for p in L.CARRIER_PLACES],
              'places_y': [nodes[p][1] for p in L.CARRIER_PLACES]}
    mode = 'epistemic' if epistemic else 'classical'

    def drive(action, outcome='', **routes):
        p = dict(roadmap, action_name=action, use_sim_time=True, robot='r1', mode=mode, outcome=outcome,
                 specialized_arguments=['r1'], **routes)
        return Node(package='watcher_moves_demo', executable='drive_node', name=f'{action}_r1', output='screen',
                    additional_env=cascade(), parameters=[p])

    def sense(action, kind, ra=(), rb=(), la='a', lb='b', outcome='', **extra):
        p = dict(action_name=action, use_sim_time=True, robot='r1', mode=mode, kind=kind, label_a=la, label_b=lb,
                 outcome=outcome, specialized_arguments=['r1'], **extra)
        if ra:
            p.update(region_a=list(ra), region_b=list(rb))
        return Node(package='watcher_moves_demo', executable='sense_node', name=f'{action}_r1', output='screen',
                    additional_env=cascade(), parameters=[p])

    ev = (lambda s, c='': s) if epistemic else (lambda s, c='': c)
    holds = {'pause_places': ['gate_w', 'gate_e'], 'pause': 3.0}
    actions += [
        # Labels a parameter file cannot mistake for anything else: YAML reads a bare y as true.
        sense('read_order', 'read_order', L.order_slot_region('x'), L.order_slot_region('y'), 'order-x', 'order-y',
              ev('e-{v}', '{v}')),
        sense('read_post', 'read_post', L.duty_slot_region('w'), L.duty_slot_region('e'), 'peek-west', 'peek-east',
              ev('e-{v}', '{v}')),
        sense('signal', 'signal', outcome=ev('e-say-{b}-{s}'), pause=4.0),
        drive('back_route', ev('e-back-{b}'), side_from_pose=True,
              **{f'route_{s}{b}': L.route_back(s, b)[1:] for s in ('w', 'e') for b in ('x', 'y')}),
        drive('to_gate', ev('e-gate-{b}'), **holds,
              **{f'route_{b}': L.route_to_gate(b)[1:] + L.route_gate_in(b)[1:] for b in ('x', 'y')}),
    ]
    if epistemic:
        for target in ('win_w', 'win_e', 'ga_w', 'ga_e'):
            routes = {f'route_from_{p}': (L.route_between(p, target)[1:] if p != target else [target])
                      for p in L.CARRIER_PLACES}
            actions.append(drive(f'drive_{target}', **places, **routes))
        actions.append(drive('gate_in', **holds, **{f'route_{b}': L.route_gate_in(b) for b in ('x', 'y')}))
        post = {s: (L.post(s)[0] - 0.35, L.post(s)[1] - 0.35, L.post(s)[0] + 0.35, L.post(s)[1] + 0.35)
                for s in ('w', 'e')}
        actions.append(sense('look', 'look', post['w'], post['e'], outcome='e-look-{s}-{v}', min_hits=2, scans=3,
                             look_for=3.0))
    else:
        actions.append(drive('to_window', key_from_arg=True,
                             **{f'route_{s}': L.route_to_window(s)[1:] for s in ('w', 'e')}))

    def ga_region(s):
        x, y = nodes[f'ga_{s}']
        return (x - 0.3, y - 0.3, x + 0.3, y + 0.3)
    regions = {'gate_x': L.gate_region('x'), 'gate_y': L.gate_region('y'),
               'window_w': L.window_region('w'), 'window_e': L.window_region('e'),
               'approach_w': ga_region('w'), 'approach_e': ga_region('e'),
               'hall_w': shrink(L.hall_region('w'), 0.2), 'hall_e': shrink(L.hall_region('e'), 0.2),
               'bay_x': shrink(L.bay_region('x'), 0.2), 'bay_y': shrink(L.bay_region('y'), 0.2)}
    for r in ('r2', 'r3'):
        actions.append(Node(package='watcher_moves_demo', executable='observer_node', name=f'observer_{r}',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'robot': r, 'carrier': 'r1', 'dir': run,
                                         'names': list(regions),
                                         'boxes': [v for b in regions.values() for v in b]}]))
    actions.append(Node(package='watcher_moves_demo', executable='watcher_node.py', name='watcher_strategy',
                        output='screen', arguments=['--strategy', strategy, '--dir', run]))

    bringup = os.path.join(get_package_share_directory('plansys2_bringup'), 'launch',
                           'plansys2_bringup_launch_monolithic.py')
    common = {
        'plansys2': {'ros__parameters': {'use_real_time': False}},
        'planner_client': {'ros__parameters': {'plan_solver_timeout': 120.0}},
        'domain_expert': {'ros__parameters': {'POPF': {'output_dir': run}}},
    }
    state = 'False'
    model = os.path.join(SHARE, 'pddl', 'watcher-classical.pddl')
    if formulation == 'v1':
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['EPISTEMIC'],
                'EPISTEMIC': {'plugin': 'plansys2/EpistemicPlanSolver', 'epddl_domain': domain,
                              'epddl_problem': problem, 'epddl_libraries': list(libraries),
                              'plank_command': '', 'task_file': '', 'heuristic': '', 'strategy': '',
                              'policy_file': '',
                              'action_mapping': os.path.join(SHARE, 'pddl', 'watcher-v1-mapping.json'),
                              'conditional_plan': 'policy'}}}})
    elif mobile:
        # The task, grounded now, for the planner the executor calls as a command.
        task_dir = os.path.join(run, 'task')
        subprocess.run([PLANK, 'export', '-d', domain, '-p', problem, '-l', *libraries, '-o', task_dir],
                       check=True, capture_output=True)
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['ALETHEIA'],
                'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver', 'command': SUBJECTIVE,
                             'arguments': '--conditional --subjective auto', 'output_dir': run,
                             'task_file': os.path.join(task_dir, 'problem_1.json'),
                             'action_mapping': os.path.join(SHARE, 'pddl', 'watcher-mobile-mapping.json'),
                             'conditional_plan': 'policy'}}}})
    else:
        params = dict(common, **{
            'planner': {'ros__parameters': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}}})
    if epistemic:
        params.update({
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            'epistemic_state': {'ros__parameters': {'epddl_domain': domain, 'epddl_problem': problem,
                                                    'epddl_libraries': list(libraries),
                                                    'subjective_branching': 'auto' if mobile else ''}}})
        state = 'True'
        model = os.path.join(SHARE, 'pddl', 'watcher-vocabulary.pddl')
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': model, 'params_file': filled, 'epistemic_state': state,
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='watcher_moves_demo', executable='mission.py', name='watcher_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation])
    recorder = Node(package='watcher_moves_demo', executable='recorder.py', name='watcher_recorder',
                    output='screen', arguments=['--dir', run])
    json.dump({'formulation': formulation, 'strategy': strategy}, open(os.path.join(run, 'launch.json'), 'w'))
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='opaque'),
        DeclareLaunchArgument('strategy', default_value='stay'),
        OpaqueFunction(function=build),
    ])
