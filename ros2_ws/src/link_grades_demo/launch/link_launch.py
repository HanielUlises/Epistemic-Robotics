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
One run of the link grades, headless.

    ros2 launch link_grades_demo link_launch.py dir:=RUN formulation:=epistemic grade:=partial seed:=1

RUN must hold the world tools/make_world.py wrote for the run's seed.

The link between the robots is epistemic_comm's: comm_monitor declares it down
and up on the grade's schedule (tools/grades.py), and six link_gates carry
across it what one robot sends the other -- its map, what its map decides,
and its reports -- and drop all of it, both ways at once, while it is down.
Each robot's mapper (scripts/mapper_node.py) reads the other's map only from
its gate.

epistemic, epistemic_open  Aletheia over link-grades.epddl, the policy
           executed by ePlanSys with independent actions of the two robots
           dispatched together;
visit, report, replan  POPF over link-classical.pddl (scripts/mission.py).

The launch shuts down when the mission has.
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

SHARE = get_package_share_directory('link_grades_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import grades as R  # noqa: E402
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402

SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'
ROBOTS = ('r1', 'r2')
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
GATES = [('/r1/grid', '/r2/peer_grid', 'nav_msgs/msg/OccupancyGrid'),
         ('/r2/grid', '/r1/peer_grid', 'nav_msgs/msg/OccupancyGrid'),
         ('/r1/decision', '/r2/peer_decision', 'std_msgs/msg/String'),
         ('/r2/decision', '/r1/peer_decision', 'std_msgs/msg/String'),
         ('/r1/report', '/r2/report_in', 'std_msgs/msg/String'),
         ('/r2/report', '/r1/report_in', 'std_msgs/msg/String')]


def cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def build(context):
    run = LaunchConfiguration('dir').perform(context)
    formulation = LaunchConfiguration('formulation').perform(context)
    grade = LaunchConfiguration('grade').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    assert formulation in R.FORMULATIONS, formulation
    assert grade in R.GRADES, grade
    epistemic = formulation.startswith('epistemic')
    mode = 'epistemic' if epistemic else 'classical'

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
            Node(package='tf2_ros', executable='static_transform_publisher', name=f'map_to_{r}_odom', output='screen',
                 arguments=['--frame-id', 'map', '--child-frame-id', f'{r}/odom']),
            Node(package='link_grades_demo', executable='mapper_node.py', name=f'mapper_{r}', output='screen',
                 arguments=['--robot', r, '--dir', run]),
        ]

    # The link.
    t_disc, t_recon = R.schedule(grade, seed)
    actions.append(Node(package='epistemic_comm', executable='comm_monitor', name='comm_monitor', output='screen',
                        parameters=[{'use_sim_time': True, 'pair': ['r1', 'r2'], 't_disc': t_disc,
                                     't_recon': t_recon}]))
    for i, (src, dst, kind) in enumerate(GATES):
        actions.append(Node(package='epistemic_comm', executable='link_gate', name=f'link_gate_{i}',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'input': src, 'output': dst, 'type': kind,
                                         'depth': 2 if kind.endswith('OccupancyGrid') else 10}]))

    # The actions.
    nodes, _ = L.roadmap()
    roadmap = {'roadmap_names': list(nodes), 'roadmap_x': [p[0] for p in nodes.values()],
               'roadmap_y': [p[1] for p in nodes.values()]}
    for r in ROBOTS:
        actions.append(Node(package='link_grades_demo', executable='drive_node', name=f'{P.DRIVE[r]}_{r}',
                            output='screen', additional_env=cascade(),
                            parameters=[dict(roadmap, action_name=P.DRIVE[r], use_sim_time=True, robot=r,
                                             mode=mode, outcome='', specialized_arguments=[r],
                                             route_fixed=L.ROUTE[r][1:])]))
        actions.append(Node(package='link_grades_demo', executable='wait_node', name=f'inspect_{r}',
                            output='screen', additional_env=cascade(),
                            parameters=[dict(action_name='inspect', use_sim_time=True, robot=r, kind='inspect',
                                             outcome='e-inspect-{v}' if epistemic else '', wait=30.0,
                                             specialized_arguments=[r])]))
    if epistemic:
        actions.append(Node(package='link_grades_demo', executable='wait_node', name='share_r1',
                            output='screen', additional_env=cascade(),
                            parameters=[dict(action_name='share', use_sim_time=True, robot='r1', kind='message',
                                             outcome='e-share-{v}', wait=R.ACK_WAIT, ack=True)]))
    else:
        actions.append(Node(package='link_grades_demo', executable='wait_node', name='report_r1',
                            output='screen', additional_env=cascade(),
                            parameters=[dict(action_name='report', use_sim_time=True, robot='r1', kind='message',
                                             outcome='', wait=R.ACK_WAIT, ack=formulation == 'replan')]))

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
        domain = os.path.join(epddl, 'link-grades.epddl')
        intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'), 'libraries',
                                    'intermediate.epddl')
        libraries = [intermediate, os.path.join(epddl, 'link.epddl')]
        # The four tasks, grounded now: the link up, the link down, the share
        # lost, and the link's state open. The mission copies the one it plans for to current.json.
        for which in ('up', 'down', 'lost', 'open'):
            subprocess.run([PLANK, 'export', '-d', domain, '-p',
                            os.path.join(epddl, 'instances', f'problem_{which}.epddl'), '-l', *libraries,
                            '-o', os.path.join(run, 'task', which)], check=True, capture_output=True)
        current = os.path.join(run, 'task', 'current.json')
        subprocess.run(['cp', os.path.join(run, 'task', 'up', 'problem_up.json'), current], check=True)
        params = dict(common, **{
            'planner': {'ros__parameters': {
                'plan_solver_timeout': 120.0, 'plan_solver_plugins': ['ALETHEIA'],
                'ALETHEIA': {'plugin': 'plansys2/AletheiaPlanSolver', 'command': PLANNER,
                             'arguments': '--conditional', 'output_dir': run, 'task_file': current,
                             'action_mapping': os.path.join(SHARE, 'pddl', 'link-mapping.json'),
                             'conditional_plan': 'policy'}}},
            'executor': {'ros__parameters': {'bt_builder_plugin': 'EpistemicBTBuilder',
                                             'bt_node_plugins': ['libplansys2_epistemic_bt_nodes.so']}},
            # Read by the tree builder from a node of its own name: the two
            # robots' independent actions dispatched together, as POPF's are.
            'epistemic_bt_builder': {'ros__parameters': {'parallel_dispatch': True}},
            'epistemic_state': {'ros__parameters': {'task_file': current}}})
        state = 'True'
        model = os.path.join(SHARE, 'pddl', 'link-vocabulary.pddl')
    else:
        params = dict(common, **{
            'planner': {'ros__parameters': {'plan_solver_timeout': 60.0, 'plan_solver_plugins': ['POPF'],
                                            'POPF': {'plugin': 'plansys2/POPFPlanSolver', 'output_dir': run}}}})
        state = 'False'
        model = os.path.join(SHARE, 'pddl', 'link-classical.pddl')
    filled = os.path.join(run, 'params.yaml')
    open(filled, 'w').write(yaml.safe_dump(params, sort_keys=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'model_file': model, 'params_file': filled, 'epistemic_state': state,
                          'epistemic_perception': 'False'}.items()))
    mission = Node(package='link_grades_demo', executable='mission.py', name='link_mission', output='screen',
                   arguments=['--dir', run, '--formulation', formulation, '--grade', grade])
    recorder = Node(package='link_grades_demo', executable='recorder.py', name='link_recorder', output='screen',
                    arguments=['--dir', run])
    json.dump({'formulation': formulation, 'grade': grade, 'seed': seed, 't_disc': t_disc, 't_recon': t_recon,
               'ack_wait': R.ACK_WAIT, 'start_at': R.START_AT},
              open(os.path.join(run, 'launch.json'), 'w'), indent=1)
    done = RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=lambda e, c: [EmitEvent(event=Shutdown(reason='mission over'))]))
    return actions + [recorder, mission, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir'),
        DeclareLaunchArgument('formulation', default_value='epistemic'),
        DeclareLaunchArgument('grade', default_value='full'),
        DeclareLaunchArgument('seed', default_value='1'),
        OpaqueFunction(function=build),
    ])
