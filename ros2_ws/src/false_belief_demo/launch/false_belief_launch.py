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
A false belief: a crate moved while the robot that stored it is away.

    ros2 launch false_belief_demo false_belief_launch.py                  # told floor
    ros2 launch false_belief_demo false_belief_launch.py floor:=untold
    ros2 launch false_belief_demo false_belief_launch.py floor:=doubt

What runs, per robot:

  gazebo                a Waffle in its own namespace, from pass_through_demo's
                        robot_sdf
  robot_state_publisher its transforms, prefixed
  static transform      map -> rN/odom, the identity: Gazebo's diff drive
                        reports odometry in the world frame

and once for the mission:

  plansys2       ePlanSys, monolithic: the EPDDL grounded by plank with the
                 intermediate and beliefs libraries, the policy found by
                 Aletheia with consistent beliefs required, executed by the
                 epistemic behaviour tree
  go_dock        the picker drives to its dock, out of sight of both bays
  relocate       the mover carries the crate from one bay to the other
  report         the mover tells the picker where the crate went
  look, fetch    the picker looks into a bay, or fetches the crate from it
  knowledge_view the crate and each robot's belief about it, drawn for RViz
  mission        asks for a policy; on the untold floor runs what the picker
                 would do on its own beliefs

`floor:=` picks the EPDDL problem. The floors share one world: whether the
robots have a radio, and what the picker makes of a relocation it does not
see, are not in the world.
"""

import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The action nodes derive from plansys2's ActionExecutorClient, built by
# ePlanSys against the rclcpp_cascade_lifecycle header Humble ships while its
# overlay loads a newer library with two members more; coordinated_attack_demo's
# launch states the consequence. Loading the library the headers describe
# removes the mismatch.
SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'


def matching_cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def flat(items):
    return [float(v) for item in items for v in item]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('false_belief_demo')
    pt_share = get_package_share_directory('pass_through_demo')
    sys.path.insert(0, os.path.join(share, 'tools'))
    import layout as L   # noqa: E402
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'pass_through_robot_sdf', os.path.join(pt_share, 'tools', 'robot_sdf.py'))
    robot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot)

    floor = LaunchConfiguration('floor').perform(context)
    if floor not in ('told', 'untold', 'doubt'):
        raise RuntimeError(f'floor:={floor} is not told, untold or doubt')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    world = os.path.join(tempfile.gettempdir(), 'false_belief.world')
    subprocess.run([sys.executable, os.path.join(share, 'tools', 'make_world.py'),
                    '--out', world], check=True)

    floorplan = os.path.join(share, 'maps', 'floorplan.yaml')
    domain = os.path.join(share, 'epddl', 'false-belief.epddl')
    problem = os.path.join(share, 'epddl', f'{floor}.epddl')
    beliefs = os.path.join(share, 'epddl', 'beliefs.epddl')
    intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'),
                                'libraries', 'intermediate.epddl')
    mapping = os.path.join(share, 'pddl', 'false-belief-mapping.json')
    model = os.path.join(share, 'pddl', 'false-belief.pddl')

    with open(os.path.join(share, 'params', 'false_belief.yaml')) as fh:
        params = (fh.read().replace('EPDDL_DOMAIN', domain).replace('EPDDL_PROBLEM', problem)
                  .replace('MAPPING_FILE', mapping).replace('INTERMEDIATE_LIBRARY', intermediate)
                  .replace('BELIEFS_LIBRARY', beliefs))
    filled = tempfile.NamedTemporaryFile('w', suffix='_false_belief.yaml', delete=False)
    filled.write(params)
    filled.close()

    agents = list(L.ROBOTS)
    namespaces = [L.ROBOTS[a][0] for a in agents]
    picker, mover = agents
    bays = list(L.BAYS)

    gazebo = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen')]
    if gui:
        gazebo.append(ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen'))

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf) as fh:
        urdf_text = fh.read()

    per_robot = []
    for k, agent in enumerate(agents):
        ns, x, y, rng, samples, colour = L.ROBOTS[agent]
        sdf = tempfile.NamedTemporaryFile('w', suffix=f'_{ns}.sdf', delete=False)
        sdf.write(robot.robot_sdf(ns, rng, samples, colour=colour))
        sdf.close()
        yaw = L.facing(agent) if agent == picker else 0.0
        per_robot.append(TimerAction(period=8.0 + 4.0 * k, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{ns}',
                 output='screen',
                 arguments=['-entity', ns, '-file', sdf.name,
                            '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': True,
                              'robot_description': urdf_text.replace('${namespace}', ns + '/')}]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'map_to_{ns}_odom', output='screen',
                 arguments=['--frame-id', 'map', '--child-frame-id', f'{ns}/odom']),
        ]))

    centres = flat([L.bay_centre(b) for b in bays])
    crate = {'bays': bays, 'bay_centres': centres, 'crate_size': list(L.CRATE_SIZE),
             'carry_z': L.CARRY_Z, 'take_gap': L.TAKE_GAP}
    common = {'floorplan': floorplan}
    performers = [
        Node(package='false_belief_demo', executable='go_dock_action',
             additional_env=matching_cascade(), output='screen',
             parameters=[{**common, 'agent': picker, 'ns': L.ROBOTS[picker][0],
                          'dock': [L.DOCK[0], L.DOCK[1], L.DOCK_YAW],
                          'bays': bays, 'bay_centres': centres,
                          'action_name': 'go_dock', 'specialized_arguments': [picker],
                          'rate': 10.0}]),
        Node(package='false_belief_demo', executable='relocate_action',
             additional_env=matching_cascade(), output='screen',
             parameters=[{**common, **crate, 'agent': mover, 'ns': L.ROBOTS[mover][0],
                          'observer': picker, 'observer_ns': L.ROBOTS[picker][0],
                          'mouths': flat([L.mouth(b, mover) for b in bays]),
                          'rest': list(L.MOVER_REST), 'yaw_in': L.facing(mover),
                          'action_name': 'relocate', 'specialized_arguments': [mover, '', ''],
                          'rate': 10.0}]),
        Node(package='false_belief_demo', executable='report_action',
             additional_env=matching_cascade(), output='screen',
             parameters=[{'action_name': 'report', 'transfer_seconds': 4.0, 'rate': 10.0}]),
    ]
    for kind in ('look', 'fetch'):
        performers.append(Node(
            package='false_belief_demo', executable='bay_action',
            additional_env=matching_cascade(), output='screen', arguments=['--kind', kind],
            parameters=[{**common, **crate, 'agent': picker, 'ns': L.ROBOTS[picker][0],
                         'mouths': flat([L.mouth(b, picker) for b in bays]),
                         'drop': list(L.DROP), 'yaw_in': L.facing(picker),
                         'look_threshold': L.LOOK_THRESHOLD,
                         'action_name': kind, 'specialized_arguments': [picker, ''],
                         'rate': 10.0}]))

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': model, 'params_file': filled.name,
                          'epistemic_state': 'True'}.items())

    colours = [c for a in agents for c in L.ROBOTS[a][5]]
    view = Node(
        package='false_belief_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'floorplan': floorplan, 'agents': agents, 'namespaces': namespaces,
                     'colours': colours, 'bays': bays, 'bay_centres': centres,
                     'crate_start': list(L.crate_rest(L.CRATE_START)),
                     'crate_size': list(L.CRATE_SIZE),
                     'dock': list(L.DOCK), 'drop': list(L.DROP)}])

    shots = [f'{a}={L.ROBOTS[a][0]}' for a in agents]
    shots += [f'{name}={L.shot_text(pose)}' for name, pose in L.SHOTS.items()]
    director = Node(
        package='false_belief_demo', executable='camera_director.py', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{'follow_file': LaunchConfiguration('follow_file'),
                     'initial': L.shot_text(L.OPENING_SHOT), 'shots': shots}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}])

    mission = Node(
        package='false_belief_demo', executable='false_belief_mission',
        output='screen',
        parameters=[{'epddl_problem': problem, 'action_mapping': mapping, 'floor': floor,
                     'picker': picker, 'mover': mover,
                     'crate_from': L.CRATE_START, 'crate_to': bays[1],
                     'policy_out': LaunchConfiguration('policy_out'),
                     'plan_only': LaunchConfiguration('plan_only'),
                     'hold': float(LaunchConfiguration('hold').perform(context))}])

    staged = []
    if LaunchConfiguration('shutdown').perform(context).lower() == 'true':
        staged.append(RegisterEventHandler(OnProcessExit(
            target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))

    return (gazebo + per_robot + [plansys2, view, director, rviz] +
            [TimerAction(period=22.0, actions=performers)] +
            [TimerAction(period=float(LaunchConfiguration('start_after').perform(context)),
                         actions=[mission])] + staged)


def generate_launch_description():
    share = get_package_share_directory('false_belief_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    xl = get_package_share_directory('warehouse_xl_rmf_demo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='told',
                              description='told: the mover can report the move; untold: it '
                                          'cannot; doubt: no report, but the picker allows that '
                                          'the crate may have been moved.'),
        DeclareLaunchArgument('gui', default_value='true', description='gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Write the current shot to follow_file.'),
        DeclareLaunchArgument('follow_file', default_value='/tmp/false_belief_follow'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'false_belief.rviz')),
        DeclareLaunchArgument('policy_out', default_value=''),
        DeclareLaunchArgument('plan_only', default_value='false'),
        DeclareLaunchArgument('hold', default_value='0.0',
                              description='Seconds the mission waits before planning.'),
        DeclareLaunchArgument('start_after', default_value='35.0',
                              description='Seconds before the mission node starts.'),
        DeclareLaunchArgument('shutdown', default_value='false',
                              description='Bring everything down when the mission ends.'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join([
            os.path.join(xl, 'models'), os.path.join(aws, 'models'), os.path.join(tb3, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', ':'.join([
            aws, '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        OpaqueFunction(function=setup),
    ])
