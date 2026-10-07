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
The muddy robots: N robots at a muster point in the AWS small warehouse, each
seeing every other robot's fault lamp and not its own.

    ros2 launch muddy_robots_demo muddy_robots_launch.py                       # PA, r1 r2 r3 lit
    ros2 launch muddy_robots_demo muddy_robots_launch.py faults:=r2            # one lamp lit
    ros2 launch muddy_robots_demo muddy_robots_launch.py floor:=silent         # no public address

What runs:

  gazebo          the AWS RoboMaker small warehouse, unchanged, with the muster
                  ring, the calibration bay and the public address added
                  (tools/make_world.py), and a Waffle per robot, from
                  pass_through_demo's robot_sdf, with a status lamp on a mast
  plansys2        ePlanSys, monolithic: tools/muster.py's EPDDL grounded by
                  plank, the policy found by Aletheia, run by the epistemic
                  behaviour tree
  muster_crew     every robot's driver; the announce and bell performers;
                  assembly before planning and dismissal after it
  knowledge_view  what each robot knows of its own lamp, drawn for RViz
  mission         assembles the robots, asks for a policy, runs it or, on the
                  silent floor, rings the bell N times, and dismisses them

`faults:=` is which lamps are lit: the fleet's diagnostics, which the robots
do not read and the planner is not told. The model starts with every
pattern with a lamp lit designated.
"""

import os
import re
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

# Why the action nodes preload the system cascade_lifecycle: see
# coordinated_attack_demo's launch file, which found it.
SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'


def matching_cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def flat(items):
    return [float(v) for item in items for v in item]


def with_lamp(sdf, lit, colour):
    """The Waffle with a status lamp on a mast behind its laser, above the
    scan plane: lit red when the robot is faulty, dark grey otherwise. The
    lamp is emissive and carries no light source: Gazebo's window draws every
    light as a wireframe, and four of them over the ring hid the lamps."""
    rgb = colour if lit else (0.18, 0.18, 0.18)
    em = f'<emissive>{rgb[0]} {rgb[1]} {rgb[2]} 1</emissive>' if lit else ''
    lamp = f"""
      <visual name="lamp_mast">
        <pose>-0.12 0 0.31 0 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.22</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
      </visual>
      <visual name="status_lamp">
        <pose>-0.12 0 0.46 0 0 0</pose>
        <geometry><sphere><radius>0.075</radius></sphere></geometry>
        <material><ambient>{rgb[0]} {rgb[1]} {rgb[2]} 1</ambient>
          <diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 1</diffuse>{em}</material>
      </visual>
"""
    return sdf.replace('<link name="base_link">', '<link name="base_link">' + lamp, 1)


def setup(context, *args, **kwargs):
    share = get_package_share_directory('muddy_robots_demo')
    pt_share = get_package_share_directory('pass_through_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    sys.path.insert(0, os.path.join(share, 'tools'))
    import layout as L   # noqa: E402
    import muster   # noqa: E402
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'pass_through_robot_sdf', os.path.join(pt_share, 'tools', 'robot_sdf.py'))
    robot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot)

    floor = LaunchConfiguration('floor').perform(context)
    if floor not in ('pa', 'silent'):
        raise RuntimeError(f'floor:={floor} is not pa or silent')
    n = int(LaunchConfiguration('robots').perform(context))
    fleet = L.robots(n)
    agents = list(fleet)
    faults = [f for f in LaunchConfiguration('faults').perform(context).split(',') if f]
    for f in faults:
        if f not in fleet:
            raise RuntimeError(f'faults:= names {f}, which is not one of {agents}')
    if not faults:
        raise RuntimeError('faults:= names no robot; the supervisor knows at least one lamp is lit')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    tmp = os.path.join(tempfile.gettempdir(), f'muddy_robots_n{n}')
    world = os.path.join(tmp, 'muster.world')
    lamps = os.path.join(tmp, 'lamps')
    os.makedirs(tmp, exist_ok=True)
    subprocess.run([sys.executable, os.path.join(share, 'tools', 'make_world.py'),
                    '--aws', aws, '--out', world, '--lamps', lamps], check=True)
    written = muster.write(tmp, n, executor=True)
    floorplan = L.floorplan_yaml()

    intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'),
                                'libraries', 'intermediate.epddl')
    with open(os.path.join(share, 'params', 'muddy_robots.yaml')) as fh:
        params = fh.read()
    params = (params.replace('EPDDL_DOMAIN', written['domain'])
                    .replace('EPDDL_PROBLEM', written[floor])
                    .replace('MAPPING_FILE', written['mapping'])
                    .replace('INTERMEDIATE_LIBRARY', intermediate))
    filled = tempfile.NamedTemporaryFile('w', suffix='_muddy_robots.yaml', delete=False)
    filled.write(params)
    filled.close()

    gazebo = [ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world], output='screen')]
    if gui:
        gazebo.append(ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen'))

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf) as fh:
        urdf_text = fh.read()

    per_robot = []
    for k, agent in enumerate(agents):
        ns, x, y, colour = fleet[agent]
        sdf = tempfile.NamedTemporaryFile('w', suffix=f'_{ns}.sdf', delete=False)
        sdf.write(with_lamp(robot.robot_sdf(ns, 8.0, 720, colour=colour), agent in faults,
                            L.LAMP_LIT))
        sdf.close()
        per_robot.append(TimerAction(period=10.0 + 4.0 * k, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{ns}',
                 output='screen',
                 arguments=['-entity', ns, '-file', sdf.name,
                            '-x', str(x), '-y', str(y), '-z', '0.01']),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': True,
                              'robot_description': urdf_text.replace('${namespace}', ns + '/')}]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'map_to_{ns}_odom', output='screen',
                 arguments=['--frame-id', 'map', '--child-frame-id', f'{ns}/odom']),
        ]))

    muster_poses = [L.muster_spot(k, n) for k in range(n)]
    crew = Node(
        package='muddy_robots_demo', executable='muster_crew', output='screen',
        additional_env=matching_cascade(),
        parameters=[{'floorplan': floorplan, 'agents': agents, 'namespaces': agents,
                     'muster': flat(muster_poses),
                     'bay': flat([L.bay_spot(k) for k in range(len(L.BAY))]),
                     'centre': list(L.MUSTER),
                     'stations': flat([(fleet[a][1], fleet[a][2], 0.0) for a in agents]),
                     'faults': faults, 'lamps': lamps,
                     'pa': [L.PA[0], L.PA[1], 2.48],
                     'bell': [L.MUSTER[0], L.MUSTER[1], 0.08],
                     'rate': 10.0}])

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': written['pddl'], 'params_file': filled.name,
                          'epistemic_state': 'True'}.items())

    colours = [c for a in agents for c in fleet[a][3]]
    view = Node(
        package='muddy_robots_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'floorplan': floorplan, 'agents': agents, 'colours': colours,
                     'faults': faults, 'muster': [L.MUSTER[0], L.MUSTER[1], L.RING],
                     'pa': list(L.PA), 'bay': list(L.BAY_PAD)}])

    shots = [f'{a}={a}' for a in agents]
    shots += [f'{name}={L.shot_text(pose)}' for name, pose in L.SHOTS.items()]
    director = Node(
        package='coordinated_attack_demo', executable='camera_director.py', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        arguments=['--ros-args', '-r', '/coordinated_attack/shot:=/muddy_robots/shot'],
        parameters=[{'follow_file': LaunchConfiguration('follow_file'),
                     'initial': L.shot_text(L.OPENING_SHOT), 'shots': shots}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}])

    budget = float(re.search(r'plan_solver_timeout:\s*([\d.]+)', params).group(1))
    mission = Node(
        package='muddy_robots_demo', executable='muddy_robots_mission', output='screen',
        arguments=['--ros-args', '-p', f'plan_solver_timeout:={budget + 15.0}'],
        parameters=[{'epddl_problem': written[floor], 'action_mapping': written['mapping'],
                     'floor': floor, 'policy_out': LaunchConfiguration('policy_out'),
                     'hold': float(LaunchConfiguration('hold').perform(context))}])

    staged = []
    if LaunchConfiguration('shutdown').perform(context).lower() == 'true':
        staged.append(RegisterEventHandler(OnProcessExit(
            target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))

    return (gazebo + per_robot + [plansys2, view, director, rviz] +
            [TimerAction(period=26.0, actions=[crew])] +
            [TimerAction(period=float(LaunchConfiguration('start_after').perform(context)),
                         actions=[mission])] + staged)


def generate_launch_description():
    share = get_package_share_directory('muddy_robots_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    xl = get_package_share_directory('warehouse_xl_rmf_demo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='pa',
                              description='pa: the hall has a public address; silent: it has none.'),
        DeclareLaunchArgument('robots', default_value='4'),
        DeclareLaunchArgument('faults', default_value='r1,r2,r3',
                              description='The robots whose lamps are lit, comma separated.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('follow_file', default_value='/tmp/muddy_robots_follow'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'muddy_robots.rviz')),
        DeclareLaunchArgument('policy_out', default_value=''),
        DeclareLaunchArgument('hold', default_value='0.0'),
        DeclareLaunchArgument('start_after', default_value='40.0'),
        DeclareLaunchArgument('shutdown', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join([
            os.path.join(xl, 'models'), os.path.join(aws, 'models'), os.path.join(tb3, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', ':'.join([
            aws, '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        OpaqueFunction(function=setup),
    ])
