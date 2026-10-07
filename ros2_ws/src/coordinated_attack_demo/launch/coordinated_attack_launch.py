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
The coordinated attack: two robots, two stands, one work order, and a lift
that needs common knowledge of which stand.

    ros2 launch coordinated_attack_demo coordinated_attack_launch.py                 # beacon floor
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py floor:=radio
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py floor:=sight
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py floor:=blind
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py order:=s2
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py robots:=4 messages:=3
    ros2 launch coordinated_attack_demo coordinated_attack_launch.py robots:=4 messages:=3 floor:=radio

What runs, per robot:

  gazebo                a Waffle in its own namespace, from pass_through_demo's
                        robot_sdf
  robot_state_publisher its transforms, prefixed
  static transform      map -> rN/odom, the identity: Gazebo's diff drive
                        reports odometry in the world frame, and with no SLAM
                        here nothing else would place the robot in the map

and once for the mission:

  plansys2       ePlanSys, monolithic: the EPDDL grounded by plank with the
                 intermediate and lossy libraries, the policy found by Aletheia,
                 executed by the epistemic behaviour tree
  read_order     south: drive to the terminal and read the order
  go_view        one per robot: drive to the viewpoint of the beacon
  radio          one per message level: tell, ack, ack2, ack3
  signal         light the beacon's tier for a stand; on the sight and blind
                 floors, signal_to, which also says whether the listener saw it
  sight          on the sight and blind floors: the two robots look at each
                 other through t2, each with its own laser
  lift           both robots, under the two ends of a load, at one start
  knowledge_view the model and the depth of mutual knowledge, drawn for RViz
  mission        asks for a policy; on the radio floor runs the protocol, and
                 on the blind floor the policy without its sighting

`floor:=` picks the floor and the EPDDL problem together. beacon and radio
differ in the beacon and in nothing else, and share the domain in which where
each robot stands is announced. sight and blind take the positions domain, in
which it is not: sight is the beacon floor, and blind the same with crates
across t2 that hide the two viewpoints from each other. `order:=` is what the
work order says.

`robots:=` and `messages:=` run tools/scaled.py's domain instead of the
published one: up to four robots, south2 and north2 joining south and north,
and that many message levels. The domain, the two floors, the action mapping
and the classical model are written at launch, as the world is. On the radio
floor the mission runs the protocol in protocols/radio-n<robots>-m<messages>.json,
which tools/ladder.py had the planner find, or the one `protocol:=` names. The
positions floors are for two robots.
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


# The action nodes derive from plansys2's ActionExecutorClient, which derives
# from rclcpp_cascade_lifecycle's CascadeLifecycleNode. ePlanSys builds every
# one of its packages against the header /opt/ros/humble ships, and its overlay
# puts the rolling-devel library it vendors first on the library path; that
# library's class has two members more. Each action node was therefore built
# with one layout and constructed by the other, and destroying it read its
# members at the wrong offsets: every action node exited on signal 11 at
# shutdown, after the mission had reported. Loading the library the headers
# describe removes the mismatch, and the nodes exit cleanly.
SYSTEM_CASCADE = '/opt/ros/humble/lib/librclcpp_cascade_lifecycle.so'


def matching_cascade():
    return {'LD_PRELOAD': SYSTEM_CASCADE} if os.path.exists(SYSTEM_CASCADE) else {}


def flat(items):
    return [float(v) for item in items for v in item]


def rviz_for(fleet, source):
    """The RViz configuration with a scan and a route for every robot: the
    published one draws r1 and r2, and a copy is written with r2's two
    displays cloned for each further robot, in its colour."""
    import yaml
    with open(source) as fh:
        config = yaml.safe_load(fh)
    displays = config['Visualization Manager']['Displays']
    for agent, (ns, *_rest, colour) in fleet.items():
        if any(d.get('Name') == f'scan {ns}' for d in displays):
            continue
        rgb = '; '.join(str(int(round(255 * c))) for c in colour)
        for kind in ('scan', 'route'):
            model = next(d for d in displays if d.get('Name') == f'{kind} r2')
            clone = yaml.safe_load(yaml.safe_dump(model))
            clone['Name'] = f'{kind} {ns}'
            clone['Color'] = rgb
            clone['Topic']['Value'] = f'/{ns}/{kind}'
            last = max(i for i, d in enumerate(displays)
                       if str(d.get('Name', '')).startswith(f'{kind} r'))
            displays.insert(last + 1, clone)
    out = tempfile.NamedTemporaryFile('w', suffix='_coordinated_attack.rviz', delete=False)
    yaml.safe_dump(config, out, sort_keys=False)
    out.close()
    return out.name


def setup(context, *args, **kwargs):
    share = get_package_share_directory('coordinated_attack_demo')
    pt_share = get_package_share_directory('pass_through_demo')
    sys.path.insert(0, os.path.join(share, 'tools'))
    import layout as L   # noqa: E402
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'pass_through_robot_sdf', os.path.join(pt_share, 'tools', 'robot_sdf.py'))
    robot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot)

    floor = LaunchConfiguration('floor').perform(context)
    if floor not in ('beacon', 'radio', 'sight', 'blind'):
        raise RuntimeError(f'floor:={floor} is not beacon, radio, sight or blind')
    positions = floor in ('sight', 'blind')
    # The world the floor is built from: the sight floor is the beacon floor.
    built = 'beacon' if floor == 'sight' else floor
    order = LaunchConfiguration('order').perform(context)
    if order not in L.STANDS:
        raise RuntimeError(f'order:={order} is not one of {sorted(L.STANDS)}')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    n = int(LaunchConfiguration('robots').perform(context))
    m = int(LaunchConfiguration('messages').perform(context))
    import scaled   # noqa: E402
    general = (n, m) != (2, 4)
    if general and positions:
        raise RuntimeError(f'floor:={floor} is for two robots and four message levels')

    # The world and the lamps, built now from layout.py.
    world = os.path.join(tempfile.gettempdir(), f'coordinated_attack_{built}.world')
    lamps = os.path.join(tempfile.gettempdir(), 'coordinated_attack_lamps')
    subprocess.run([sys.executable, os.path.join(share, 'tools', 'make_world.py'),
                    '--floor', built, '--out', world, '--lamps', lamps], check=True)

    floorplan = os.path.join(share, 'maps', f'floorplan_{built}.yaml')
    if positions:
        domain = os.path.join(share, 'epddl', 'coordinated-attack-positions.epddl')
        problem = os.path.join(share, 'epddl', f'positions-{floor}.epddl')
        mapping = os.path.join(share, 'pddl', 'coordinated-attack-positions-mapping.json')
    else:
        domain = os.path.join(share, 'epddl', 'coordinated-attack.epddl')
        problem = os.path.join(share, 'epddl', f'{floor}.epddl')
        mapping = os.path.join(share, 'pddl', 'coordinated-attack-mapping.json')
    lossy = os.path.join(share, 'epddl', 'lossy.epddl')
    moves = os.path.join(share, 'epddl', 'moves.epddl')
    intermediate = os.path.join(get_package_share_directory('plansys2_epddl_grounder'),
                                'libraries', 'intermediate.epddl')
    model = os.path.join(share, 'pddl', 'coordinated-attack.pddl')
    protocol = ''
    if general:
        written = scaled.write(
            os.path.join(tempfile.gettempdir(), f'coordinated_attack_n{n}_m{m}'), n, m)
        domain, problem = written['domain'], written[floor]
        mapping, model = written['mapping'], written['pddl']
        if floor == 'radio':
            protocol = LaunchConfiguration('protocol').perform(context) or os.path.join(
                share, 'protocols', f'radio-n{n}-m{m}.json')
            if not os.path.exists(protocol):
                raise RuntimeError(
                    f'no radio protocol for {n} robots and {m} levels at {protocol}: '
                    f'tools/ladder.py --robots {n} --depth {m} --protocol writes one')

    with open(os.path.join(share, 'params', 'coordinated_attack.yaml')) as fh:
        params = fh.read()
    if positions:
        params = params.replace('"LOSSY_LIBRARY"]', '"LOSSY_LIBRARY", "MOVES_LIBRARY"]')
    # Message levels past the published four need timeouts of their own.
    extra = [k for k in scaled.kinds(m) if k not in ('tell', 'ack', 'ack2', 'ack3')]
    if extra:
        params = params.replace('"radio_ack3",', '"radio_ack3", ' + ', '.join(
            f'"radio_{k}"' for k in extra) + ',')
        params = params.replace('      go_view:\n', ''.join(
            f'      radio_{k}:\n        duration_overrun_percentage: 2000.0\n' for k in extra)
            + '      go_view:\n')
    params = (params.replace('EPDDL_DOMAIN', domain).replace('EPDDL_PROBLEM', problem)
                  .replace('MAPPING_FILE', mapping).replace('INTERMEDIATE_LIBRARY', intermediate)
                  .replace('LOSSY_LIBRARY', lossy).replace('MOVES_LIBRARY', moves))
    filled = tempfile.NamedTemporaryFile('w', suffix='_coordinated_attack.yaml', delete=False)
    filled.write(params)
    filled.close()

    fleet = L.robots(n)
    agents = list(fleet)
    namespaces = [fleet[a][0] for a in agents]
    stands = sorted(L.STANDS)

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
        ns, x, y, rng, samples, colour = fleet[agent]
        sdf = tempfile.NamedTemporaryFile('w', suffix=f'_{ns}.sdf', delete=False)
        sdf.write(robot.robot_sdf(ns, rng, samples, colour=colour))
        sdf.close()
        per_robot.append(TimerAction(period=8.0 + 4.0 * k, actions=[
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

    common = {'floorplan': floorplan}
    reader = L.READS_ORDER
    poses = []
    for s in stands:
        for a in agents:
            mx, my = L.mouth(s, a, n)
            ux, uy = L.under_end(s, a, n)
            poses += [mx, my, ux, uy, L.facing(a)]
    signal = 'signal_to' if positions else 'signal'
    performers = [
        Node(package='coordinated_attack_demo', executable='read_order_action',
             additional_env=matching_cascade(),
             output='screen', arguments=['--agent', reader],
             parameters=[{**common, 'ns': fleet[reader][0],
                          'terminal': [L.TERMINAL_READ[0], L.TERMINAL_READ[1], L.TERMINAL_YAW],
                          'order': order, 'action_name': 'read_order',
                          'specialized_arguments': [reader, ''], 'rate': 10.0}]),
        Node(package='coordinated_attack_demo', executable='signal_action',
             additional_env=matching_cascade(),
             output='screen', arguments=['--name', signal],
             parameters=[{**common, 'agents': agents, 'namespaces': namespaces,
                          'beacon': list(L.BEACON_POST),
                          'lamps': [f'{s}={os.path.join(lamps, f"lamp_{s}.sdf")}' for s in stands],
                          'action_name': signal, 'rate': 10.0}]),
        Node(package='coordinated_attack_demo', executable='lift_action',
             additional_env=matching_cascade(),
             output='screen',
             parameters=[{**common, 'agents': agents, 'namespaces': namespaces,
                          'stands': stands, 'poses': poses,
                          'loads': flat([(L.stand_x(s), 0.0) for s in stands]),
                          'lift_height': L.LIFT_HEIGHT,
                          'action_name': 'lift', 'rate': 10.0}]),
    ]
    for agent in agents:
        vx, vy = L.viewpoint(agent, n)
        performers.append(Node(
            package='coordinated_attack_demo', executable='go_view_action',
            additional_env=matching_cascade(),
            output='screen', arguments=['--agent', agent],
            parameters=[{**common, 'ns': fleet[agent][0],
                         'viewpoint': [vx, vy, L.facing(agent)],
                         'beacon': list(L.BEACON_POST),
                         'action_name': 'go_view',
                         'specialized_arguments': [agent], 'rate': 10.0}]))
    if positions:
        performers.append(Node(
            package='coordinated_attack_demo', executable='sight_action',
            additional_env=matching_cascade(),
            output='screen',
            parameters=[{'agents': agents, 'namespaces': namespaces,
                         'action_name': 'sight', 'rate': 10.0}]))
    for kind in scaled.kinds(m):
        performers.append(Node(
            package='coordinated_attack_demo', executable='radio_action',
            additional_env=matching_cascade(),
            output='screen', arguments=['--kind', kind],
            parameters=[{'agents': agents, 'action_name': f'radio_{kind}', 'rate': 10.0,
                         'transfer_seconds': 4.0, 'group': general}]))

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': model, 'params_file': filled.name,
                          'epistemic_state': 'True'}.items())

    colours = [c for a in agents for c in fleet[a][5]]
    view = Node(
        package='coordinated_attack_demo', executable='knowledge_view.py', output='screen',
        parameters=[{'floorplan': floorplan, 'agents': agents, 'namespaces': namespaces,
                     'colours': colours, 'stands': stands,
                     'load_boxes': flat([L.load_box(s) for s in stands]),
                     'beacon': list(L.BEACON_POST),
                     'viewpoints': flat([L.viewpoint(a, n) for a in agents]),
                     'terminal': list(L.TERMINAL), 'has_beacon': floor != 'radio',
                     'positions': positions,
                     'crates': list(L.crates_box()) if floor == 'blind' else [0.0]}])

    shots = [f'{a}={fleet[a][0]}' for a in agents]
    shot_poses = dict(L.SHOTS)
    if n > 2:
        # With more robots the published shots frame too little: a message
        # crosses the whole hall, and four robots under a load are lost from
        # eleven metres up.
        shot_poses['radio'] = L.look_at((0.0, -27.0, 21.0), (0.0, -1.0, 0.0))
        for s in L.STANDS:
            x = L.stand_x(s)
            shot_poses[s] = L.look_at((x - 0.4, -4.6, 6.0), (x, 0.4, 0.0))
    shots += [f'{name}={L.shot_text(pose)}' for name, pose in shot_poses.items()]
    director = Node(
        package='coordinated_attack_demo', executable='camera_director.py', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{'follow_file': LaunchConfiguration('follow_file'),
                     'initial': L.shot_text(L.OPENING_SHOT), 'shots': shots}])

    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if n > 2:
        rviz_config = rviz_for(fleet, rviz_config)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}])

    # The mission's PlannerClient is a node of its own in the mission's
    # process, and waits plan_solver_timeout for an answer, 15 s unless told.
    # On the published floors the planner answers in under a second; on the
    # radio floor with four robots it searches its whole budget, and the
    # client is given that budget and a margin, so that what the mission
    # reports is the planner's answer and not its own impatience.
    budget = float(re.search(r'plan_solver_timeout:\s*([\d.]+)', params).group(1))
    mission = Node(
        package='coordinated_attack_demo', executable='coordinated_attack_mission',
        output='screen',
        arguments=['--ros-args', '-p', f'plan_solver_timeout:={budget + 15.0}'],
        parameters=[{'epddl_problem': problem, 'action_mapping': mapping, 'floor': floor,
                     'protocol': protocol,
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
    share = get_package_share_directory('coordinated_attack_demo')
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    xl = get_package_share_directory('warehouse_xl_rmf_demo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='beacon',
                              description='beacon: the floor with a beacon; radio: without; '
                                          'sight and blind: positions not announced, with and '
                                          'without a sight line through t2.'),
        DeclareLaunchArgument('order', default_value='s1',
                              description='The stand the work order names: s1 or s2.'),
        DeclareLaunchArgument('robots', default_value='2',
                              description='2 to 4. Other than 2, or messages other than 4, '
                                          'runs the domain tools/scaled.py writes.'),
        DeclareLaunchArgument('messages', default_value='4',
                              description='Message levels on the radio, tell, ack, ack2, ...'),
        DeclareLaunchArgument('protocol', default_value='',
                              description='With robots or messages given, the radio protocol '
                                          'to run; default protocols/radio-n<r>-m<m>.json.'),
        DeclareLaunchArgument('gui', default_value='true', description='gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Write the current shot to follow_file.'),
        DeclareLaunchArgument('follow_file', default_value='/tmp/coordinated_attack_follow'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'config', 'coordinated_attack.rviz')),
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
