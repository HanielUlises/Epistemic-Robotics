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
ePlanSys over the warehouse, for a fleet of one, two or three.

    ros2 launch warehouse_demo warehouse_demo_launch.py               # r1 alone
    ros2 launch warehouse_demo warehouse_demo_launch.py robots:=2
    ros2 launch warehouse_demo warehouse_demo_launch.py robots:=3
    ros2 launch warehouse_demo warehouse_demo_launch.py robots:=3 gui:=false

What runs, and why each piece is there:

  gazebo           the AWS RoboMaker small warehouse -- the world the
                   RoboticsAcademy exercise itself runs on -- and one TurtleBot3
                   per robot. The grid the planner reasons over is rasterised
                   from this world's own collision meshes, so a rack it plans
                   around is the rack the laser sees
  slam_toolbox     one per robot, building that robot's own map from its own
                   laser -- which is what makes the demo honest twice over: a
                   bay nobody has looked into is unobserved in the grid and
                   undecided in the epistemic model at once, and a bay *r2* has
                   not looked into is not something r2 knows merely because r1
                   did. Sensing here is semi-private, and a shared map would
                   quietly make it public
  plansys2         the six-node epistemic bringup, once for the fleet: there is
                   one planner and one epistemic state, because there is one
                   problem with several agents in it
  perception       one per robot, reading that robot's map and reporting what
                   it saw as that robot's inspection
  mu_path_planner  one per robot: the least fixed point over that robot's map,
                   deciding whether a zone may be driven to at all
  nav2             one stack per robot, in that robot's namespace, doing the
                   driving once something above it has said where
  action nodes     one set per robot, in the global namespace so they can reach
                   the fleet's action hub, but with every robot-specific topic
                   remapped into that robot's namespace
  mission          seeds the problem with the whole fleet, asks for the policy,
                   runs it

Which robots exist, where they come on shift and which EPDDL instance describes
them is FLEET and INSTANCES below, and those have to agree with the instances
in epddl-workspace/robot-warehouse: the planner reads the EPDDL, the mission
node seeds the classical half, and a robot in one and not the other is a policy
the executor cannot dispatch.
"""

import os
import re
import shutil
import tempfile

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The fleet, in the order robots are added. Each entry is the agent name the
# EPDDL instance uses, the zone it comes on shift in, and the constant in
# warehouse_scenario's header that says where that is in metres.
#
# The zone and the pose are two statements of one fact and have to agree: the
# mission node tells PlanSys2 the robot is in the zone, Gazebo puts it at the
# pose, and if they disagree the first drive is dispatched from somewhere the
# robot is not.
FLEET = [
    ('r1', 'dock_south', 'kR1Start'),
    ('r2', 'dock_north', 'kR2Start'),
    ('r3', 'corridor', 'kR3Start'),
]

# The EPDDL instance for each fleet size, and the robot whose knowledge its
# goal is about. One robot: r1 delivers and has to know. Two: r2 must know and
# never leaves receiving. Three: r2 and r3 must both know, and the policy pays
# for an announcement to manage it.
INSTANCES = {
    1: os.path.join('instances', 'problem_1.epddl'),
    2: 'warehouse-problem.epddl',
    3: os.path.join('instances', 'problem_3.epddl'),
}

# The same instances as plank ground them, checked in beside the source. Handed
# to the epistemic state so that it does not have to run plank while it is
# configuring. Regenerate with epddl-workspace/robot-warehouse/validate.sh.
TASKS = {
    1: os.path.join('out', 'problem_1.json'),
    2: os.path.join('out', 'warehouse-problem.json'),
    3: os.path.join('out', 'problem_3.json'),
}
KNOWER = {1: 'r1', 2: 'r2', 3: 'r2'}

# Where a pallet stands in each bay, in the world's own frame. These are the
# poses `look_action_node` turns to face and the poses perception's regions are
# drawn on the near face of, so they are one fact with three readers and have
# to agree.
PALLET = {
    'bay2': (4.50, -2.14),
    'bay3': (4.50, -3.94),
}


def start_pose(constant):
    """Where warehouse_scenario says a robot stands, read from its own header.

    The drive action turns a world box into a map box using these poses.
    Writing them here as well would be two definitions of one number, and the
    failure when they drift is not a crash -- it is a robot driving to the
    wrong place, which is the expensive kind of wrong.
    """
    header = os.path.join(
        get_package_prefix('warehouse_scenario'),
        'include', 'warehouse_scenario', 'warehouse.hpp')
    with open(header) as handle:
        found = re.search(
            constant + r'\s*\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}',
            handle.read())
    if not found:
        raise RuntimeError('no ' + constant + ' in ' + header)
    return found.group(1), found.group(2)


def aletheia():
    """Where ALETHEIA's planner is, without writing one machine's path down.

    The solver plugin shells out to it, so it has to be a real path by the time
    the planner configures. `epistemic_planner` on PATH is the normal answer;
    ALETHEIA_PLANNER overrides it for a build kept somewhere else, which is the
    same knob validate.sh takes.
    """
    named = os.environ.get('ALETHEIA_PLANNER')
    if named:
        return named
    found = shutil.which('epistemic_planner')
    if found:
        return found
    for guess in (os.path.expanduser('~/aletheia/team-3/build/epistemic_planner'),
                  os.path.expanduser('~/Projects/team-3/build/epistemic_planner')):
        if os.path.exists(guess):
            return guess
    raise RuntimeError(
        'ALETHEIA\'s planner was not found. It is what solves the epistemic '
        'task: put `epistemic_planner` on PATH, or set ALETHEIA_PLANNER to it.')


def written(text, suffix):
    """A parameters or model file with this launch's substitutions in it."""
    handle = tempfile.NamedTemporaryFile(mode='w', suffix=suffix, delete=False)
    handle.write(text)
    handle.close()
    return handle.name


def robot_sdf(source, robot, single):
    """The burger's model, with its plugins moved into one robot's namespace.

    The shipped SDF carries a commented-out `<namespace>/tb3</namespace>` in
    every plugin's `<ros>` block, and names its frames `odom`, `base_footprint`
    and `base_scan`. One robot can use it as it stands. Two cannot: both would
    publish `/scan` and both would claim the `odom -> base_footprint` edge of
    the transform tree, and TF admits one parent per frame, so the second robot
    to start does not so much fail as make the first one wrong.

    So each robot gets its own copy, with the namespace filled in and the
    frames prefixed. The `map` frame is deliberately left alone: every robot
    starts in this world and measures the same building, so the fleet shares
    one coordinate frame and rviz can draw it. What differs per robot is what
    it has measured, which is a difference in the maps and not in the frame.
    """
    with open(source) as handle:
        sdf = handle.read()

    if single:
        return sdf

    sdf = sdf.replace(
        '<!-- <namespace>/tb3</namespace> -->',
        '<namespace>/' + robot + '</namespace>')

    # Only the *frame* tags. `<odometry_topic>odom</odometry_topic>` sits two
    # lines from `<odometry_frame>odom</odometry_frame>`, and rewriting both --
    # which replacing the bare text does -- publishes odometry on
    # /r2/r2/odom, where the namespace has been applied to a name that already
    # carried it, and Nav2's odom_topic then points at nothing.
    for tag, value in (('odometry_frame', 'odom'),
                       ('robot_base_frame', 'base_footprint'),
                       ('frame_name', 'base_scan')):
        sdf = sdf.replace(
            '<{0}>{1}</{0}>'.format(tag, value),
            '<{0}>{2}/{1}</{0}>'.format(tag, value, robot))
    return sdf


def nav2_params_for(source, robot, single):
    """Nav2's parameters, rewritten for one robot's namespace and frames.

    Every top-level key in the file is a node name, and a node inside a
    namespace is only handed parameters filed under that namespace. The frames
    and the two sensor topics move with it; `/floorplan` does not, because the
    building is given to the whole fleet rather than measured by any of them.
    """
    with open(source) as handle:
        text = handle.read()
    if single:
        return text

    text = re.sub(r'^([A-Za-z_][A-Za-z0-9_]*):$',
                  '/' + robot + r'/\1:', text, flags=re.M)
    text = text.replace('robot_base_frame: base_footprint',
                        'robot_base_frame: ' + robot + '/base_footprint')
    text = text.replace('global_frame: odom', 'global_frame: ' + robot + '/odom')
    text = text.replace('odom_topic: /odom', 'odom_topic: /' + robot + '/odom')
    text = text.replace('topic: /scan', 'topic: /' + robot + '/scan')
    return text


def perception_params(source, fleet, single):
    """One perception node per robot, each over that robot's own map.

    The shipped block watches two bays and reports them as *r1's* inspections,
    which is right for one robot and a lie for three: an observation is an
    event in one agent's sensing action, and semi-private sensing means the
    agent that looked is the only one it settles anything for. So the block is
    copied per robot, filed under that robot's node name, pointed at that
    robot's map, and its `sensing_action` entries renamed to that robot's.
    """
    with open(source) as handle:
        text = handle.read()
    if single:
        return text

    start = text.index('epistemic_perception:')
    head, block = text[:start], text[start:]

    copies = []
    for robot, _zone, _pose in fleet:
        one = block.replace('epistemic_perception:',
                            'epistemic_perception_' + robot + ':', 1)
        one = one.replace('map_topic: "/map"', 'map_topic: "/' + robot + '/map"')
        one = one.replace('inspect_r1_', 'inspect_' + robot + '_')
        copies.append(one)
    return head + '\n'.join(copies)


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('warehouse_demo')
    gui = LaunchConfiguration('gui')

    count = int(LaunchConfiguration('robots').perform(context))
    if count not in INSTANCES:
        raise RuntimeError(
            'robots:={} -- there are EPDDL instances for {}, and a fleet size '
            'without one is a mission nobody has written'
            .format(count, sorted(INSTANCES)))
    fleet = FLEET[:count]
    single = count == 1

    epddl = os.path.join(pkg, 'epddl')
    domain = os.path.join(epddl, 'warehouse-domain.epddl')
    problem = os.path.join(epddl, INSTANCES[count])
    mapping = os.path.join(pkg, 'pddl', 'warehouse-mapping.json')
    model = os.path.join(pkg, 'pddl', 'warehouse.pddl')

    params = perception_params(
        os.path.join(pkg, 'params', 'warehouse_demo.yaml'), fleet, single)
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('ACTION_MAPPING', mapping)
              .replace('EPDDL_TASK', os.path.join(epddl, TASKS[count]))
              .replace('ALETHEIA_COMMAND', aletheia()))
    filled = written(params, '_warehouse_demo.yaml')

    # The exercise's own world, unmodified. warehouse_scenario's floor plan is
    # rasterised from these very meshes, so the two cannot drift apart.
    aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    world = os.path.join(
        aws, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')

    models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models + ':' + os.path.join(aws, 'models') + ':' + aws + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''))

    # The AWS models point at their meshes with `file://models/<name>/...`,
    # which is a path relative to a resource root rather than a model:// URI.
    # Without the package share on GAZEBO_RESOURCE_PATH every collision mesh
    # fails to load, and the warehouse comes up as a floor with nothing on it
    # -- a laser that returns nothing, and a demonstration of nothing.
    resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        aws + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', ''))

    plugins = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose'] + plugins + [world],
        output='screen', condition=IfCondition(gui))
    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '--verbose'] + plugins + [world],
        output='screen', condition=UnlessCondition(gui))

    # The pallet, in one of the two bays. Which one is the fact the fleet is
    # uncertain about, and perception is what settles it -- so this pose is the
    # ground truth the demo is *not* told.
    #
    # And which one is worth choosing. In `bay2` the robot looks where the
    # policy looks first and finds it there, so the run takes the found branch
    # and never exercises the other one. In `bay3` it looks into bay2, finds it
    # *empty*, and has to come back out to the lane and try the other aisle --
    # three moves the building charges for a wrong guess, because the racks
    # leave no way through. That is the branch worth watching: it is the one
    # that shows the plan is a policy rather than a sequence, and with three
    # robots it is the only branch on which the announcement is spent.
    #
    # Nothing else changes. Same domain, same instance, same goal; the planner
    # is not told either way, and perception already watches both bays.
    where = LaunchConfiguration('pallet').perform(context)
    if where not in PALLET:
        raise RuntimeError(
            'pallet:={} -- it is in one of the two bays the domain disputes: {}'
            .format(where, ' or '.join(sorted(PALLET))))
    pallet_x, pallet_y = PALLET[where]

    pallet = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_pallet',
        output='screen',
        arguments=['-entity', 'pallet',
                   '-file', os.path.join(pkg, 'worlds', 'pallet.sdf'),
                   '-x', str(pallet_x), '-y', str(pallet_y), '-z', '0.25'])

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf')
    with open(urdf) as handle:
        urdf_text = handle.read()

    burger = os.path.join(models, 'turtlebot3_burger', 'model.sdf')
    floorplan = os.path.join(
        get_package_share_directory('warehouse_scenario'), 'maps',
        'aws_small_warehouse.yaml')
    nav2_source = os.path.join(pkg, 'params', 'nav2.yaml')

    # Everything a robot needs, and when.
    #
    # A fleet does not come up the way one robot does. `/spawn_entity` is a
    # single service and Gazebo answers it one model at a time while it is also
    # loading a warehouse of collision meshes; ask three times at once on a
    # loaded machine and the second and third calls simply never return. So the
    # robots go in one at a time, and everything that depends on a robot
    # existing waits for it.
    per_robot, nav2_by_robot, nav2_managed_names = [], [], []
    spawn_every = 0.0 if single else 12.0
    # The first robot waits too, and does not go in at zero. Gazebo is still
    # loading the warehouse's collision meshes then, and a model spawned into
    # it comes up with its diff drive plugin attached but silent: the robot is
    # in the world, `robot_state_publisher` has its links, SLAM gets scans --
    # and nothing ever publishes odom -> base_footprint, so every costmap
    # waits on a transform that is never coming. The robots that went in later
    # were the ones that worked.
    first_spawn = 0.0 if single else 10.0
    for index, (robot, zone, constant) in enumerate(fleet):
        at = first_spawn + index * spawn_every

        def later(node, delay=at):
            """The same node, held back until the machine is ready for it."""
            return node if delay <= 0.0 else TimerAction(period=delay,
                                                         actions=[node])

        ns = '' if single else '/' + robot
        prefix = '' if single else robot + '/'
        base = prefix + 'base_footprint'
        suffix = '' if single else '_' + robot

        # The spawn.
        #
        # The yaw is zero, and it has to be. Everything downstream -- the zone
        # boxes, the snapshots -- is written in this world's frame, and SLAM
        # fixes the map frame to wherever the robot was standing when it
        # started, orientation included. The drive turns a world box into a map
        # box by subtracting that pose, which is a translation and nothing
        # else; spawn the robot facing any other way and the map frame is
        # rotated against the world while the zone boxes are not, so every zone
        # lands somewhere the robot has no business driving to and it turns on
        # the spot looking for it.
        x, y = start_pose(constant)
        per_robot.append(later(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            name='spawn' + suffix, output='screen',
            arguments=['-entity', robot,
                       '-file', written(robot_sdf(burger, robot, single), '.sdf'),
                       # No `-robot_namespace` here. The SDF above already
                       # names the namespace inside each plugin's own `<ros>`
                       # block, which is the part that decides where that
                       # plugin publishes; passing it here as well applies it
                       # a second time, and the topics come up doubled.
                       '-x', x, '-y', y, '-z', '0.01', '-Y', '0.0'])))

        # Gazebo's diff drive plugin publishes odom -> base_footprint and
        # nothing else; the laser reports in base_scan. Without the robot's own
        # static joints SLAM drops every scan it is handed and never produces a
        # map.
        per_robot.append(later(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=ns, output='screen',
            parameters=[{
                'use_sim_time': True,
                # The prefix goes in once, through the URDF's own
                # `${namespace}` hook. robot_state_publisher's `frame_prefix`
                # would prepend it a *second* time to link names that already
                # carry it, and the tree comes up as r2/r2/base_footprint --
                # which nothing is looking for.
                'robot_description': urdf_text.replace('${namespace}', prefix),
            }])))

        per_robot.append(later(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=ns, output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': prefix + 'odom',
                'map_frame': 'map',
                'base_frame': base,
                'scan_topic': (ns or '') + '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'max_laser_range': 8.0,
                # A sensing action stands the robot still and looks; with the
                # default travel thresholds the mapper stops integrating
                # exactly while it does, and the bay it was dispatched to
                # settle is never observed at all.
                'minimum_travel_distance': 0.0,
                'minimum_travel_heading': 0.0,
                'map_update_interval': 1.0,
            }],
            # slam_toolbox names its *services* relative to the node and its
            # map absolutely: the publisher is `/map`, not `map`. So putting
            # the node in a namespace moves everything about it except the one
            # topic that matters, and three mappers in three namespaces all
            # publish the same `/map` and overwrite each other.
            #
            # This is worth more than a startup bug. A fleet sharing one map is
            # a fleet where anything one robot has seen is immediately seen by
            # all of them -- which would make the sensing in this domain public
            # while the domain says it is semi-private, and the demonstration
            # would be quietly proving the wrong thing.
            remappings=None if single else [
                ('/map', ns + '/map'),
                ('/map_metadata', ns + '/map_metadata')])))

        per_robot.append(Node(
            package='mu_path_planner', executable='mu_path_planner_node',
            name='mu_path_planner', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True,
                         'free_below': 25,
                         'occupied_above': 65,
                         # 1.5 m over 0.05 m cells, the same reach the offline
                         # scenario resolves its queries with.
                         'sensor_range_cells': 30,
                         # A burger is 0.21 m across over 0.05 m cells, so its
                         # own radius is two cells. Six is well over that, and
                         # deliberately: it also shuts the gaps the building
                         # leaves that are technically wider than the robot and
                         # not worth threading -- 0.43 m between the pallet jack
                         # and the south wall, 0.15 m behind the racks. A route
                         # that goes round costs seconds; one that goes through
                         # ends with the robot wedged. The aisles are 0.92 m and
                         # stay open. kInflationCells in drive_action_node.cpp
                         # has to match: the drive picks where to stand and the
                         # planner decides whether it can be stood in.
                         'inflation_cells': 6}]))

        # Perception stays in the global namespace, because it talks to the
        # fleet's one epistemic state and would look for it inside the robot's
        # namespace otherwise. What makes it this robot's perception is the
        # map it reads and the sensing actions it reports, both of which are in
        # the parameters above.
        per_robot.append(Node(
            package='plansys2_epistemic_perception',
            executable='epistemic_perception_node',
            name='epistemic_perception' + suffix, output='screen',
            parameters=[filled, {'use_sim_time': True}]))

        # Nav2, composed by hand: nav2_bringup is not installed here, and the
        # servers are the whole of what it would have started anyway.
        #
        # No AMCL. slam_toolbox is building the map and publishing
        # map -> odom as it goes; a second localiser fighting it over the same
        # transform is how a robot ends up believing it is outside the
        # building.
        nav2 = written(nav2_params_for(nav2_source, robot, single), '_nav2.yaml')
        nav2_by_robot.append([
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', namespace=ns, output='screen',
                 parameters=[nav2], remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', namespace=ns, output='screen',
                 parameters=[nav2]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', namespace=ns, output='screen',
                 parameters=[nav2]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', namespace=ns, output='screen',
                 parameters=[nav2]),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', namespace=ns, output='screen',
                 parameters=[nav2],
                 remappings=[('cmd_vel', 'cmd_vel_nav'),
                             ('cmd_vel_smoothed', 'cmd_vel')]),
        ])
        nav2_managed_names.append([
            (ns + '/' if ns else '') + name for name in
            ('controller_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'velocity_smoother')])

        # The action nodes stay global too, and for the same reason: the
        # executor's `actions_hub` is a relative name, so a node inside a
        # namespace would advertise its own hub and never hear the fleet's.
        # Everything about them that *is* the robot's is remapped instead.
        robot_topics = [] if single else [
            ('cmd_vel', ns + '/cmd_vel'),
            ('map', ns + '/map'),
            ('epistemic/state', ns + '/epistemic/state'),
            ('mu_planner/query', ns + '/mu_planner/query'),
            ('mu_planner/path', ns + '/mu_planner/path'),
            # All five interfaces of the action, by name.
            #
            # rclcpp_action does not remap an action by its action name: the
            # client expands `navigate_to_pose` into three services and two
            # topics first, and a remap rule for the bare name never matches
            # any of them. The failure is silent -- `action_server_is_ready()`
            # simply stays false, the drive returns without sending a goal, and
            # the robot sits at its dock computing perfectly good routes it
            # never drives.
            ('navigate_to_pose/_action/send_goal',
             ns + '/navigate_to_pose/_action/send_goal'),
            ('navigate_to_pose/_action/cancel_goal',
             ns + '/navigate_to_pose/_action/cancel_goal'),
            ('navigate_to_pose/_action/get_result',
             ns + '/navigate_to_pose/_action/get_result'),
            ('navigate_to_pose/_action/feedback',
             ns + '/navigate_to_pose/_action/feedback'),
            ('navigate_to_pose/_action/status',
             ns + '/navigate_to_pose/_action/status'),
        ]

        def act(executable, name, action_name, args, extra=None):
            # `specialized_arguments` is how one robot's node declines another
            # robot's action: the executor matches it positionally against the
            # action's own arguments, and "" is a wildcard. Without it every
            # drive node in the fleet claims every drive.
            #
            # With one robot there is nothing to decline and the list is empty
            # -- and it is left out rather than passed empty, because launch
            # cannot give a type to an empty list and refuses the node.
            parameters = [{'action_name': action_name, 'use_sim_time': True}]
            if args:
                parameters[0]['specialized_arguments'] = args
            if extra:
                parameters.append(extra)
            return Node(
                package='warehouse_demo', executable=executable,
                name=name + suffix, output='screen',
                parameters=parameters,
                # `or None` for the same reason as the empty argument list
                # above: with one robot there is nothing to remap, and launch
                # will not take an empty list.
                remappings=robot_topics or None)

        drive_args = [robot, '', ''] if not single else []
        bay_args = [robot, ''] if not single else []
        per_robot += [
            act('drive_action_node', 'drive_action', 'goto_zone', drive_args,
                {'robot': robot, 'base_frame': base}),
            act('look_action_node', 'look_action', 'look_into', bay_args,
                {'base_frame': base, 'robot': robot,
                 'knows_bay2': '(Kw ' + robot + ' pallet-at_bay2)',
                 'knows_bay3': '(Kw ' + robot + ' pallet-at_bay3)'}),
            act('handle_action_node', 'pick_action', 'pick_up', bay_args,
                {'verb': 'lifting the pallet', 'dwell': 4.0}),
            act('handle_action_node', 'drop_action', 'drop_off', bay_args,
                {'verb': 'unloading at the dock', 'dwell': 4.0}),
            act('handle_action_node', 'announce_action', 'announce', bay_args,
                {'verb': 'radioing the fleet', 'dwell': 3.0}),
        ]

    # The floor plan, served to the whole fleet on a topic of its own.
    #
    # This is the split the demo turns on. A warehouse robot is given the
    # building; what it does not have is a record of what it has actually
    # looked at. So Nav2 plans over `/floorplan` -- the rasterised AWS world,
    # the same one warehouse_scenario carries -- and drives straight to a zone
    # instead of feeling its way there, while each robot's own `map` stays what
    # its slam_toolbox has genuinely measured and stays what its
    # mu_path_planner answers "is there a route over floor anyone has seen?"
    # against.
    #
    # One server for the fleet, because the building is one building and
    # nobody measured it.
    floorplan_nodes = [Node(
        package='nav2_map_server', executable='map_server',
        name='floorplan_server', output='screen',
        parameters=[{'use_sim_time': True,
                     'yaml_filename': floorplan,
                     'topic_name': 'floorplan',
                     'frame_id': 'map'}])]

    # The manager goes up after the servers it manages, not beside them.
    #
    # Started together, it creates its service clients and begins the bringup
    # about a hundred milliseconds later -- before controller_server has
    # advertised its lifecycle services at all. The change_state call fails
    # instantly, the manager aborts the whole bringup, and every server logs a
    # clean configure just afterwards, so the logs show nothing wrong with any
    # of them.
    #
    # One manager per robot: a fleet's worth of servers under a single manager
    # is a single bringup, and one slow robot then aborts the others.
    # And something to bring perception up.
    #
    # `epistemic_perception_node`'s main spins the node and nothing else: it is
    # a lifecycle node, and left alone it sits unconfigured forever, reading no
    # map and reporting nothing. Inside the plansys2 bringup the sixth managed
    # node is configured for it; started standalone -- which is what a fleet
    # needs, one per robot -- it needs a manager of its own.
    #
    # Without this the mission gets all the way to the bay, faces it, and times
    # out with "(Kw r1 pallet-at_bay2) still does not hold", which reads like a
    # perception failure and is actually a node that was never switched on.
    perception_manager = [] if single else [Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_perception', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'bond_timeout': 0.0,
                     'node_names': ['epistemic_perception_' + name
                                    for name, _z, _p in fleet]}])]

    floorplan_manager = [Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_floorplan', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'bond_timeout': 30.0,
                     'attempt_respawn_reconnection': True,
                     'node_names': ['floorplan_server']}])]
    nav2_manager = []
    for index, names in enumerate(nav2_managed_names):
        nav2_manager.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation_' + fleet[index][0],
            output='screen',
            parameters=[{
                # Wall clock, deliberately, and the only nodes here that do not
                # use the simulator's. Bringing five servers through their
                # lifecycle is not something that happens in simulated time,
                # and its timeouts should not stretch or shrink with how fast
                # the simulator is managing to run.
                'use_sim_time': False,
                'autostart': True,
                # The default is four seconds, and on a cold start this machine
                # spends longer than that just loading the warehouse meshes.
                # The manager then declares a server failed while it is still
                # configuring and aborts the whole of Nav2 -- with the servers
                # themselves reporting no error at all, because nothing was
                # wrong with them.
                #
                # And for a fleet, no bond at all. A bond is a heartbeat the
                # manager expects from each server, and a server that is merely
                # slow -- two or three Nav2 stacks and a simulator on one
                # machine is exactly that -- misses it while running perfectly
                # well. The manager then tears the stack down and brings it up
                # again, forever, and the robot never gets a navigator. Zero
                # turns the heartbeat off and leaves the lifecycle transitions,
                # which are what actually matter here.
                'bond_timeout': 30.0 if single else 0.0,
                'attempt_respawn_reconnection': True,
                'node_names': names}]))

    # Started once the simulator and the mappers have had a moment. Nav2's
    # costmaps want a map and a transform tree, and bringing them up into an
    # empty one is the other half of the race above. A larger fleet is a slower
    # simulator, so the wait grows with it.
    # After the last robot is in the world, and after plansys2 has had a clear
    # run at coming up. Its own lifecycle manager times out waiting for the
    # executor to report a state, and three Nav2 stacks configuring their
    # costmaps at the same moment is exactly what makes the executor slow
    # enough to miss it -- a bringup that failed with nothing wrong with any of
    # the nodes in it.
    settle = 20.0 if single else first_spawn + spawn_every * len(fleet) + 30.0

    # And one robot's Nav2 at a time.
    #
    # Five servers configuring at once is a lot of costmap; three robots' worth
    # at once is enough that a lifecycle manager's `get_state` call to a server
    # that has just finished configuring perfectly well times out and the
    # manager aborts the bringup. The servers are not the problem and their
    # logs say nothing is wrong, which is what makes it worth writing down.
    nav2_every = 0.0 if single else 30.0
    brought_up = [TimerAction(period=settle, actions=floorplan_nodes),
                  TimerAction(period=settle + 8.0, actions=floorplan_manager)]
    if perception_manager:
        # After the epistemic state, which is what perception reports to.
        brought_up.append(
            TimerAction(period=settle + 8.0, actions=perception_manager))
    for index, nodes in enumerate(nav2_by_robot):
        at = settle + 12.0 + index * nav2_every
        brought_up.append(TimerAction(period=at, actions=nodes))
        brought_up.append(
            TimerAction(period=at + 12.0, actions=[nav2_manager[index]]))
    last_nav2_up = settle + 12.0 + max(0, len(fleet) - 1) * nav2_every + 24.0

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled,
            'epistemic_state': 'True',
            # For one robot the bringup starts perception itself, as it always
            # has. For a fleet it must not: there is one perception node per
            # robot and they are started above, because an observation belongs
            # to the agent that made it.
            'epistemic_perception': 'True' if single else 'False',
        }.items())

    return [
        model_path, resource_path, gazebo_gui, gazebo_headless, pallet,
    ] + per_robot + [
    ] + brought_up + [
        plansys2,
        Node(package='warehouse_demo', executable='markers_node',
             name='warehouse_markers', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='warehouse_demo', executable='mission_node',
             name='warehouse_mission', output='screen',
             parameters=[{'use_sim_time': True,
                          'agents': [name for name, _z, _p in fleet],
                          'starts': [zone for _n, zone, _p in fleet],
                          'knower': KNOWER[len(fleet)],
                          # A fleet is a slower simulator and a slower bringup,
                          # and planning before there is a map only means the
                          # first drive begins blind.
                          'settle': last_nav2_up + 10.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(gui),
             arguments=['-d', os.path.join(pkg, 'config', 'warehouse_demo.rviz')],
             parameters=[{'use_sim_time': True}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo client and rviz, or neither'),
        DeclareLaunchArgument(
            'pallet', default_value='bay2',
            description='Which bay the pallet actually stands in: bay2 or '
                        'bay3. Nobody in the domain is told; it decides which '
                        'branch of the policy the run takes. bay3 is the one '
                        'where the first look comes back empty.'),
        DeclareLaunchArgument(
            'robots', default_value='1',
            description='How many robots come on shift: 1, 2 or 3. Each has an '
                        'EPDDL instance of its own, and they differ in whose '
                        'knowledge the goal is about, not in the mission.'),
        OpaqueFunction(function=setup),
    ])
