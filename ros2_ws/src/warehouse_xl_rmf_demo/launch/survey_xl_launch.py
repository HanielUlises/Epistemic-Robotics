# Copyright 2026 Haniel Ulises
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
The site survey of `eplansys`, over the larger warehouse floor.

The mission, the EPDDL and the policy are `eplansys_demo`'s, unchanged. What
changes is the floor: the survey runs over the 30 by 50 metre warehouse with
thirty-four aisles instead of the office, and the site the scan reports on is
`east_08` --- down an aisle, which is a place a robot has to enter before it
can tell anything about it.

    ros2 launch warehouse_xl_rmf_demo survey_xl_launch.py
    ros2 launch warehouse_xl_rmf_demo survey_xl_launch.py site:=clean
    ros2 launch warehouse_xl_rmf_demo survey_xl_launch.py rmf:=false

Each agent runs a `radio` on the speech-act channels, so who was spoken to is
recorded rather than assumed. When the robots have stopped, `mission_check`
asks the epistemic state whether the mission came out the way the goal asked
and reads those transcripts to see who actually heard anything.
`check:=false` leaves both out.

The mission, the EPDDL and the policy are `eplansys_demo`'s. What this file
changes is the performers: instead of waiting out a duration, they submit RMF
tasks and wait for the fleet to report them done.

`rmf:=false` leaves the fleet to a separate terminal, which is what to use when
the floor is already running. Launched here, the adapter's websocket is pointed
at the bridge, which is the only way a task's outcome reaches the epistemic
state on Humble.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OUTCOMES = {
    'dirty': 'e-scan-dirty',
    'clean': 'e-scan-clean',
}

WEBSOCKET_PORT = 7879

# Every agent of the mission, including the one the goal excludes. `observer`
# runs a radio precisely so that its silence is recorded rather than assumed.
AGENTS = ('scout', 'relay', 'observer')

CHANNEL_PREFIX = '/eplansys/channel'


def launch_setup(context, *args, **kwargs):
    demo = get_package_share_directory('eplansys_demo')
    here = get_package_share_directory('warehouse_xl_rmf_demo')

    site = LaunchConfiguration('site').perform(context)
    if site not in OUTCOMES:
        raise RuntimeError(
            f'site:={site} is not one of {sorted(OUTCOMES)}. It is what the '
            'scout turns out to find.')

    domain = os.path.join(demo, 'epddl', 'survey-team.epddl')
    problem = os.path.join(demo, 'epddl', 'survey-team-problem.epddl')
    mapping = os.path.join(demo, 'pddl', 'survey-mapping.json')
    model = os.path.join(demo, 'pddl', 'survey.pddl')

    with open(os.path.join(demo, 'params', 'survey.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey_rmf.yaml', delete=False)
    filled.write(params)
    filled.close()

    # The task map is read at start up, and site: selects what the scan falls
    # back to when the fleet reports nothing. Writing it out here keeps the
    # checked-in map free of a value that is really a launch argument.
    with open(os.path.join(here, 'config', 'warehouse_xl_survey.json')) as handle:
        task_map = handle.read()
    task_map = task_map.replace('"e-scan-dirty"', f'"{OUTCOMES[site]}"')

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_warehouse_xl_survey.json', delete=False)
    map_file.write(task_map)
    map_file.close()

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            'epistemic_state': 'True',
        }.items())

    bridge = Node(
        package='eplansys_rmf_bridge',
        executable='rmf_action_node',
        name='eplansys_rmf_bridge',
        output='screen',
        parameters=[{
            'task_map': map_file.name,
            'websocket_port': WEBSOCKET_PORT,
            # 180 s is the office floor's figure and far too short here: this
            # warehouse is forty metres across, a TurtleBot crosses it at about
            # half a metre a second, and the first run failed with "RMF task
            # timed out" on goto_site with the robot still underway.
            'task_timeout': 900.0,
            'channel_prefix': CHANNEL_PREFIX,
        }])

    # One pair of ears per agent, up before anything is said. A speech act is
    # published once, and although the channels are latched so a late listener
    # still hears, a radio that starts first is the honest arrangement: the
    # transcript then records what was heard rather than what was replayed.
    radios = [
        Node(
            package='eplansys_rmf_demo',
            executable='radio',
            name=f'radio_{agent}',
            output='screen',
            parameters=[{
                'agent': agent,
                'channel_prefix': CHANNEL_PREFIX,
            }])
        for agent in AGENTS
    ]

    # The sensing action's eyes. Without it the scan reports whatever the task
    # map says, and the policy branches on a constant.
    perception = Node(
        package='warehouse_xl_rmf_demo',
        executable='scan_perception.py',
        name='scan_perception',
        output='screen',
        parameters=[{
            'robot': 'r1',
            'scan_topic': '/scan',
            'observation_topic': '/eplansys/observation',
            'site_x': -3.35,
            'site_y': 2.00,
            'site_radius': 0.40,
            'settle': 3,
            'threshold': 0.70,
        }])

    mission = Node(
        package='eplansys_demo',
        executable='survey_mission',
        name='survey_mission',
        output='screen')

    # The larger floor's own fleet launch, which brings up Gazebo, the three
    # robots, the traffic schedule and RViz, and points the fleet adapter's
    # websocket at the bridge.
    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            here, 'launch', 'warehouse_xl_fleet.launch.py')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'server_uri': f'ws://localhost:{WEBSOCKET_PORT}',
        }.items())

    # The third conjunct of the goal is a negative one, and `observer` is bound
    # to no robot and never moves, so it is satisfied by everything that fails
    # to happen. Asking the epistemic state afterwards is what tells a run that
    # honoured the private channel apart from a run that simply did nothing.
    #
    # The window for asking is narrow: the executor has to be finished and the
    # state node has to still be alive, which is between the mission exiting
    # and the shutdown it used to emit directly.
    check = Node(
        package='eplansys_rmf_demo',
        executable='mission_check',
        name='mission_check',
        output='screen',
        parameters=[{'channel_prefix': CHANNEL_PREFIX}])

    checking = LaunchConfiguration('check').perform(context).lower() in ('true', '1')

    staged = []
    if checking:
        staged.append(
            RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[check])))

    # Whether the mission ending takes the simulation down with it. It should,
    # for an unattended run; it must not while a recording is being made, since
    # the windows close the moment the checks print and the last thing filmed
    # is a desktop.
    if LaunchConfiguration('shutdown').perform(context).lower() in ('true', '1'):
        staged.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=check if checking else mission,
                    on_exit=[EmitEvent(event=Shutdown())])))

    return [fleet, plansys2, bridge, perception, *radios, mission, *staged]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'site', default_value='dirty',
            description='What the scout turns out to find: dirty or clean.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo headless and leave rviz out.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('warehouse_xl_rmf_demo'),
                'config', 'warehouse_xl.rviz'),
            description='RViz configuration, passed through to the fleet.'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join('/tmp', 'warehouse_xl_dirty.world'),
            description='Which variant of the floor to run: the pallet is in '
                        'the dirty one and absent from the clean one, and '
                        'nothing else differs.'),
        DeclareLaunchArgument(
            'shutdown', default_value='true',
            description='Bring the simulation down when the mission ends. '
                        'false to leave it up, which is what recording needs.'),
        DeclareLaunchArgument(
            'check', default_value='true',
            description='Ask the epistemic state, once the robots have '
                        'stopped, whether the goal actually came out.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the warehouse fleet too. false when it is '
                        'already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
