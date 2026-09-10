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
The warehouse fleet of two, with the radio between them cut for a while.

    ros2 launch epistemic_comm link_outage_launch.py
    ros2 launch epistemic_comm link_outage_launch.py t_disc:=40 t_recon:=100

What this adds to `warehouse_demo`, which it includes unmodified:

  comm_monitor   owns the link state and is the only thing that does. It
                 publishes /comm_monitor/link_down and /comm_monitor/link_up
                 at the configured instants, which is RF-05's t_disc and
                 t_recon
  link_gate      two of them, one for the partner's odometry and one for the
                 partner's map. Each reads the topic the fleet already
                 publishes and writes a copy that only r1's side reads, and
                 stops writing between the two events
  belief_update  RF-05's engine, on r2, believing about r1. It reads the gated
                 odometry, so the outage is real for it, and integrates the
                 last twist it saw while nothing arrives
  map_fusion     section 5.9's reconciliation, from epistemic_slam, given r2's
                 own map and the gated copy of r1's

Which robot believes about which is not free. In the two-robot instance r2 is
the one whose knowledge the goal is about and it never leaves the receiving
dock, while r1 does the driving. Believing about a robot that does not move
would make the propagation right by default and the measurement vacuous, so
the belief is r2's and its subject is r1.
  reconcile      calls the fusion when the link returns and counts what each
                 robot learned
  grid_align     puts the two SLAM maps on the union of their extents before
                 the fusion sees them. Two maps that two robots actually built
                 are never the same shape, and 5.9 refuses maps whose
                 discretisation does not coincide
  patrol         drives r1 on a fixed square. The mission's own driving does
                 not work in this workspace -- r1's global costmap never
                 receives its map and nav2 cannot plan -- and RF-05 needs a
                 partner that moves, not a partner that is pursuing a goal
  link_experiment measures RF-05's criterion. It reads the UNGATED odometry,
                 because the error to measure is against where r1 actually
                 went and not against the silence r2 was reasoning from

The two robots drive their own mission throughout. Nothing here steers them,
and nothing here tells the planner that the link fell: the point of the
scenario is what happens to a belief while the system carries on unaware.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The platform's own limit, from warehouse_demo/params/nav2.yaml, where
# max_vel_x and max_speed_xy are both 0.22. RF-05's bound is v_max*dt +
# sigma_prop, and taking v_max from anywhere but the controller that enforces
# it would be measuring against a speed the robot cannot reach.
V_MAX = 0.22

# The standard deviation of the propagation model, in metres. It is the part of
# the bound that does not grow, and it stands for what is unknown about the
# partner's pose at the instant the link falls rather than for anything that
# happens afterwards. A wheel-odometry pose on this platform is good to a few
# centimetres over the distances involved.
SIGMA_PROP = 0.05

# Growth of each positional variance per second of outage, in m^2/s. With two
# axes the trace grows at twice this, so it reaches RF-05's sigma_max^2 of
# 1.0 m^2 at fifty seconds: an outage longer than that ends with the partner's
# position marked uncertain, which is the invariant the requirement asks for
# and is worth being able to reach inside one run.
VARIANCE_RATE = 0.01


def setup(context, *args, **kwargs):
    demo = get_package_share_directory('warehouse_demo')
    scenario = LaunchConfiguration('scenario').perform(context)
    t_disc = LaunchConfiguration('t_disc').perform(context)
    t_recon = LaunchConfiguration('t_recon').perform(context)
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir, exist_ok=True)

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(demo, 'launch', 'warehouse_demo_launch.py')),
        launch_arguments={
            'robots': '2',
            # The fleet's own rviz watches one robot's namespace and cannot
            # show a belief one robot holds about another. This scenario
            # brings its own, and takes Gazebo from the fleet.
            'gui': LaunchConfiguration('gazebo_gui'),
        }.items())

    monitor = Node(
        package='epistemic_comm', executable='comm_monitor',
        name='comm_monitor', output='screen',
        parameters=[{
            'pair': ['r1', 'r2'],
            't_disc': float(t_disc),
            't_recon': float(t_recon),
            'use_sim_time': True,
        }])

    def gate(name, source, sink, kind, latch=False):
        return Node(
            package='epistemic_comm', executable='link_gate',
            name=name, output='screen',
            parameters=[{
                'input': source,
                'output': sink,
                'type': kind,
                'transient_local': latch,
                'use_sim_time': True,
            }])

    gates = [
        gate('gate_odom', '/r1/odom', '/r2/partner_odom',
             'nav_msgs/msg/Odometry'),
        # The map is latched: a reader that comes up between two publications
        # expects to find the last one, and a gate that did not latch would
        # hand it nothing until r2's SLAM next published.
        gate('gate_map', '/r1/map', '/r2/partner_map',
             'nav_msgs/msg/OccupancyGrid', latch=True),
    ]

    belief = Node(
        package='epistemic_comm', executable='belief_update',
        name='belief_update_r2', output='screen',
        parameters=[{
            'holder': 'r2',
            'subject': 'r1',
            'partner_odom': '/r2/partner_odom',
            'rate': 10.0,
            'v_max': V_MAX,
            'sigma_prop': SIGMA_PROP,
            'variance_rate': VARIANCE_RATE,
            'sigma_max_squared': 1.0,
            'use_sim_time': True,
        }])

    align = Node(
        package='epistemic_comm', executable='grid_align.py',
        name='grid_align', output='screen',
        parameters=[{
            'in_a': '/r2/map',
            'in_b': '/r2/partner_map',
            'out_a': '/aligned/map_a',
            'out_b': '/aligned/map_b',
            'use_sim_time': True,
        }])

    patrol = Node(
        package='epistemic_comm', executable='patrol',
        name='patrol_r1', output='screen',
        parameters=[{
            'cmd_vel': '/r1/cmd_vel',
            'speed': 0.15,
            'turn_rate': 0.5,
            'side_seconds': 12.0,
            'start_after': 5.0,
            'use_sim_time': True,
        }])

    fusion = Node(
        # The executable is `map_fusion`: epistemic_slam builds the node as a
        # library called map_fusion_node and renames the binary that wraps it.
        package='epistemic_slam', executable='map_fusion',
        name='map_fusion', output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('~/map_a', '/aligned/map_a'),
            ('~/map_b', '/aligned/map_b'),
        ])

    reconcile = Node(
        package='epistemic_comm', executable='reconcile.py',
        name='reconcile', output='screen',
        parameters=[{
            'map_a': '/aligned/map_a',
            'map_b': '/aligned/map_b',
            'merged': '/map_fusion/merged',
            'fuse_service': '/map_fusion/fuse',
            'out': os.path.join(out_dir, f'reconcile_{scenario}.json'),
            'use_sim_time': True,
        }])

    viewer = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('epistemic_comm'),
            'config', 'link_outage.rviz')],
        parameters=[{'use_sim_time': True}])

    experiment = Node(
        package='epistemic_comm', executable='link_experiment',
        name='link_experiment', output='screen',
        parameters=[{
            'belief_topic': '/r2/belief/partner',
            # Ungated, deliberately. See the module docstring.
            'truth_topic': '/r1/odom',
            'out': os.path.join(out_dir, f'link_{scenario}.json'),
            'use_sim_time': True,
        }])

    # Held back, and not because of a race in this package. The fleet's own
    # bringup takes tens of seconds to have a clock, a map and an odometry, and
    # a monitor that starts its schedule against a clock that reads zero fires
    # the whole outage before the robots have moved.
    staged = TimerAction(
        period=25.0,
        actions=[monitor, *gates, belief, align, patrol, fusion, reconcile,
                 viewer, experiment])

    return [fleet, staged]


def generate_launch_description():
    from launch.actions import OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='baseline',
            description='Names the JSON files this run writes.'),
        DeclareLaunchArgument(
            't_disc', default_value='40.0',
            description='Seconds from the monitor\'s first tick to the cut.'),
        DeclareLaunchArgument(
            't_recon', default_value='100.0',
            description='Seconds from the monitor\'s first tick to the repair.'),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Gazebo client, from the fleet launch.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='This scenario\'s own rviz, showing the belief.'),
        DeclareLaunchArgument(
            'out_dir', default_value='/tmp/epistemic_comm',
            description='Where the two measurement records are written.'),
        OpaqueFunction(function=setup),
    ])
