#!/usr/bin/env python3
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
Decides the fire doors with one robot's own laser.

    door_node.py --robot c

While the robot stands within NEAR of the bend of a lobby, on either side of
the band, each scan's returns are put in the world frame by the robot's
odometry and the door decided by grid.decide, the rule tools/check_views.py
checks in every state of the doors. A decision is published on /<robot>/door
as "<door>:<s|n>:<open|shut>"; a scan that decides nothing publishes nothing,
and so does a robot anywhere but at a bend.
"""

import argparse
import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_revision_demo'), 'tools'))
import grid as G  # noqa: E402
import layout as L  # noqa: E402

NEAR = 0.5
LASER_X = -0.032          # the Burger's laser sits this far behind its base


class Doors(Node):

    def __init__(self, robot):
        super().__init__(f'doors_{robot}', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.robot = robot
        self.pose = None
        self.pub = self.create_publisher(String, f'/{robot}/door', 10)
        self.create_subscription(Odometry, f'/{robot}/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, f'/{robot}/scan', self.on_scan, qos_profile_sensor_data)
        self.bends = [(d, side, G.bend_point(d, side)) for d in L.DOORS for side in ('S', 'N')]

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_scan(self, m):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        near = [(d, side) for d, side, p in self.bends if math.dist((x, y), p) <= NEAR]
        if not near:
            return
        lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
        pts = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = yaw + m.angle_min + i * m.angle_increment
            pts.append((lx + r * math.cos(a), ly + r * math.sin(a)))
        for d, side in near:
            v = G.decide(d, side, pts)
            if v:
                self.pub.publish(String(data=f'{d}:{side.lower()}:{v}'))


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Doors(a.robot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
