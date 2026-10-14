#!/usr/bin/env python3
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
The watcher, which may change posts.

    watcher_node.py --strategy stay|chase --dir RUN

stay   it keeps the post it was given.
chase  it knows how the carrier decides. A carrier that reads the duty board
       will tell the ally on the side the watcher is not, so the moment the
       carrier starts its first drive the watcher leaves for the other post,
       through the passage behind the north wall, out of everyone's sight; it
       has one change of post, and uses it then.

The watcher acts only on when the carrier starts to drive, which it is given
as the executor's action states: a stronger watcher than one that has to see
it. It drives with a plain pursuit along the passage, which has nothing in it.
Writes RUN/watcher.json: the strategy, and when it left and arrived.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from plansys2_msgs.msg import ActionExecutionInfo
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('watcher_moves_demo'), 'tools'))
import layout as L  # noqa: E402

DRIVES = ('drive_', 'to_window', 'to_gate', 'back_route')
SPEED, TURN = 0.22, 1.4


class Watcher(Node):

    def __init__(self, a):
        super().__init__('watcher_strategy', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.pose = None
        self.legs = []
        self.record = {'strategy': a.strategy, 'moves': []}
        self.cmd = self.create_publisher(Twist, '/r3/cmd_vel', 10)
        self.create_subscription(Odometry, '/r3/odom', self.on_odom, 10)
        self.create_subscription(ActionExecutionInfo, '/action_execution_info', self.on_action, 50)
        self.create_timer(0.1, self.step)
        self.write()

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write(self):
        json.dump(self.record, open(os.path.join(self.a.dir, 'watcher.json'), 'w'), indent=1)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_action(self, m):
        if self.a.strategy != 'chase' or self.record['moves'] or self.pose is None:
            return
        if m.status != ActionExecutionInfo.EXECUTING or not m.action.startswith(DRIVES):
            return
        here = 'w' if self.pose[0] < 0 else 'e'
        there = 'e' if here == 'w' else 'w'
        nodes, _ = L.roadmap()
        self.legs = [nodes[p] for p in L.route_watcher(here, there)[1:]]
        self.record['moves'].append({'from': here, 'to': there, 'trigger': m.action, 't_left': round(self.sim(), 2)})
        self.get_logger().info(f'[watcher] the carrier started {m.action}; leaving for the {there} post')
        self.write()

    def step(self):
        if not self.legs or self.pose is None:
            return
        x, y, yaw = self.pose
        tx, ty = self.legs[0]
        d = math.hypot(tx - x, ty - y)
        out = Twist()
        if d < (0.12 if len(self.legs) == 1 else 0.25):
            self.legs.pop(0)
            if not self.legs:
                self.cmd.publish(out)
                self.record['moves'][-1]['t_arrived'] = round(self.sim(), 2)
                self.get_logger().info('[watcher] at its new post')
                self.write()
            return
        err = math.atan2(ty - y, tx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        out.angular.z = max(-TURN, min(TURN, 2.5 * err))
        out.linear.x = 0.0 if abs(err) > 0.5 else SPEED * max(0.3, 1 - abs(err))
        self.cmd.publish(out)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', choices=['stay', 'chase'], default='stay')
    ap.add_argument('--dir', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Watcher(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
