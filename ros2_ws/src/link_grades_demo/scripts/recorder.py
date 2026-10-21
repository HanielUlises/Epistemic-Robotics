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
Records one run of the link grades. It observes and never acts.

    recorder.py --dir RUN

RUN/actions.csv  every change of state of an action of the plan, as the
                 executor publishes it on /action_execution_info;
RUN/pose.csv     both robots' poses from the simulator's odometry, which is the
                 world pose, at 5 Hz each, with simulated and wall time;
RUN/link.csv     every event of comm_monitor: the instant it declared the link
                 down or up, and the simulated time it was heard;
RUN/gate.csv     every message offered to a gate, passed or dropped by the link
                 state at that moment, and every message a gate delivered.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from epistemic_msgs.msg import LinkEvent
from nav_msgs.msg import OccupancyGrid, Odometry
from plansys2_msgs.msg import ActionExecutionInfo
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

STATUS = {1: 'NOT_EXECUTED', 2: 'EXECUTING', 3: 'FAILED', 4: 'SUCCEEDED', 5: 'CANCELLED'}
EVENTS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=8, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
# input -> output of each gate, and the type
GATES = [('/r1/grid', '/r2/peer_grid', OccupancyGrid), ('/r2/grid', '/r1/peer_grid', OccupancyGrid),
         ('/r1/decision', '/r2/peer_decision', String), ('/r2/decision', '/r1/peer_decision', String),
         ('/r1/report', '/r2/report_in', String), ('/r2/report', '/r1/report_in', String)]


class Recorder(Node):

    def __init__(self, run):
        super().__init__('link_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

        def table(name, header):
            w = csv.writer(open(os.path.join(run, name), 'w', newline='', buffering=1))
            w.writerow(header)
            return w
        self.actions = table('actions.csv', ['t_sim', 't_wall', 'action', 'arguments', 'status', 'outcome',
                                             'message'])
        self.pose = table('pose.csv', ['t_sim', 't_wall', 'robot', 'x', 'y', 'yaw'])
        self.link = table('link.csv', ['t_declared', 't_heard', 'state', 'previous_held'])
        self.gate = table('gate.csv', ['t_sim', 'topic', 'event'])
        self.up = True
        self.last_status = {}
        self.last_pose = {}
        self.create_subscription(ActionExecutionInfo, '/action_execution_info', self.on_action, 50)
        for r in ('r1', 'r2'):
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        self.create_subscription(LinkEvent, '/comm_monitor/link_down', self.on_link, EVENTS)
        self.create_subscription(LinkEvent, '/comm_monitor/link_up', self.on_link, EVENTS)
        for src, dst, kind in GATES:
            self.create_subscription(kind, src, lambda m, s=src: self.on_offered(s), 10)
            self.create_subscription(kind, dst, lambda m, d=dst: self.gate.writerow(
                [f'{self.sim():.2f}', d, 'delivered']), 10)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_action(self, m):
        # One row per change of state of one node of the plan. A policy holds
        # the same action in several branches, and the nodes not taken stay
        # NOT_EXECUTED throughout, so they are not written.
        key = (m.action_full_name or m.action, tuple(m.arguments))
        status = STATUS.get(m.status, str(m.status))
        if status == 'NOT_EXECUTED' or self.last_status.get(key) == status:
            return
        self.last_status[key] = status
        self.actions.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', m.action, ' '.join(m.arguments), status,
                               m.outcome, m.message_status])

    def on_link(self, m):
        self.up = m.up
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.link.writerow([f'{stamp:.2f}', f'{self.sim():.2f}', 'up' if m.up else 'down',
                            f'{m.previous_state_held_for:.2f}'])

    def on_offered(self, src):
        self.gate.writerow([f'{self.sim():.2f}', src, 'passed' if self.up else 'dropped'])

    def on_odom(self, r, m):
        t = self.sim()
        if t - self.last_pose.get(r, -1.0) < 0.2:
            return
        self.last_pose[r] = t
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose.writerow([f'{t:.2f}', f'{time.time():.3f}', r, f'{m.pose.pose.position.x:.3f}',
                            f'{m.pose.pose.position.y:.3f}', f'{yaw:.3f}'])


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Recorder(a.dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
