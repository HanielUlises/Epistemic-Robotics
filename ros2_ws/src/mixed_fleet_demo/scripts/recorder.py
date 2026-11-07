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
Records one run of belief revision in a mixed fleet. It observes and never acts.

    recorder.py --dir RUN --robots c d

RUN/actions.csv  every change of state of an action of the policy, as the
                 executor publishes it on /action_execution_info;
RUN/pose.csv     the robots' poses from the simulator's odometry, which is
                 the world pose, and the porter's, at 5 Hz each;
RUN/radio.csv    every report on /radio;
RUN/doors.csv    every change of the fire doors, from /doors;
RUN/state.csv    every change of the epistemic state's size, from
                 epistemic_state/state.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from plansys2_msgs.msg import ActionExecutionInfo
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

STATUS = {1: 'NOT_EXECUTED', 2: 'EXECUTING', 3: 'FAILED', 4: 'SUCCEEDED', 5: 'CANCELLED'}
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Recorder(Node):

    def __init__(self, run, robots):
        super().__init__('fleet_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

        def table(name, header):
            w = csv.writer(open(os.path.join(run, name), 'w', newline='', buffering=1))
            w.writerow(header)
            return w
        self.actions = table('actions.csv', ['t_sim', 't_wall', 'action', 'arguments', 'status', 'outcome',
                                             'message'])
        self.pose = table('pose.csv', ['t_sim', 't_wall', 'robot', 'x', 'y', 'yaw'])
        self.radio = table('radio.csv', ['t_sim', 't_wall', 'message'])
        self.doors = table('doors.csv', ['t_sim', 't_wall', 'a', 'b', 'c', 'd'])
        self.state = table('state.csv', ['t_sim', 't_wall', 'worlds', 'designated'])
        self.last_state = None
        self.last_status = {}
        self.last_pose = {}
        self.create_subscription(ActionExecutionInfo, '/action_execution_info', self.on_action, 50)
        for r in robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        self.create_subscription(String, '/porter/state', self.on_porter, 10)
        self.create_subscription(String, '/radio', self.on_radio, LATCHED)
        self.create_subscription(String, '/doors', self.on_doors, LATCHED)
        self.create_subscription(String, '/epistemic_state/state', self.on_state, QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_action(self, m):
        key = (m.action_full_name or m.action, tuple(m.arguments))
        status = STATUS.get(m.status, str(m.status))
        if status == 'NOT_EXECUTED' or self.last_status.get(key) == status:
            return
        self.last_status[key] = status
        self.actions.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', m.action, ' '.join(m.arguments), status,
                               m.outcome, m.message_status])

    def on_radio(self, m):
        self.radio.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', m.data])

    def on_doors(self, m):
        d = json.loads(m.data)['open']
        self.doors.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', *[int(d[k]) for k in 'abcd']])

    def on_state(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        key = (d.get('num_worlds', d.get('worlds')), d.get('num_designated', d.get('designated')))
        if key != self.last_state:
            self.last_state = key
            self.state.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', *key])

    def on_porter(self, m):
        parts = m.data.split()
        if len(parts) < 3:
            return
        t = self.sim()
        if t - self.last_pose.get('p', -1.0) < 0.2:
            return
        self.last_pose['p'] = t
        self.pose.writerow([f'{t:.2f}', f'{time.time():.3f}', 'p', parts[1], parts[2], ''])

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
    ap.add_argument('--robots', nargs='+', default=['c'])
    a = ap.parse_args(argv)
    rclpy.init()
    node = Recorder(a.dir, a.robots)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
