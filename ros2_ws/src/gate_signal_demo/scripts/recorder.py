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
Records one run of the gate signal. It observes and never acts.

    recorder.py --dir RUN

RUN/actions.csv  every change of state of an action of the plan as each
                 executor publishes it on its action_execution_info topic: the
                 team's under the epistemic policy, each robot's own in the
                 classical runs;
RUN/pose.csv     both robots' poses from the simulator's odometry, which is
                 the world pose, at 5 Hz each, with simulated and wall time.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from plansys2_msgs.msg import ActionExecutionInfo
from rclpy.node import Node

STATUS = {1: 'NOT_EXECUTED', 2: 'EXECUTING', 3: 'FAILED', 4: 'SUCCEEDED', 5: 'CANCELLED'}


class Recorder(Node):

    def __init__(self, run):
        super().__init__('gate_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.actions = csv.writer(open(os.path.join(run, 'actions.csv'), 'w', newline='', buffering=1))
        self.actions.writerow(['t_sim', 't_wall', 'stack', 'action', 'arguments', 'status', 'outcome',
                               'message'])
        self.pose = csv.writer(open(os.path.join(run, 'pose.csv'), 'w', newline='', buffering=1))
        self.pose.writerow(['t_sim', 't_wall', 'robot', 'x', 'y', 'yaw'])
        self.last_status = {}
        self.last_pose = {}
        for stack in ('', '/r1', '/r2'):
            self.create_subscription(ActionExecutionInfo, f'{stack}/action_execution_info',
                                     lambda m, s=stack: self.on_action(s, m), 50)
        for r in ('r1', 'r2'):
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_action(self, stack, m):
        # One row per change of state of one node of the plan. A policy holds
        # the same action in several branches, and the nodes not taken stay
        # NOT_EXECUTED throughout, so they are not written.
        key = (stack, m.action_full_name or m.action, tuple(m.arguments))
        status = STATUS.get(m.status, str(m.status))
        if status == 'NOT_EXECUTED' or self.last_status.get(key) == status:
            return
        self.last_status[key] = status
        self.actions.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', stack.strip('/') or 'team', m.action,
                               ' '.join(m.arguments), status, m.outcome, m.message_status])

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
