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
Records one run of the lookout board. It observes and never acts.

    recorder.py --dir RUN

RUN/regions.csv  for every grid the mapper publishes: simulated and wall time,
                 and for each region, the zone, the four windows and the four
                 slots of the board, its class by the perception's rule and
                 how many of its cells have been observed;
RUN/actions.csv  every change of an action's state as the executor publishes
                 it on /action_execution_info;
RUN/pose.csv     the robot's pose from the simulator's odometry, which is the
                 world pose, at 2 Hz.
"""

import argparse
import csv
import os
import sys
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from plansys2_msgs.msg import ActionExecutionInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('lookout_board_demo'), 'tools'))
import layout as L  # noqa: E402

STATUS = {1: 'NOT_EXECUTED', 2: 'EXECUTING', 3: 'FAILED', 4: 'SUCCEEDED', 5: 'CANCELLED'}
REGIONS = ([('zone', L.zone_region())]
           + [(f'window_{k}', L.window_region(k)) for k in range(1, 5)]
           + [(f'slot_{k}', L.slot_region(k)) for k in range(1, 5)])


def cells(grid, box):
    info = grid.info
    r, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    out = []
    for j in range(info.height):
        cy = oy + (j + 0.5) * r
        if box[1] <= cy <= box[3]:
            for i in range(info.width):
                cx = ox + (i + 0.5) * r
                if box[0] <= cx <= box[2]:
                    out.append(grid.data[j * info.width + i])
    return out


def classify(values):
    if any(v > 65 for v in values):
        return 'blocked'
    if values and all(0 <= v < 25 for v in values):
        return 'clear'
    return 'unknown'


class Recorder(Node):

    def __init__(self, run):
        super().__init__('lookout_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.regions = csv.writer(open(os.path.join(run, 'regions.csv'), 'w', newline='', buffering=1))
        head = ['t_sim', 't_wall']
        for name, _ in REGIONS:
            head += [f'{name}_class', f'{name}_seen', f'{name}_cells']
        self.regions.writerow(head)
        self.actions = csv.writer(open(os.path.join(run, 'actions.csv'), 'w', newline='', buffering=1))
        self.actions.writerow(['t_sim', 't_wall', 'action', 'arguments', 'status', 'outcome', 'message'])
        self.pose = csv.writer(open(os.path.join(run, 'pose.csv'), 'w', newline='', buffering=1))
        self.pose.writerow(['t_sim', 'x', 'y', 'yaw'])
        self.last_status = {}
        self.last_pose = -1.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(ActionExecutionInfo, '/action_execution_info', self.on_action, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, grid):
        row = [f'{self.sim():.2f}', f'{time.time():.3f}']
        for _, box in REGIONS:
            v = cells(grid, box)
            row += [classify(v), sum(1 for x in v if x >= 0), len(v)]
        self.regions.writerow(row)

    def on_action(self, m):
        key = (m.action, tuple(m.arguments))
        status = STATUS.get(m.status, str(m.status))
        if self.last_status.get(key) == status:
            return
        self.last_status[key] = status
        self.actions.writerow([f'{self.sim():.2f}', f'{time.time():.3f}', m.action,
                               ' '.join(m.arguments), status, m.outcome, m.message_status])

    def on_odom(self, m):
        t = self.sim()
        if t - self.last_pose < 0.5:
            return
        self.last_pose = t
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose.writerow([f'{t:.2f}', f'{m.pose.pose.position.x:.3f}',
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
