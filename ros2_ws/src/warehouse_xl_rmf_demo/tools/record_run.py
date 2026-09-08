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

"""Records where the robots actually went, from `/fleet_states`, to a CSV.

A recording of a run is worth more than a screenshot of one: it is the floor's
own claim, that a robot can only reach an aisle by way of the lane, put to the
fleet and answered in metres.

    record_run.py --seconds 180 --out run.csv
"""

import argparse
import csv

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import FleetState


class Recorder(Node):
    def __init__(self, path, seconds):
        super().__init__('warehouse_xl_recorder')
        self.rows = []
        self.path = path
        self.create_subscription(FleetState, '/fleet_states', self.on_state, 10)
        self.start = self.get_clock().now()
        self.deadline = seconds

    def on_state(self, msg):
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        for r in msg.robots:
            self.rows.append((round(t, 2), r.name, round(r.location.x, 3),
                              round(r.location.y, 3), round(r.location.yaw, 3),
                              r.task_id))
        if t > self.deadline:
            raise SystemExit

    def write(self):
        with open(self.path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(('t', 'robot', 'x', 'y', 'yaw', 'task_id'))
            w.writerows(self.rows)
        print(f'{self.path}: {len(self.rows)} samples')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=180.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.out, args.seconds)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.write()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
