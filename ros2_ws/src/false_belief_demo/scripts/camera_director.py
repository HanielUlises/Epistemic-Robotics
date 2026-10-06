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
Points the Gazebo camera at what the policy is doing.

The performers announce a shot on /false_belief/shot as they start: an
agent's name when that robot drives somewhere, a bay's name when the crate is
set down or looked for there, `dock` while the picker charges and when a report
reaches it. This writes the shot into the file warehouse_xl_rmf_demo's
chase_camera re-reads with --follow-file: a robot's model name, which it
chases, or `pose X Y Z PITCH YAW`, which it holds.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import String


class CameraDirector(Node):

    def __init__(self):
        super().__init__('camera_director')
        self.declare_parameter('follow_file', '/tmp/false_belief_follow')
        self.declare_parameter('initial', '')
        # name=shot, one per entry: agents to model names, places to poses.
        self.declare_parameter('shots', [''])
        self.shots = {}
        for entry in self.get_parameter('shots').value:
            if '=' in entry:
                name, shot = entry.split('=', 1)
                self.shots[name.strip()] = shot.strip()
        self.path = self.get_parameter('follow_file').value
        initial = self.get_parameter('initial').value
        if initial:
            self.write(initial)
        self.create_subscription(
            String, '/false_belief/shot', self.on_shot,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def write(self, shot):
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as fh:
            fh.write(shot + '\n')
        os.replace(tmp, self.path)

    def on_shot(self, msg):
        shot = self.shots.get(msg.data)
        if shot:
            self.write(shot)
            self.get_logger().info(f'[camera] {msg.data}: {shot}')


def main():
    rclpy.init()
    node = CameraDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
