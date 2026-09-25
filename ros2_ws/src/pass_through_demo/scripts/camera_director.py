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
Points the Gazebo camera at whichever robot the policy has set to work.

The performers announce on /pass_through/acting which agent has just started
an action that moves it. This writes that agent's shot into the file
warehouse_xl_rmf_demo's chase_camera re-reads with --follow-file, and the
camera cuts to it. A map exchange moves nobody, so it announces nothing and the
camera stays where it was.

A shot is either a model name, which the camera chases, or `pose X Y Z PITCH
YAW`, which it holds. The scouts get fixed shots over the mouth of the bay they
read: a chase camera is placed behind a robot's heading when it cuts to it,
and a scout starts facing along its aisle with a wall or a rack row behind it.
A fixed shot over the mouth also has the load in frame, which is the thing the
scout is about to find.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import String


class CameraDirector(Node):

    def __init__(self):
        super().__init__('camera_director')
        self.declare_parameter('agents', ['west', 'east', 'carrier'])
        self.declare_parameter('namespaces', ['r1', 'r2', 'r3'])
        self.declare_parameter('follow_file', '/tmp/pass_through_follow')
        self.declare_parameter('initial', 'r3')
        # agent=shot, overriding the chase of that agent's model.
        self.declare_parameter('shots', [''])
        self.models = dict(zip(self.get_parameter('agents').value,
                               self.get_parameter('namespaces').value))
        for entry in self.get_parameter('shots').value:
            if '=' in entry:
                agent, shot = entry.split('=', 1)
                self.models[agent.strip()] = shot.strip()
        self.path = self.get_parameter('follow_file').value
        self.write(self.get_parameter('initial').value)
        self.create_subscription(
            String, '/pass_through/acting', self.on_acting,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def write(self, model):
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as fh:
            fh.write(model + '\n')
        os.replace(tmp, self.path)

    def on_acting(self, msg):
        model = self.models.get(msg.data)
        if model:
            self.write(model)
            self.get_logger().info(f'[camera] following {msg.data} ({model})')


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
