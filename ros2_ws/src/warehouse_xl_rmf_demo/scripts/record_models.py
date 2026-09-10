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
Records the epistemic model after each product update of a mission.

`epistemic_state` publishes its state on `epistemic_state/state`, and with
`publish_model` set it includes the model itself: the worlds, the atoms true at
each, the designated set, and one accessibility relation per agent. That is the
whole structure, and it is what a figure of a Kripke model has to be drawn from
if the figure is to be evidence and not illustration.

One file is written per distinct model, numbered in the order they occurred,
alongside a manifest. A model identical to its predecessor is not written
twice: the state is republished on a timer as well as on change, and a
directory of two hundred copies of six models is not a record of anything.

The message carries no name for the action that produced the model it holds.
The manifest therefore records the shape and the ordinal, and the action is
recovered by position against the `applied` lines of the run's own log, which
are emitted in the same order by the same node.

    record_models.py --ros-args -p out_dir:=/tmp/models
"""

import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String


class ModelRecorder(Node):
    def __init__(self):
        super().__init__('record_models')

        self.declare_parameter('state_topic', '/epistemic_state/state')
        self.declare_parameter('out_dir', '/tmp/epistemic_models')

        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.seen = []          # the models, in order of first appearance
        self.manifest = []      # what produced each

        # Transient local, matching the publisher: the first model is published
        # when the state activates, which is before this node is likely to have
        # subscribed, and a record missing its initial model is a record of a
        # sequence with no beginning.
        qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, self.get_parameter('state_topic').value, self.on_state, qos)

        self.get_logger().info(f'recording models to {self.out_dir}')

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        model = payload.get('model')
        if not model:
            return

        # Identity is the model, not the message. The state is republished
        # periodically and the same structure arrives many times.
        key = json.dumps(model, sort_keys=True)
        if self.seen and self.seen[-1] == key:
            return
        if key in self.seen:
            return

        index = len(self.seen)
        self.seen.append(key)
        self.manifest.append({
            'index': index,
            'worlds': len(model.get('worlds', [])),
            'designated': len(model.get('designated', [])),
            'goal_holds': payload.get('goal_holds'),
        })

        path = os.path.join(self.out_dir, f'model_{index:02d}.json')
        with open(path, 'w') as handle:
            json.dump(model, handle, indent=2, sort_keys=True)
        with open(os.path.join(self.out_dir, 'manifest.json'), 'w') as handle:
            json.dump(self.manifest, handle, indent=2)

        self.get_logger().info(
            f'model {index}: {len(model.get("worlds", []))} worlds, '
            f'{len(model.get("designated", []))} designated, '
            f'goal {payload.get("goal_holds")}')


def main():
    rclpy.init()
    node = ModelRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
