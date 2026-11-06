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
The news of one run, from outside the plan.

    news.py --dir RUN --act 2 --formulation replan

At worlds.NEWS_AT seconds into the mission the news is said: published on
/news, which every robot and the classical mission hear, and, under the
replan formulation, announced into the executor's epistemic state with
epistemic_state/announce, which keeps the worlds where it holds and counts a
change of belief. Under null a tautology is announced at the same instant
instead: the policy is interrupted, and the model is left as it was. Under keep
and classical nothing is announced. Writes RUN/news.json.
"""

import argparse
import json
import os
import sys
import time

import rclpy
from plansys2_epistemic_msgs.srv import Announce
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('replanning_news_demo'), 'tools'))
import model as M  # noqa: E402
import worlds as W  # noqa: E402


class News(Node):

    def __init__(self, a):
        super().__init__('news', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.pub = self.create_publisher(String, '/news', QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go(self):
        a = self.a
        at = W.START_AT + W.NEWS_AT
        while rclpy.ok() and self.sim() < at:
            rclpy.spin_once(self, timeout_sec=0.05)
        formula = {'replan': M.ANNOUNCE[a.act], 'null': W.TAUTOLOGY}.get(a.formulation)
        record = {'act': a.act, 'formulation': a.formulation, 't_sim': round(self.sim(), 2),
                  't_mission': round(self.sim() - W.START_AT, 2), 't_wall': time.time(),
                  'text': M.NEWS_TEXT[a.act], 'kind': M.NEWS_KIND[a.act], 'formula': M.ANNOUNCE[a.act],
                  'announced': formula}
        self.pub.publish(String(data=json.dumps(record)))
        self.get_logger().info(f'[news] {M.NEWS_TEXT[a.act]}')
        if formula:
            client = self.create_client(Announce, '/epistemic_state/announce')
            if client.wait_for_service(timeout_sec=10.0):
                future = client.call_async(Announce.Request(formula=formula))
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                r = future.result()
                record['answer'] = None if r is None else {
                    'success': r.success, 'error': r.error, 'worlds': r.num_worlds, 'designated': r.num_designated}
            else:
                record['answer'] = 'no epistemic state'
            self.get_logger().info(f'[news] announced {formula}: {record["answer"]}')
        json.dump(record, open(os.path.join(a.dir, 'news.json'), 'w'), indent=1)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True, choices=W.FORMULATIONS)
    a = ap.parse_args(argv)
    rclpy.init()
    node = News(a)
    try:
        node.go()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
