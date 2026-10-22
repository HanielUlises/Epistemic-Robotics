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
What the world of the belief and empathy demonstration does, apart from its
physics: the radio and the task.

    world_node.py --dir RUN --range 2.5

the radio  a message on /radio/send, {from, to, body}, reaches /<to>/inbox
           when the two robots are within --range metres of each other in the
           plane, by their odometry, at the moment it is sent; otherwise it is
           lost. Every message, delivered or not, goes to /radio/log. Under the
           ideal formulation the range is unlimited.
the task   it is found when any robot first comes within layout.R_DETECT of its
           site, which /task/found says, latched, with who found it and when;
           it is done when the aerial vehicle and a ground vehicle with fewer
           than two faults have both been within layout.R_SITE of the site for
           layout.DELTA seconds, which /task/done says.

In the variant with latent tool faults, the task is attempted with a named
vehicle, which the aerial vehicle announces on /task/attempt: once the two have
been within layout.R_SITE of the site for layout.DELTA seconds, the task is
done if that vehicle's tool works, and otherwise the attempt fails, which
/task/failed says. The world knows the tools from RUN/world.json; nobody else
does.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_empathy_demo'), 'tools'))
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class World(Node):

    def __init__(self, a):
        super().__init__('be_world', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.range = a.range
        world = json.load(open(os.path.join(a.dir, 'world.json')))
        self.site = tuple(world['site'])
        self.latent = world.get('variant') == 'latent'
        self.tool_fault = world.get('tool_fault', {})
        self.attempt, self.failed = None, set()
        self.pose, self.capable = {}, {g: True for g in L.UGVS}
        self.found, self.done, self.together = None, None, None
        self.inbox = {r: self.create_publisher(String, f'/{r}/inbox', 50) for r in L.ROBOTS}
        self.log = self.create_publisher(String, '/radio/log', 50)
        self.found_pub = self.create_publisher(String, '/task/found', LATCHED)
        self.done_pub = self.create_publisher(String, '/task/done', LATCHED)
        self.failed_pub = self.create_publisher(String, '/task/failed', LATCHED)
        self.create_subscription(String, '/task/attempt', self.on_attempt, 10)
        for r in L.ROBOTS:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.pose.__setitem__(
                r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        for g in L.UGVS:
            self.create_subscription(String, f'/{g}/state', lambda m, g=g: self.capable.__setitem__(
                g, json.loads(m.data)['capable']), LATCHED)
        self.create_subscription(String, '/radio/send', self.on_send, 50)
        self.create_timer(0.1, self.tick)

    def rel(self):
        return self.get_clock().now().nanoseconds * 1e-9 - W.START_AT

    def on_attempt(self, m):
        g = m.data
        if g != self.attempt and g not in self.failed:
            self.attempt, self.together = g, None
            self.get_logger().info(f'an attempt with {g}')

    def on_send(self, m):
        msg = json.loads(m.data)
        a, b = self.pose.get(msg['from']), self.pose.get(msg['to'])
        d = math.dist(a, b) if a and b else float('inf')
        delivered = d <= self.range
        if delivered:
            self.inbox[msg['to']].publish(m)
        self.log.publish(String(data=json.dumps({'t': round(self.rel(), 2), 'from': msg['from'], 'to': msg['to'],
                                                 'kind': msg['body'].get('kind'), 'delivered': delivered,
                                                 'd': round(d, 2) if math.isfinite(d) else None})))

    def tick(self):
        t = self.rel()
        if t < 0 or len(self.pose) < len(L.ROBOTS):
            return
        if self.found is None:
            for r in L.ROBOTS:
                if math.dist(self.pose[r], self.site) <= L.R_DETECT:
                    self.found = {'t': round(t, 2), 'by': r, 'site': list(self.site)}
                    self.found_pub.publish(String(data=json.dumps(self.found)))
                    self.get_logger().info(f'the task is found by {r} at {t:.1f} s')
                    break
        if self.found is None or self.done is not None:
            return
        near = lambda r: math.dist(self.pose[r], self.site) <= L.R_SITE  # noqa: E731
        if self.latent:
            self.attempt_tick(t, near)
            return
        with_ = [g for g in L.UGVS if near(g) and self.capable[g]]
        if near('u') and with_:
            if self.together is None:
                self.together = (t, with_[0])
            elif t - self.together[0] >= L.DELTA:
                self.done = {'t': round(t, 2), 'by': self.together[1]}
                self.done_pub.publish(String(data=json.dumps(self.done)))
                self.get_logger().info(f'the task is done by u and {self.together[1]} at {t:.1f} s')
        else:
            self.together = None


    def attempt_tick(self, t, near):
        g = self.attempt
        if g is None or not (near('u') and near(g)):
            self.together = None
            return
        if self.together is None:
            self.together = (t, g)
        elif t - self.together[0] >= L.DELTA:
            if self.tool_fault.get(g):
                self.failed.add(g)
                self.failed_pub.publish(String(data=json.dumps({'t': round(t, 2), 'by': g})))
                self.get_logger().info(f'the attempt with {g} fails at {t:.1f} s')
            else:
                self.done = {'t': round(t, 2), 'by': g}
                self.done_pub.publish(String(data=json.dumps(self.done)))
                self.get_logger().info(f'the task is done by u and {g} at {t:.1f} s')
            self.attempt, self.together = None, None


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--range', type=float, default=L.R)
    a = ap.parse_args(argv)
    rclpy.init()
    node = World(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
