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
The night porter's rounds, and the fire doors he shuts and opens.

    porter_node.py --dir RUN

From RUN/world.json, the loop he walks at one and whether at two he walks it
back. At ROUND1_AT simulated seconds he leaves the lodge and walks the loop,
north through its first door and south through its second (tools/layout.py),
and each door swings shut behind him once he is PAST metres beyond it. When he
is back in the lodge, "round1" goes on /porter/done: the couriers set out then.

On "round2" on /porter/go (the policy's round action, scripts/wait_node) he
walks at two. If he reopens, he walks the loop the other way round, stopping
short of each shut door while it swings open; if he is held up, he walks to the
day room's door and back instead. Either way "round2" goes on /porter/done when
he is back.

Each door is a kinematic model swung by /gazebo/set_entity_state about its
hinge (layout.leaf_swing). The doors' states go on /doors as JSON whenever one
changes.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_revision_demo'), 'tools'))
import layout as L  # noqa: E402

ROUND1_AT = 3.0       # s of simulated time
PAST = 0.9            # m beyond the leaf before it swings shut behind him
SHORT = 0.75          # m short of a shut leaf, where he waits while it opens
SWING = 1.4           # s a swing takes
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Porter(Node):

    def __init__(self, run):
        super().__init__('porter', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        world = json.load(open(os.path.join(run, 'world.json')))
        self.loop = tuple(world['loop'])
        self.plan = world.get('plan')
        self.state = {d: True for d in L.DOORS}
        self.swings = {}                     # door -> (from s, to s, start time)
        self.pending = []                    # round 1: doors to shut once he is past them, in order
        self.publisher_route = self.create_publisher(String, '/porter/route', LATCHED)
        self.done = self.create_publisher(String, '/porter/done', LATCHED)
        self.doors_pub = self.create_publisher(String, '/doors', LATCHED)
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_subscription(String, '/porter/state', self.on_state, 10)
        self.create_subscription(String, '/porter/done', self.on_done, LATCHED)
        self.create_subscription(String, '/porter/go', self.on_go, LATCHED)
        self.pos = None
        self.phase = 'waiting'
        self.segments = []
        self.create_timer(0.05, self.tick)
        self.publish_doors()

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_doors(self):
        self.doors_pub.publish(String(data=json.dumps({'t': round(self.sim(), 2), 'open': self.state})))

    def send(self, name, places_or_points, speed=L.PORTER_SPEED):
        nodes = L.roadmap()[0]
        pts = [nodes[p] if isinstance(p, str) else p for p in places_or_points]
        self.publisher_route.publish(String(data=f'{name} {speed} ' + ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)))
        self.get_logger().info(f'[porter] {name}: {len(pts)} points')

    # -- the doors -----------------------------------------------------------------------

    def swing(self, d, to_open):
        s0 = 0.0 if self.state[d] is False else 1.0
        self.swings[d] = (s0, 1.0 if to_open else 0.0, self.sim())
        self.state[d] = to_open
        self.publish_doors()
        self.get_logger().info(f'[porter] door {d} swings {"open" if to_open else "shut"}')

    def place_leaf(self, d, s):
        cx, cy, yaw = L.leaf_swing(d, s)
        p = Pose()
        p.position.x, p.position.y, p.position.z = cx, cy, 0.0
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req = SetEntityState.Request()
        req.state = EntityState(name=f'door_{d}', pose=p, reference_frame='world')
        self.set_state.call_async(req)

    # -- the rounds ----------------------------------------------------------------------

    def on_state(self, m):
        parts = m.data.split()
        if len(parts) >= 3:
            self.pos = (float(parts[1]), float(parts[2]))

    def on_done(self, m):
        name = m.data
        if self.phase == 'round2' and name.startswith('round2-'):
            k = int(name.split('-')[1])
            if k + 1 < len(self.segments):
                d = self.segments[k][1]
                if d is not None:
                    self.swing(d, True)
                    self.next_segment_at = (self.sim() + SWING + 0.3, k + 1)
                else:
                    self.send(f'round2-{k + 1}', self.segments[k + 1][0])
            else:
                self.phase = 'idle'
                self.done.publish(String(data='round2'))
                self.get_logger().info('[porter] round 2 done')

    def on_go(self, m):
        if m.data == 'round2' and self.phase == 'idle':
            self.start_round2()

    def start_round1(self):
        self.phase = 'round1'
        first, second = self.loop
        self.pending = [first, second]
        self.send('round1', L.porter_path(self.loop))

    def start_round2(self):
        self.phase = 'round2'
        self.next_segment_at = None
        nodes = L.roadmap()[0]
        if self.plan == 'reopen':
            # the loop the other way round, cut short of each shut leaf, where he
            # waits while it swings open
            path = L.porter_path(self.loop, back=True)
            pts = [nodes[p] for p in path]
            segments, i0 = [], 0
            for d in self.loop[::-1]:
                a, b = path.index('j' + d), path.index('k' + d)
                start, end = min(a, b), max(a, b)
                direction = 1.0 if pts[end][0] > pts[start][0] else -1.0
                stop = (L.LEAF_X[d] - direction * SHORT, L.BEND_Y)
                segments.append((pts[i0:start + 1] + [stop], d))
                i0 = start + 1
            segments.append((pts[i0:], None))
        else:
            # held up: to the day room's door and back
            away = [nodes['ld'], nodes['sl'], nodes['vb'], nodes['va'], (-11.5, -1.6), (-11.5, -2.8)]
            segments = [(away + away[::-1][1:], None)]
        self.segments = segments
        self.send('round2-0', self.segments[0][0])

    def tick(self):
        t = self.sim()
        if self.phase == 'waiting' and t >= ROUND1_AT:
            self.start_round1()
        for d, (s0, s1, t0) in list(self.swings.items()):
            u = min(1.0, (t - t0) / SWING)
            self.place_leaf(d, s0 + (s1 - s0) * (0.5 - 0.5 * math.cos(math.pi * u)))
            if u >= 1.0:
                del self.swings[d]
        if self.phase == 'round1' and self.pending and self.pos is not None:
            d = self.pending[0]
            first = d == self.loop[0]
            # north through the first door: the jog from j (west) to k (east); south through the second: k to j
            x, y = self.pos
            in_jog = L.JOG_Y[0] - 0.3 <= y <= L.JOG_Y[1] + 0.8
            past = (x > L.LEAF_X[d] + PAST) if first else (x < L.LEAF_X[d] - PAST)
            if in_jog and past and abs(x - L.LEAF_X[d]) < 2.6:
                self.swing(d, False)
                self.pending.pop(0)
        if self.phase == 'round2' and getattr(self, 'next_segment_at', None) and t >= self.next_segment_at[0]:
            k = self.next_segment_at[1]
            self.next_segment_at = None
            self.send(f'round2-{k}', self.segments[k][0])


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Porter(a.dir)

    def round1_done(m):
        if m.data == 'round1' and node.phase == 'round1':
            node.phase = 'idle'
            node.get_logger().info('[porter] round 1 done')
    node.create_subscription(String, '/porter/done', round1_done, LATCHED)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
