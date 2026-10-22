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
The aerial vehicle of the belief and empathy demonstration.

    uav_node.py --dir RUN --formulation epistemic

What it does by itself: from the start, it covers the middle strip along its
route at layout.V_UAV (or holds its place in the flock at /flock/s), and when
the task is found (/task/found, by tools' task_node) by itself it stops and
hovers there.

What it does when asked, on /u/request, answering on /u/reply:

  fly_p1, back_p2, back_p3  [u g]  fly to where particle b of g will be, by the
          shortest intercept from where it is, and follow the particle; it
          answers once it is over it;
  hail_p1, hail_p2, hail_p3  [u g]  follow particle b of g and call g on the
          radio once a second for layout.HAIL_WINDOW seconds, the task's site
          in the call. An answer from g is the event e-met-b, silence e-absent-b;
  cross_west, cross_east  [u]  the turn to the other half of the area, in the
          model a step of its own; the flight is the next action's, so it
          answers at once;
  fly_site  [u]  fly to the task's site and hover over it;
  perform  [u g]  hover over the site until the task is done (/task/done); in
          the variant with latent tool faults, attempt it with g, announced on
          /task/attempt, and answer the event: e-done, or e-fail when the
          attempt fails (/task/failed);
  tell  [u g to]  say to g on the radio which way it is assigned (site, base
          or cover), and answer when g acknowledges it.

Everything it heard and every answer it got go to /events.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_empathy_demo'), 'tools'))
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

GAIN = 1.2
OVER = 0.3            # m, over a point
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Uav(Node):

    def __init__(self, a):
        super().__init__('uav_u', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.mode = 'wait'
        self.pose = None
        self.site = None
        self.found_by = None
        self.done = False
        self.done_by = None
        self.failed = set()
        world = json.load(open(os.path.join(a.dir, 'world.json')))
        self.latent = world.get('variant') == 'latent'
        self.flock_s = 0.0
        self.target = None            # a fixed point to fly to and hover over
        self.follow = None            # (g, b, intercept point, intercept time)
        self.request, self.answered = None, False
        self.hail = None              # (g, b, started, last call)
        self.heard = {}
        self.cmd = self.create_publisher(Twist, '/u/cmd_vel', 10)
        self.reply = self.create_publisher(String, '/u/reply', 10)
        self.radio = self.create_publisher(String, '/radio/send', 50)
        self.events = self.create_publisher(String, '/events', 50)
        self.state = self.create_publisher(String, '/u/state', LATCHED)
        self.create_subscription(Odometry, '/u/odom', self.on_odom, 10)
        self.create_subscription(String, '/u/inbox', self.on_inbox, 50)
        self.create_subscription(String, '/u/request', self.on_request, 10)
        self.create_subscription(String, '/task/found', self.on_found, LATCHED)
        self.create_subscription(String, '/task/done', self.on_done, LATCHED)
        self.create_subscription(String, '/task/failed', lambda m: self.failed.add(json.loads(m.data)['by']),
                                 LATCHED)
        self.attempt_pub = self.create_publisher(String, '/task/attempt', 10)
        self.create_subscription(Float64, '/flock/s', lambda m: setattr(self, 'flock_s', m.data), 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, lambda: self.state.publish(String(data=json.dumps(
            {'robot': 'u', 'mode': self.mode, 'site': self.site, 'follow': self.follow[:2] if self.follow else None}))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel(self):
        return self.now() - W.START_AT

    def event(self, kind, **kw):
        self.events.publish(String(data=json.dumps(dict(kw, robot='u', kind=kind, t=round(self.rel(), 2)))))

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_found(self, m):
        f = json.loads(m.data)
        self.site, self.found_by = tuple(f['site']), f['by']
        if self.mode == 'cover':
            self.mode, self.target = 'hover', self.pose[:2]
            self.get_logger().info(f'u: found the task at {self.site}; hovering')

    def on_done(self, m):
        self.done = True
        self.done_by = json.loads(m.data)['by']

    def on_inbox(self, m):
        msg = json.loads(m.data)
        self.heard.setdefault(msg['from'], []).append((self.rel(), msg['body']))
        self.event('heard', frm=msg['from'], body=msg['body'])

    def answer(self, ok, outcome='', why=''):
        if self.request is not None and not self.answered:
            self.last_answer = String(data=json.dumps({'id': self.request['id'], 'ok': ok, 'outcome': outcome,
                                                       'why': why}))
            self.reply.publish(self.last_answer)
            self.answered = True

    def on_request(self, m):
        req = json.loads(m.data)
        if self.request is not None and req['id'] == self.request['id']:
            # asked again: the answer, if there is one, may have been missed
            if self.answered:
                self.reply.publish(self.last_answer)
            return
        act, args = req['action'], req.get('args', [])
        self.request, self.answered = req, False
        self.event('request', action=act, args=args)
        if act in ('fly_p1', 'back_p2', 'back_p3'):
            self.start_follow(args[1], int(act[-1]))
        elif act.startswith('hail_p'):
            g, b = args[1], int(act[-1])
            if self.follow is None or self.follow[:2] != (g, b):
                self.start_follow(g, b)
            self.hail = (g, b, None, -1e9)
            self.heard.pop(g, None)
        elif act in ('cross_west', 'cross_east'):
            self.answer(True, why='turned')
        elif act in ('fly_site', 'perform'):
            if self.site is None:
                self.answer(False, why='no task known')
                return
            self.follow, self.mode, self.target = None, 'goto', self.site
        elif act == 'goto':
            self.follow, self.mode, self.target = None, 'goto', tuple(args[1:3])
        elif act == 'tell':
            g, to = args[1], args[2]
            self.heard.pop(g, None)
            self.radio.publish(String(data=json.dumps({'from': 'u', 'to': g, 'body': {
                'kind': 'assign', 'to': to, 'site': self.site}})))
            self.tell = (g, self.now())
            self.mode = self.mode if self.mode != 'cover' else 'hover'
        else:
            self.answer(False, why=f'no action {act}')

    def start_follow(self, g, b):
        point, when = L.intercept(self.pose[:2], self.rel(), g, b)
        self.follow, self.mode = (g, b, point, when), 'follow'
        self.event('follow', g=g, b=b, intercept=[round(v, 3) for v in point], at=round(when, 2))

    # ──────────────────────────────────────────────────────────── control ──

    def fly(self, target, feed=(0.0, 0.0)):
        x, y, yaw = self.pose
        vx = feed[0] + GAIN * (target[0] - x)
        vy = feed[1] + GAIN * (target[1] - y)
        n = math.hypot(vx, vy)
        if n > L.V_UAV:
            vx, vy = vx * L.V_UAV / n, vy * L.V_UAV / n
        out = Twist()
        # planar_move takes the velocity in the body frame
        out.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        out.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        if n > 0.15:
            err = math.atan2(math.sin(math.atan2(vy, vx) - yaw), math.cos(math.atan2(vy, vx) - yaw))
            out.angular.z = max(-1.0, min(1.0, 1.5 * err))
        self.cmd.publish(out)

    def tick(self):
        if self.pose is None:
            return
        t = self.rel()
        if self.mode == 'wait':
            if t < 0:
                return
            self.mode = 'flock' if self.a.formulation == 'flock' else 'cover'
        if self.mode == 'cover':
            self.fly(L.at(L.ROUTE['u'], L.V_UAV * t)[0])
        elif self.mode == 'flock':
            self.fly(L.flock_point('u', self.flock_s))
        elif self.mode in ('hover', 'goto'):
            self.fly(self.target)
            if self.mode == 'goto' and math.dist(self.pose[:2], self.target) < OVER and self.request:
                act = self.request['action']
                if act == 'perform' and self.latent:
                    g = self.request['args'][1]
                    self.attempt_pub.publish(String(data=g))
                    if self.done and self.done_by == g:
                        self.answer(True, outcome='e-done', why='done')
                    elif g in self.failed:
                        self.answer(True, outcome='e-fail', why='failed')
                elif act == 'perform':
                    if self.done:
                        self.answer(True, why='done')
                else:
                    self.answer(True, why='over the point')
        elif self.mode == 'follow':
            g, b, point, when = self.follow
            here = L.particle(g, b, t)
            if t < when:
                self.fly(point)
            else:
                ahead = L.particle(g, b, t + 1.0)
                self.fly(here, feed=(ahead[0] - here[0], ahead[1] - here[1]))
            over = math.dist(self.pose[:2], here) < OVER
            act = self.request['action'] if self.request else ''
            if over and act in ('fly_p1', 'back_p2', 'back_p3'):
                self.answer(True, why='over the particle')
            if self.hail is not None and act.startswith('hail_p'):
                self.hailing(over)
        if self.request and self.request['action'] == 'tell' and not self.answered:
            g, sent = self.tell
            acks = [b for _, b in self.heard.get(g, []) if b.get('kind') == 'ack']
            if acks:
                self.answer(True, why='acknowledged')
            elif self.now() - sent > 3.0:
                self.answer(False, why='no acknowledgement')

    def hailing(self, over):
        g, b, started, last = self.hail
        now = self.now()
        if started is None:
            if not over:
                return
            started = now
        if now - last >= 1.0:
            self.radio.publish(String(data=json.dumps({'from': 'u', 'to': g, 'body': {
                'kind': 'hail', 'b': b, 'site': self.site}})))
            last = now
        self.hail = (g, b, started, last)
        answers = [body for _, body in self.heard.get(g, []) if body.get('kind') == 'answer']
        if answers:
            got = answers[-1]
            if got['particle'] != b:
                self.event('model_violation', g=g, b=b, particle=got['particle'])
            self.event('met', g=g, b=b, particle=got['particle'], faults=got['faults'])
            self.hail = None
            self.answer(True, outcome=f'e-met-{b}', why=f"answered from particle {got['particle']}")
        elif now - started >= L.HAIL_WINDOW:
            self.event('absent', g=g, b=b)
            self.hail = None
            self.answer(True, outcome=f'e-absent-{b}', why='no answer')


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Uav(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
