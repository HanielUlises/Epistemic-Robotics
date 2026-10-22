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
One ground vehicle of the belief and empathy demonstration.

    ugv_node.py --robot g1 --dir RUN --formulation epistemic

What the vehicle does by itself:

  cover   from the start, along its coverage route, holding the particle its
          faults leave it on: particle 1 with none, 2 with one, 3 with two. A
          fault, at the times RUN/world.json gives, lowers its top speed to
          layout.CAP[k], and the vehicle goes to its next particle, back along
          the route if the particle is behind it, and holds it: the paper's
          empathy, done by the vehicle that has the fault;
  flock   under the flock formulation, its place in the flock at the
          reference /flock/s; with two faults it leaves the flock and holds;
  answer  a hail heard on /g1/inbox is answered on the radio with the particle
          it holds, and the task's site it carries is kept: the vehicle now
          knows of the task. An assignment heard on the radio is kept too.

What it does when asked, on /g1/request, answering on /g1/reply:

  go_site  drive the roadmap to the site of a task it has heard of, at
           layout.DRIVE[k]; it FAILS when it has heard of none. It answers on
           arrival; in the variant with latent tool faults it answers when it
           sets out, and the attempt at the site waits for it;
  go_base  drive the roadmap to the base.

In that variant a drive fault slows the vehicle and costs it nothing else; a
tool fault is latent, and the vehicle does not know of it.

Its state, on /g1/state at 2 Hz: faults, particle, mode, whether it knows of a
task. Faults and hails heard go to /events for the recorder.
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_empathy_demo'), 'tools'))
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

LOOKAHEAD = 0.35
TURN = 0.8            # rad of heading error beyond which the vehicle turns on the spot
GAIN = 0.6            # 1/s, speed per metre of gap to the point held
ARRIVED = 0.25
STOP = 0.22           # m, a return this close ahead stops the vehicle
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Ugv(Node):

    def __init__(self, a):
        super().__init__(f'ugv_{a.robot}', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.r, self.a = a.robot, a
        world = json.load(open(os.path.join(a.dir, 'world.json')))
        self.latent = world.get('variant') == 'latent'
        self.fault_at = [W.START_AT + t for t in world['fault_times'].get(self.r, [])]
        self.faults = 0
        self.mode = 'wait'
        self.pose = None
        self.front = float('inf')
        self.s = 0.0                      # arc length of the vehicle along the path it holds
        self.site = None                  # the site of a task the vehicle has heard of
        self.assigned = None
        self.path, self.request = None, None
        self.answers = {}
        self.flock_s = 0.0
        self.cmd = self.create_publisher(Twist, f'/{self.r}/cmd_vel', 10)
        self.reply = self.create_publisher(String, f'/{self.r}/reply', 10)
        self.state = self.create_publisher(String, f'/{self.r}/state', LATCHED)
        self.radio = self.create_publisher(String, '/radio/send', 50)
        self.events = self.create_publisher(String, '/events', 50)
        self.create_subscription(Odometry, f'/{self.r}/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, f'/{self.r}/scan', self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(String, f'/{self.r}/inbox', self.on_inbox, 50)
        self.create_subscription(String, f'/{self.r}/request', self.on_request, 10)
        self.create_subscription(Float64, '/flock/s', lambda m: setattr(self, 'flock_s', m.data), 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_state)

    # ───────────────────────────────────────────────────────────── inputs ──

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_scan(self, m):
        best = float('inf')
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if math.isfinite(r) and r > m.range_min and abs(wrap(a)) < 0.45:
                best = min(best, r)
        self.front = best

    def event(self, kind, **kw):
        self.events.publish(String(data=json.dumps(dict(kw, robot=self.r, kind=kind,
                                                        t=round(self.now() - W.START_AT, 2)))))

    def on_inbox(self, m):
        msg = json.loads(m.data)
        body = msg['body']
        if body.get('site') is not None and self.site is None:
            self.site = tuple(body['site'])
        if body['kind'] == 'hail':
            self.event('heard', frm=msg['from'], b=body.get('b'), particle=self.faults + 1)
            self.radio.publish(String(data=json.dumps({
                'from': self.r, 'to': msg['from'],
                'body': {'kind': 'answer', 'b': body.get('b'), 'particle': self.faults + 1, 'faults': self.faults,
                         'capable': self.faults < 2, 'pose': [round(v, 3) for v in self.pose[:2]]}})))
        elif body['kind'] == 'assign':
            self.assigned = body['to']
            self.event('assigned', to=body['to'], frm=msg['from'])
            self.radio.publish(String(data=json.dumps({'from': self.r, 'to': msg['from'],
                                                       'body': {'kind': 'ack', 'to': body['to']}})))

    def on_request(self, m):
        req = json.loads(m.data)
        if req['id'] in self.answers:
            self.reply.publish(self.answers[req['id']])
            return
        if self.request is not None and req['id'] == self.request['id']:
            return
        act = req['action']
        if act == 'go_site':
            if self.site is None:
                self.answer(req, False, why='no task known')
                return
            goal = self.site
        elif act == 'go_base':
            goal = L.BASE
        else:
            self.answer(req, False, why=f'no action {act}')
            return
        self.path = L.ground_path(self.pose[:2], goal)
        self.s = 0.0
        self.mode, self.request = 'drive', req
        self.event('drive', to=act, path=self.path)
        if self.latent and act == 'go_site':
            self.answer(req, True, why='set out')
        self.get_logger().info(f'{self.r}: {act} over {len(self.path) - 1} legs, {L.length(self.path):.1f} m')

    def answer(self, req, ok, outcome='', why=''):
        self.answers[req['id']] = String(data=json.dumps({'id': req['id'], 'ok': ok, 'outcome': outcome,
                                                          'why': why}))
        self.reply.publish(self.answers[req['id']])

    def publish_state(self):
        self.state.publish(String(data=json.dumps({
            'robot': self.r, 'faults': self.faults, 'particle': self.faults + 1,
            'capable': True if self.latent else self.faults < 2,
            'mode': self.mode, 'knows_task': self.site is not None, 'assigned': self.assigned})))

    # ──────────────────────────────────────────────────────────── control ──

    def steer(self, target, speed):
        """Pure pursuit of a point at a speed, turning on the spot when facing
        away from it, and stopping short of anything close ahead."""
        x, y, yaw = self.pose
        err = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        out = Twist()
        if abs(err) > TURN:
            out.angular.z = math.copysign(1.0, err)
        else:
            v = speed
            if self.front < STOP:
                v = 0.0
            out.linear.x = v
            out.angular.z = max(-1.2, min(1.2, 2.2 * err))
        self.cmd.publish(out)

    def hold_arc(self, path, s_target, speed):
        """Hold the point at arc s_target of a path: towards it along the path,
        forwards or back, faster the further it is, at most the current cap."""
        # The vehicle's own arc, found near where it last was, so that a lane
        # beside the one it is on is never taken for it.
        lo, hi = max(0.0, self.s - 1.5), self.s + 1.5
        best, best_d, s = self.s, float('inf'), lo
        while s <= hi:
            d = math.dist(L.at(path, s)[0], self.pose[:2])
            if d < best_d:
                best, best_d = s, d
            s += 0.05
        self.s = best
        gap = s_target - self.s
        cap = L.CAP[min(self.faults, 2)]
        v = min(cap, speed + GAIN * abs(gap))
        if abs(gap) < 0.12 and best_d < 0.2:
            v = min(v, speed)
        if gap >= -0.3:
            carrot = L.at(path, self.s + LOOKAHEAD)[0] if gap > 0.05 else L.at(path, s_target)[0]
        else:
            carrot = L.at(path, self.s - LOOKAHEAD)[0]
        if gap < 0.05 and gap > -0.3 and best_d < 0.2:
            self.cmd.publish(Twist())       # at the point: wait for it to move on
            return
        self.steer(carrot, v)

    def tick(self):
        if self.pose is None:
            return
        t = self.now()
        while self.fault_at and t >= self.fault_at[0]:
            self.fault_at.pop(0)
            self.faults += 1
            self.event('fault', faults=self.faults, particle=self.faults + 1)
            self.get_logger().warn(f'{self.r}: fault {self.faults}; holds particle {self.faults + 1} now')
            if self.mode == 'flock' and self.faults >= 2:
                self.mode = 'left'
                self.cmd.publish(Twist())
                self.event('left_flock')
        if self.mode == 'wait':
            if t < W.START_AT:
                return
            self.mode = 'flock' if self.a.formulation == 'flock' else 'cover'
        if self.mode == 'cover':
            b = min(self.faults, 2) + 1
            self.hold_arc(L.ROUTE[self.r], L.SPEED[b - 1] * (t - W.START_AT), L.SPEED[b - 1])
        elif self.mode == 'flock':
            own = L.FLOCK[self.r]
            self.hold_arc(own, L.flock_arc(self.r, self.flock_s), L.SPEED[0])
        elif self.mode == 'drive':
            end = L.length(self.path)
            if math.dist(self.pose[:2], self.path[-1]) < ARRIVED:
                self.cmd.publish(Twist())
                self.mode = 'at_' + self.request['action'][3:]
                self.event('arrived', at=self.mode)
                if not (self.latent and self.request['action'] == 'go_site'):
                    self.answer(self.request, True, why='arrived')
                return
            s, _ = L.project(self.path, self.pose[:2])
            carrot = L.at(self.path, min(end, s + LOOKAHEAD))[0]
            self.steer(carrot, L.DRIVE[min(self.faults, 2)])
        else:
            self.cmd.publish(Twist())


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Ugv(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
