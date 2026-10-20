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
One robot's map, and what it decides of the corridor.

    mapper_node.py --robot r1 --dir RUN

Builds an occupancy grid from the robot's own laser at the pose its odometry
gives (tools/grid.py), and holds beside it the last grid the other robot sent,
which arrives on /<robot>/peer_grid only through the link's gate. What the
robot has is the two merged: its own cell where it has one, the peer's where
it has none.

publishes  /<robot>/grid      its own grid, ternary, at 1 Hz: what goes over the link
           /<robot>/decision  what the merged grid decides of the corridor,
                              'blocked', 'clear' or '', at 4 Hz
writes     RUN/decision_<robot>.csv  every change of the decision, own and merged
           RUN/maps/<robot>.npz      own and merged grids every 2 s, and at the end

The node is the robot's: it reads nothing of the world but its laser and its
odometry, and nothing of the other robot but what the gate lets through.
"""

import argparse
import collections
import csv
import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('link_grades_demo'), 'tools'))
import grid as G  # noqa: E402

LASER_BACK = 0.032     # the Burger's laser sits behind its base
SNAPSHOT = 2.0
SAVE = 10.0


class Mapper(Node):

    def __init__(self, a):
        super().__init__(f'mapper_{a.robot}', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.logodds = np.zeros((G.H, G.W))
        self.peer = None
        self.odom = collections.deque(maxlen=200)
        self.decided = {'own': '', 'merged': ''}
        self.snaps = {'t': [], 'own': [], 'merged': []}
        self.last_snap = -1.0
        self.last_save = -1.0
        os.makedirs(os.path.join(a.dir, 'maps'), exist_ok=True)
        self.log = csv.writer(open(os.path.join(a.dir, f'decision_{a.robot}.csv'), 'w', newline='', buffering=1))
        self.log.writerow(['t_sim', 'map', 'decision'])
        r = a.robot
        self.create_subscription(Odometry, f'/{r}/odom', self.on_odom, 50)
        self.create_subscription(LaserScan, f'/{r}/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, f'/{r}/peer_grid', self.on_peer, 2)
        self.grid_pub = self.create_publisher(OccupancyGrid, f'/{r}/grid', 2)
        self.decision_pub = self.create_publisher(String, f'/{r}/decision', 10)
        self.create_timer(1.0, self.publish_grid)
        self.create_timer(0.25, self.publish_decision)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))

    def pose_at(self, t):
        if not self.odom:
            return None
        best = min(self.odom, key=lambda p: abs(p[0] - t))
        return best if abs(best[0] - t) < 0.15 else None

    def on_scan(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = self.pose_at(t)
        if p is None:
            return
        _, x, y, yaw = p
        ranges = np.array(m.ranges, dtype=float)
        ranges[(ranges < m.range_min)] = np.nan
        angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
        G.integrate(self.logodds, x - LASER_BACK * math.cos(yaw), y - LASER_BACK * math.sin(yaw), yaw, angles,
                    ranges, m.range_max)
        self.snapshot()

    def on_peer(self, m):
        if m.info.width == G.W and m.info.height == G.H:
            self.peer = np.array(m.data, dtype=np.int8).reshape(G.H, G.W)

    def grids(self):
        own = G.ternary(self.logodds)
        return own, G.merge(own, self.peer)

    def publish_grid(self):
        own, _ = self.grids()
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = G.RES
        m.info.width, m.info.height = G.W, G.H
        m.info.origin.position.x, m.info.origin.position.y = G.ORIGIN
        m.info.origin.orientation.w = 1.0
        m.data = own.ravel().tolist()
        self.grid_pub.publish(m)

    def publish_decision(self):
        own, merged = self.grids()
        t = self.sim()
        for k, g in (('own', own), ('merged', merged)):
            d = G.decide(g)
            if d != self.decided[k]:
                self.decided[k] = d
                self.log.writerow([f'{t:.2f}', k, d])
        self.decision_pub.publish(String(data=self.decided['merged']))

    def snapshot(self, force=False):
        t = self.sim()
        if not force and t - self.last_snap < SNAPSHOT:
            return
        self.last_snap = t
        own, merged = self.grids()
        self.snaps['t'].append(t)
        self.snaps['own'].append(own)
        self.snaps['merged'].append(merged)
        if force or t - self.last_save >= SAVE:
            self.last_save = t
            self.save()

    def save(self):
        path = os.path.join(self.a.dir, 'maps', f'{self.a.robot}.npz')
        np.savez_compressed(path + '.tmp.npz', t=np.array(self.snaps['t']), own=np.array(self.snaps['own']),
                            merged=np.array(self.snaps['merged']), origin=np.array(G.ORIGIN), res=G.RES)
        os.replace(path + '.tmp.npz', path)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    ap.add_argument('--dir', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Mapper(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.snapshot(force=True)


if __name__ == '__main__':
    main()
