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
One robot's SLAM map, and what it decides of the two doorways.

    doorway_node.py --robot s1 --dir RUN

Reads the occupancy grid this robot's own slam_toolbox builds, on /<robot>/map,
resamples it into the grid of tools/grid.py (the map frame is the robot's
odometry frame at its start, which is the world frame), classifies each cell
with slam_toolbox's own thresholds -- free below 25, occupied above 65, unknown
between and where never seen -- and decides each doorway by tools/grid.py's
rule.

publishes  /<robot>/decision_w, /<robot>/decision_e   'blocked', 'clear' or '', at 4 Hz
writes     RUN/decision_<robot>.csv  every change of a decision
           RUN/maps/<robot>.npz      the ternary grid every 2 s, and at the end

The node is the robot's: it reads nothing of the world but that robot's map.
"""

import argparse
import csv
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('collapsed_wing_demo'), 'tools'))
import grid as G  # noqa: E402

FREE_BELOW, OCC_ABOVE = 25, 65
SNAPSHOT = 2.0
SAVE = 10.0
DOORS = ('w', 'e')


class Doorways(Node):

    def __init__(self, a):
        super().__init__(f'doorways_{a.robot}', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.grid = np.full((G.H, G.W), G.UNKNOWN, dtype=np.int8)
        self.decided = {d: '' for d in DOORS}
        self.snaps = {'t': [], 'grid': []}
        self.last_snap = -1.0
        self.last_save = -1.0
        jj, ii = np.mgrid[0:G.H, 0:G.W]
        self.cx = G.ORIGIN[0] + (ii + 0.5) * G.RES
        self.cy = G.ORIGIN[1] + (jj + 0.5) * G.RES
        os.makedirs(os.path.join(a.dir, 'maps'), exist_ok=True)
        self.log = csv.writer(open(os.path.join(a.dir, f'decision_{a.robot}.csv'), 'w', newline='', buffering=1))
        self.log.writerow(['t_sim', 'door', 'decision'])
        r = a.robot
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, f'/{r}/map', self.on_map, qos)
        self.pubs = {d: self.create_publisher(String, f'/{r}/decision_{d}', 10) for d in DOORS}
        self.create_timer(0.25, self.publish)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, m):
        res, w, h = m.info.resolution, m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        raw = np.array(m.data, dtype=np.int16).reshape(h, w)
        i = np.floor((self.cx - ox) / res).astype(int)
        j = np.floor((self.cy - oy) / res).astype(int)
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        v = np.full(self.cx.shape, -1, dtype=np.int16)
        v[inside] = raw[j[inside], i[inside]]
        g = np.full(v.shape, G.UNKNOWN, dtype=np.int8)
        g[(v >= 0) & (v < FREE_BELOW)] = G.KNOWN_FREE
        g[v > OCC_ABOVE] = G.KNOWN_OCC
        self.grid = g
        self.snapshot()

    def publish(self):
        t = self.sim()
        for d in DOORS:
            dec = G.decide(self.grid, d)
            if dec != self.decided[d]:
                self.decided[d] = dec
                self.log.writerow([f'{t:.2f}', d, dec])
            self.pubs[d].publish(String(data=self.decided[d]))

    def snapshot(self, force=False):
        t = self.sim()
        if not force and t - self.last_snap < SNAPSHOT:
            return
        self.last_snap = t
        self.snaps['t'].append(t)
        self.snaps['grid'].append(self.grid.copy())
        if force or t - self.last_save >= SAVE:
            self.last_save = t
            self.save()

    def save(self):
        if not self.snaps['t']:
            return
        path = os.path.join(self.a.dir, 'maps', f'{self.a.robot}.npz')
        np.savez_compressed(path + '.tmp.npz', t=np.array(self.snaps['t']), grid=np.array(self.snaps['grid']),
                            origin=np.array(G.ORIGIN), res=G.RES)
        os.replace(path + '.tmp.npz', path)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    ap.add_argument('--dir', required=True)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Doorways(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.snapshot(force=True)


if __name__ == '__main__':
    main()
