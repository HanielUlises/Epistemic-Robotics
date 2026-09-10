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
Put two SLAM maps on a common grid so the reconciliation can fuse them.

Section 5.9 refuses two maps whose discretisation does not coincide, and is
right to: a wrong registration produces a merged map that is confidently
wrong. The refusal was never exercised by the unit tests, because a test that
builds both grids builds them the same shape. Two maps that two robots
actually built are never the same shape -- each grows as its own robot
discovers floor -- and the first real pair this reconciliation was ever handed
was refused with `grids differ in size: 118x128 and 134x86`.

The refusal is not the defect and is not weakened here. What was missing is the
step before it. This node places both maps on the union of their extents at
their common resolution, which is a translation by whole cells and not a
registration: no interpolation, no rotation, no scaling, and a cell keeps the
value it had. Maps at different resolutions are still refused, because putting
those on one grid would be the approximation 5.9 declines to make.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

UNKNOWN = -1


def latched(depth=4):
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class GridAlign(Node):

    def __init__(self):
        super().__init__('grid_align')
        self.declare_parameter('in_a', '/r2/map')
        self.declare_parameter('in_b', '/r2/partner_map')
        self.declare_parameter('out_a', '/aligned/map_a')
        self.declare_parameter('out_b', '/aligned/map_b')
        self.declare_parameter('resolution_tolerance', 1e-6)

        self.a = None
        self.b = None
        self.pub_a = self.create_publisher(
            OccupancyGrid, self.get_parameter('out_a').value, latched())
        self.pub_b = self.create_publisher(
            OccupancyGrid, self.get_parameter('out_b').value, latched())

        self.create_subscription(
            OccupancyGrid, self.get_parameter('in_a').value,
            self._on_a, latched())
        self.create_subscription(
            OccupancyGrid, self.get_parameter('in_b').value,
            self._on_b, latched())
        self.refused = False

    def _on_a(self, grid):
        self.a = grid
        self._align()

    def _on_b(self, grid):
        self.b = grid
        self._align()

    def _align(self):
        if self.a is None or self.b is None:
            return
        tol = self.get_parameter('resolution_tolerance').value
        if abs(self.a.info.resolution - self.b.info.resolution) > tol:
            if not self.refused:
                self.refused = True
                self.get_logger().error(
                    'resolutions differ ({:.4f} and {:.4f} m/cell): these are '
                    'not alignable by translation and 5.9 declines to '
                    'approximate them'.format(
                        self.a.info.resolution, self.b.info.resolution))
            return

        res = self.a.info.resolution
        # The union of the two extents, in world metres, snapped outward to
        # whole cells of the shared resolution.
        x0 = min(self.a.info.origin.position.x, self.b.info.origin.position.x)
        y0 = min(self.a.info.origin.position.y, self.b.info.origin.position.y)
        x0 = math.floor(x0 / res) * res
        y0 = math.floor(y0 / res) * res
        x1 = max(
            self.a.info.origin.position.x + self.a.info.width * res,
            self.b.info.origin.position.x + self.b.info.width * res)
        y1 = max(
            self.a.info.origin.position.y + self.a.info.height * res,
            self.b.info.origin.position.y + self.b.info.height * res)
        width = int(math.ceil((x1 - x0) / res))
        height = int(math.ceil((y1 - y0) / res))

        self.pub_a.publish(self._placed(self.a, x0, y0, width, height))
        self.pub_b.publish(self._placed(self.b, x0, y0, width, height))
        self.get_logger().info(
            '{}x{} and {}x{} -> {}x{} at {:.3f} m/cell'.format(
                self.a.info.width, self.a.info.height,
                self.b.info.width, self.b.info.height,
                width, height, res),
            once=True)

    def _placed(self, grid, x0, y0, width, height):
        res = grid.info.resolution
        # Whole-cell offset of this map's origin within the union.
        ox = int(round((grid.info.origin.position.x - x0) / res))
        oy = int(round((grid.info.origin.position.y - y0) / res))

        canvas = np.full((height, width), UNKNOWN, dtype=np.int8)
        source = np.asarray(grid.data, dtype=np.int8).reshape(
            grid.info.height, grid.info.width)
        canvas[oy:oy + grid.info.height, ox:ox + grid.info.width] = source

        out = OccupancyGrid()
        out.header = grid.header
        out.info.resolution = res
        out.info.width = width
        out.info.height = height
        out.info.origin.position.x = x0
        out.info.origin.position.y = y0
        out.info.origin.orientation.w = 1.0
        out.data = canvas.reshape(-1).tolist()
        return out


def main():
    rclpy.init()
    node = GridAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
