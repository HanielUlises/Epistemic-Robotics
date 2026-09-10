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
Run the map reconciliation of section 5.9 when the link comes back.

`epistemic_slam` has existed as a library and a node with unit tests over grids
built in memory. This is the first thing that hands it two maps that two
robots actually built, from two lasers, in a simulator, having been apart while
they built them.

It measures the divergence it is reconciling, which is the part a unit test on
an artificial pair cannot: how much of the floor each robot had seen when the
link fell, how much when it returned, and how many cells the fusion moves from
unknown to decided for each of them. A reconciliation that learns nothing is
either a link that was not cut or two robots that stood still, and both look
like success from inside the fusion.

The counts of what was learned come from the fusion's own reply and are not
recomputed here. Recomputing them was the first thing this node did and it was
wrong: the aligner keeps publishing as both maps grow, so by the time a reply
arrives this node may hold a newer pair than the fusion was given, and
differencing two pairs that are not the same pair reported a merged map with
fewer decided cells than one of its inputs.
"""

import json
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from epistemic_msgs.msg import LinkEvent
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger


def latched(depth=8):
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def decided(grid, free_below=25, occupied_above=65):
    """
    Count the cells this map has an opinion about.

    Not the same as cells that are free. A cell at -1 was never observed, and a
    cell between the thresholds was observed and not resolved; section 5.9
    counts both as unknown, and so does this.
    """
    if grid is None:
        return 0
    return sum(
        1 for v in grid.data
        if v >= 0 and (v < free_below or v > occupied_above))


class Reconcile(Node):

    def __init__(self):
        super().__init__('reconcile')
        self.declare_parameter('map_a', '/r2/map')
        self.declare_parameter('map_b', '/r2/partner_map')
        self.declare_parameter('merged', '/map_fusion/merged')
        self.declare_parameter('fuse_service', '/map_fusion/fuse')
        self.declare_parameter('out', '/tmp/reconcile.json')

        self.a = None
        self.b = None
        self.merged = None
        self.at_link_down = {}
        self.at_link_up = {}
        self.written = False

        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_a').value,
            self._on_a, latched())
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_b').value,
            self._on_b, latched())
        self.create_subscription(
            OccupancyGrid, self.get_parameter('merged').value,
            self._on_merged, latched())
        self.create_subscription(
            LinkEvent, '/comm_monitor/link_down', self._on_down, latched())
        self.create_subscription(
            LinkEvent, '/comm_monitor/link_up', self._on_up, latched())

        self.client = self.create_client(
            Trigger, self.get_parameter('fuse_service').value)

    def _on_a(self, grid):
        self.a = grid

    def _on_b(self, grid):
        self.b = grid

    def _on_merged(self, grid):
        self.merged = grid

    def _on_down(self, _event):
        self.at_link_down = {
            'a_decided': decided(self.a),
            'b_decided': decided(self.b),
        }
        self.get_logger().warn(
            'link down with {a_decided} and {b_decided} cells decided'
            .format(**self.at_link_down))

    def _on_up(self, _event):
        self.get_logger().info('link up: reconciling')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'the fusion service never appeared; epistemic_slam is not '
                'running and nothing was reconciled')
            return
        # Snapshot before the call, so that what is reported as the state at
        # reconnection is the state the fusion was asked about.
        self.at_link_up = {
            'a_decided': decided(self.a),
            'b_decided': decided(self.b),
            'cells_total': len(self.a.data) if self.a else 0,
        }
        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self._on_fused)

    def _on_fused(self, future):
        try:
            response = future.result()
        except Exception as error:                       # noqa: BLE001
            self.get_logger().error(f'the fusion call failed: {error}')
            return
        if not response.success:
            # A refusal is a result and is recorded as one. The first real
            # pair this reconciliation was handed was refused for differing in
            # size, and a run that swallowed that would have reported nothing.
            self.get_logger().error(f'the fusion refused: {response.message}')
            self._write({'refused': response.message})
            return

        self._write(self._counts(response.message))

    @staticmethod
    def _counts(message):
        """
        Parse the fusion's own reply.

        It reads `a learned N cells, b learned M, K in conflict`.
        """
        numbers = [int(n) for n in re.findall(r'\d+', message)]
        if len(numbers) != 3:
            return {'reply': message}
        return {
            'a_learned': numbers[0],
            'b_learned': numbers[1],
            'conflicts': numbers[2],
            'reply': message,
        }

    def _write(self, outcome):
        if self.written:
            return
        self.written = True
        total = self.at_link_up.get('cells_total', 0)
        a = self.at_link_up.get('a_decided', 0)
        b = self.at_link_up.get('b_decided', 0)
        record = {
            'at_link_down': self.at_link_down,
            'at_link_up': self.at_link_up,
            'coverage_a': round(a / total, 4) if total else 0.0,
            'coverage_b': round(b / total, 4) if total else 0.0,
        }
        record.update(outcome)
        path = self.get_parameter('out').value
        with open(path, 'w') as handle:
            json.dump(record, handle, indent=2)
        if 'refused' in outcome:
            self.get_logger().error(f'recorded a refusal in {path}')
        else:
            self.get_logger().info(
                'reconciled: {} decided for the holder and {} for the partner '
                'of {} cells; {}'.format(a, b, total, outcome.get('reply', '')))
            self.get_logger().info(f'wrote {path}')


def main():
    rclpy.init()
    node = Reconcile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
