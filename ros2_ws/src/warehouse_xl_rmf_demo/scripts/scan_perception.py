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
Turns a laser scan into the observation the sensing action reports.

This is the link the demonstration was missing. The epistemic machinery was
always real -- the product update, the branch, the model checking -- but the
value it consumed came from `default_outcome` in the task map, so the policy
branched on a constant. What was uncertain in the domain was not uncertain in
the simulation.

The detector is the simplest thing that can work, because the claim is about
where the observation comes from and not about perception. The site is an
aisle: an eighteen-metre rack 1.39 m to the west, clutter 0.89 m to the east,
open floor to the north. A pallet placed beside the waypoint is nearer than
either, so the nearest return in the scan decides. Below the threshold there is
something present that is not the building; above it there is not.

Two worlds differ by that pallet and by nothing else. The navigation graph is
identical for both, and neither the planner nor the policy is told which world
it is in. The branch has to be decided by what the robot measures.

    scan_perception.py --ros-args -p robot:=r1 -p site_x:=-3.35 -p site_y:=2.0
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

from rmf_fleet_msgs.msg import FleetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanPerception(Node):
    def __init__(self):
        super().__init__('scan_perception')

        self.declare_parameter('robot', 'r1')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('observation_topic', '/eplansys/observation')
        self.declare_parameter('site_x', -3.35)
        self.declare_parameter('site_y', 2.00)
        # How near the robot must be for a reading to be a reading of the site.
        #
        # Tight, and it has to be. At 1.20 m the detector fired while the robot
        # was still a metre short, and from there the object beside the site
        # reads about 1.0 m instead of 0.34 m -- so it reported "clean" in both
        # worlds and the demonstration proved nothing. Measured from the site
        # itself the object is at 0.34 m and the aisle clutter at 0.91 m, which
        # is the separation the threshold below relies on.
        self.declare_parameter('site_radius', 0.40)
        # Consecutive checks inside the radius before a reading is taken, so
        # the observation is of a robot that has arrived and not one passing
        # through.
        self.declare_parameter('settle', 3)
        # Measured from the site in both worlds. Clean: the nearest structure
        # is aisle clutter 1.80 m to the east, whose near face reads 0.88 m.
        # Dirty: the pallet reads 0.32 m. The threshold sits between the two
        # with about 0.18 m of margin either side.
        self.declare_parameter('threshold', 0.70)
        self.declare_parameter('present_outcome', 'e-scan-dirty')
        self.declare_parameter('absent_outcome', 'e-scan-clean')

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.robot = g('robot')
        self.site = (g('site_x'), g('site_y'))
        self.radius = g('site_radius')
        self.threshold = g('threshold')
        self.present = g('present_outcome')
        self.absent = g('absent_outcome')

        # Latched. The bridge may subscribe after the reading was taken, and an
        # observation nobody could hear is not an observation.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, g('observation_topic'), latched)

        self.settle = int(g('settle'))
        self.nearest = math.inf
        self.inside = 0
        # Distance to the site at the last fleet report, carried only so the
        # log line can say where the robot was standing when it looked.
        self.distance = math.inf
        self.reported = None

        self.create_subscription(
            LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(FleetState, '/fleet_states', self.on_fleet, 10)
        self.create_timer(0.5, self.decide)

        self.get_logger().info(
            f'watching {g("scan_topic")} for {self.robot}; site {self.site} '
            f'r={self.radius}; threshold {self.threshold} m')

    def on_scan(self, msg):
        good = [r for r in msg.ranges
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.nearest = min(good) if good else math.inf

    def on_fleet(self, msg):
        for r in msg.robots:
            if r.name == self.robot:
                d = math.dist((r.location.x, r.location.y), self.site)
                self.inside = self.inside + 1 if d <= self.radius else 0
                self.distance = d
                return

    def decide(self):
        """Report once the robot is at the site and the laser has spoken.

        Reported once per arrival: the observation is of a place at a time, and
        republishing it every half second would say the robot kept looking.
        """
        if self.inside < self.settle or not math.isfinite(self.nearest):
            if self.inside == 0:
                self.reported = None
            return
        if self.reported is not None:
            return

        outcome = self.present if self.nearest < self.threshold else self.absent
        self.reported = outcome
        self.pub.publish(String(data=outcome))
        self.get_logger().info(
            f'at the site ({self.distance:.2f} m from it), '
            f'nearest return {self.nearest:.2f} m '
            f'({"<" if self.nearest < self.threshold else ">="} '
            f'{self.threshold:.2f}) -> {outcome}')


def main():
    rclpy.init()
    node = ScanPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
