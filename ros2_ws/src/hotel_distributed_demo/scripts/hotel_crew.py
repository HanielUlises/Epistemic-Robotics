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
Everything in the hotel that is not an action of the policy.

  setup      before planning, the three robots go where the model starts
             them, one after another: the concierge to the lobby, where the
             guest is, the porter to the kitchen and the cleaner to its dock in
             the restaurant. The mission asks for it on /hotel/crew/request and
             is told on /hotel/crew when every robot has arrived.
  the scene  the guest, a figure in the lobby with no robot and no collision,
             and the water in the room the leak is in, which goes when the
             model says the leak is contained. Both are drawn for the film;
             neither is seen by any robot.
  stand-down after the policy, each of the cleaner and the concierge does what
             it knows at the actual world: a robot that knows the leak is
             contained stands down, the cleaner back to its charger and the
             concierge to the desk; one that does not know stays where it is.
             The porter, which contained it, always knows.

Robots are moved by Open-RMF tasks pinned to them, as the bridge moves them,
and an arrival is read off /fleet_states: the robot within 0.4 m of the
waypoint, on its level, with no task.
"""

import json
import math
import os
import uuid

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

try:
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
except ImportError:   # noqa: BLE001  headless without gazebo_msgs
    SpawnEntity = DeleteEntity = None
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest

LATCHED = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
RMF = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

ROBOTS = {
    'concierge': ('tinyRobot', 'tinyBot_1', 0),
    'porter': ('deliveryRobot', 'deliveryBot_1', 2),
    'cleaner': ('cleanerBotA', 'cleanerBotA_1', 1),
}
SETUP = {'concierge': 'lobby', 'porter': 'kitchen', 'cleaner': 'clean_restaurant'}
STAND_DOWN = {'concierge': 'lobby', 'cleaner': 'cleanerbot_charger1'}
WHERE = {'lobby': 'the lobby', 'restaurant': 'the restaurant', 'kitchen': 'the kitchen',
         'clean_restaurant': 'the restaurant', 'cleanerbot_charger1': 'its charger'}
ELEVATION = {'L1': 0.0, 'L2': 8.0, 'L3': 16.0}
GUEST_AT = (17.9, -28.7)

GUEST_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="hotel_guest">
    <static>true</static>
    <link name="body">
      <visual name="torso">
        <pose>0 0 0.55 0 0 0</pose>
        <geometry><cylinder><radius>0.24</radius><length>1.1</length></cylinder></geometry>
        <material><ambient>0.80 0.45 0.10 1</ambient><diffuse>0.85 0.50 0.12 1</diffuse></material>
      </visual>
      <visual name="head">
        <pose>0 0 1.32 0 0 0</pose>
        <geometry><sphere><radius>0.17</radius></sphere></geometry>
        <material><ambient>0.85 0.70 0.55 1</ambient><diffuse>0.90 0.74 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

WATER_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="leak_water">
    <static>true</static>
    <link name="water">
      <visual name="pool">
        <geometry><cylinder><radius>1.1</radius><length>0.02</length></cylinder></geometry>
        <material><ambient>0.05 0.30 0.85 0.8</ambient><diffuse>0.10 0.40 0.95 0.8</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def waypoints(graph_dir):
    """(graph index, name) -> (level, x, y), from the hotel's nav graphs."""
    out = {}
    for g in (0, 1, 2):
        with open(os.path.join(graph_dir, f'{g}.yaml')) as fh:
            data = yaml.safe_load(fh)
        for level, ld in data['levels'].items():
            for v in ld['vertices']:
                name = v[2].get('name')
                if name:
                    out[(g, name)] = (level, float(v[0]), float(v[1]))
    return out


class Crew(Node):

    def __init__(self):
        super().__init__('hotel_crew')
        self.leak = self.declare_parameter('leak', 'L3_room1').value
        self.scene = self.declare_parameter('scene', True).value
        graphs = self.declare_parameter('nav_graphs', '').value
        self.points = waypoints(graphs)
        self.requests = self.create_publisher(ApiRequest, '/task_api_requests', RMF)
        self.reply = self.create_publisher(String, '/hotel/crew', LATCHED)
        self.create_subscription(String, '/hotel/crew/request', self.on_request, 10)
        self.create_subscription(String, '/hotel/knowledge', self.on_knowledge, LATCHED)
        self.create_subscription(String, '/hotel/shown_floor', self.on_shown_floor, LATCHED)
        self.create_subscription(FleetState, '/fleet_states', self.on_fleet, 10)
        self.pose = {}
        self.knowledge = None
        self.pending = {}           # agent -> (waypoint, began, what to say on arrival)
        self.job = None             # 'setup' or 'stand-down'
        self.job_began = None
        self.said = []
        self.queue = []
        self.water = False
        self.water_gone = False
        self.shown = None
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity') if SpawnEntity else None
        self.delete = self.create_client(DeleteEntity, '/delete_entity') if DeleteEntity else None
        self.create_timer(0.5, self.tick)
        self.scene_placed = not self.scene

    # ── Open-RMF ──────────────────────────────────────────────────────────

    def send(self, agent, waypoint):
        fleet, robot, _ = ROBOTS[agent]
        request = {
            'type': 'robot_task_request', 'fleet': fleet, 'robot': robot,
            'request': {'category': 'compose', 'requester': 'hotel_crew',
                        'description': {'category': 'go_to_place', 'phases': [
                            {'activity': {'category': 'go_to_place',
                                          'description': {'waypoint': waypoint}}}]}}}
        self.requests.publish(ApiRequest(json_msg=json.dumps(request),
                                         request_id=f'hotel-crew-{uuid.uuid4().hex[:10]}'))

    def on_fleet(self, msg):
        for r in msg.robots:
            self.pose[r.name] = (r.location.level_name, r.location.x, r.location.y, r.task_id)

    def at(self, agent, waypoint):
        fleet, robot, g = ROBOTS[agent]
        if robot not in self.pose or (g, waypoint) not in self.points:
            return False
        level, x, y, task = self.pose[robot]
        wl, wx, wy = self.points[(g, waypoint)]
        return level == wl and math.hypot(x - wx, y - wy) < 0.4 and not task

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── requests from the mission ─────────────────────────────────────────

    def on_request(self, msg):
        what = msg.data.strip()
        if what == 'setup' and self.job is None:
            self.job, self.job_began = 'setup', self.now()
            # One robot at a time. Sent together, the porter and the cleaner
            # take the same corridor to the restaurant, and in one recording
            # Open-RMF's negotiation between them failed for twenty minutes.
            self.queue = list(SETUP.items())
            self.next_setup()
        elif what == 'stand-down' and self.job is None:
            self.stand_down()

    def next_setup(self):
        while self.queue:
            agent, wp = self.queue.pop(0)
            if self.at(agent, wp):
                self.get_logger().info(f'[crew] {agent} is already at {wp}')
                continue
            self.pending[agent] = (wp, self.now(), f'{agent} in {WHERE[wp]}')
            self.send(agent, wp)
            self.get_logger().info(f'[crew] setup: {agent} to {wp}')
            return

    def stand_down(self):
        self.job, self.job_began = 'stand-down', self.now()
        if self.knowledge is None:
            self.finish('stand-down refused: no knowledge has been published')
            return
        said = []
        for agent, wp in STAND_DOWN.items():
            status = self.knowledge['agents'][agent]['safe']
            if status != 'contained':
                text = (f'{agent} does not know the leak is contained ({status}): '
                        f'it stays where it is')
                self.get_logger().info('[crew] ' + text)
                said.append(text)
                continue
            text = f'{agent} knows the leak is contained: it stands down, to {WHERE[wp]}'
            self.get_logger().info('[crew] ' + text)
            said.append(text)
            if not self.at(agent, wp):
                self.pending[agent] = (wp, self.now(), f'{agent} at {WHERE[wp]}')
                self.send(agent, wp)
        self.said = said
        if not self.pending:
            self.finish('stood down: ' + '; '.join(said))

    def finish(self, text):
        self.reply.publish(String(data=text))
        self.get_logger().info('[crew] ' + text)
        self.job = None

    def tick(self):
        if not self.scene_placed:
            self.place_scene()
        for agent, (wp, began, text) in list(self.pending.items()):
            if self.at(agent, wp):
                del self.pending[agent]
                self.get_logger().info(f'[crew] {text} after {self.now() - began:.0f} s')
            elif self.now() - began > 900.0:
                del self.pending[agent]
                self.get_logger().error(f'[crew] {agent} never reached {wp}')
                self.finish(f'{self.job} failed: {agent} never reached {wp}')
                self.pending.clear()
                return
        if self.job == 'setup' and not self.pending and self.queue:
            self.next_setup()
        if self.job and not self.pending:
            if self.job == 'setup':
                self.finish('ready: the concierge in the lobby, the porter in the kitchen, '
                            'the cleaner in the restaurant')
            elif self.job == 'stand-down':
                self.finish('stood down: ' + '; '.join(self.said))

    # ── the scene ─────────────────────────────────────────────────────────

    def place_scene(self):
        if not self.spawn or not self.spawn.service_is_ready():
            return
        self.scene_placed = True
        x, y = GUEST_AT
        self.spawn_model('hotel_guest', GUEST_SDF, x, y, 0.0)
        self.show_water()

    def spawn_model(self, name, sdf, x, y, z):
        req = SpawnEntity.Request()
        req.name, req.xml = name, sdf
        req.initial_pose.position.x, req.initial_pose.position.y = x, y
        req.initial_pose.position.z = z
        self.spawn.call_async(req)

    def water_level(self):
        return self.leak.split('_')[0]

    def show_water(self):
        """The water is drawn while its floor is the one shown, or no floor is
        named, and while the leak is not contained. The world's toggle_floors
        hides each floor's own models; this one is not among them, and from
        above it would float over a lower floor."""
        if self.water_gone:
            return
        wanted = self.shown is None or self.shown == self.water_level()
        if wanted and not self.water:
            level, x, y = self.points[(2, self.leak)]
            self.spawn_model('leak_water', WATER_SDF, x, y, ELEVATION[level] + 0.05)
            self.water = True
        elif not wanted and self.water:
            self.remove('leak_water')
            self.water = False

    def remove(self, name):
        if self.delete and self.delete.service_is_ready():
            req = DeleteEntity.Request()
            req.name = name
            self.delete.call_async(req)

    def on_shown_floor(self, msg):
        self.shown = msg.data.strip() or None
        if self.scene_placed and self.scene:
            self.show_water()

    def on_knowledge(self, msg):
        try:
            self.knowledge = json.loads(msg.data)
        except ValueError:
            return
        if self.knowledge.get('safe') and not self.water_gone:
            self.water_gone = True
            if self.water:
                self.remove('leak_water')
                self.water = False
            self.get_logger().info(f'[crew] the water in {self.leak} stops: the leak is contained')


def main():
    rclpy.init()
    node = Crew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
