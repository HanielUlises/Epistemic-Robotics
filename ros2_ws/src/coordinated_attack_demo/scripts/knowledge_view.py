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
Draws what the two robots know about the work order, and how deep it goes.

Everything here is computed from the model the epistemic state publishes
after every product update. For the stand the order names, once the branch
has settled which one it is:

  K_i job(s)   for each robot: job(s) holds in every world i considers
               possible from every designated world
  depth        the largest k with E^k job(s), where E is "both robots know".
               A breadth-first walk over the union of the two relations from
               the designated worlds finds the nearest world where job(s)
               fails; if it is L steps away, E^(L-1) holds and E^L does not.
               No such world is common knowledge, C job(s).
  chain        the agents along that walk, which is the chain of "south
               considers ... north considers ..." that keeps C from holding

These are logged as a [knows] line whenever they change, which the video's
captions are timed from, and published on /coordinated_attack/knowledge.

On the floor: the stands with one tile per robot, green when that robot knows
the order names the stand; the depth beside the stand; the beacon, lit or
dark; each robot's viewpoint and its sight line to the beacon; the terminal;
and a radio message in flight, as an arc over the block from sender to
receiver, with what it says.

On the sight and blind floors, where the robots' positions are not
announced, also: beside each viewpoint, whether the other robot knows that
this one stands there, K_j at-view(i), which is logged on the same [knows]
line; the crates in t2 on the blind floor; and, when the two look at each
other through t2, the line between them, green when both lasers read clear.
"""

import json
import math
import os
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

KNOWS = (0.10, 0.62, 0.30)
UNSURE = (0.62, 0.62, 0.62)
LOAD = (0.55, 0.30, 0.12)
BEACON_ON = (0.15, 0.95, 0.40)
BEACON_OFF = (0.20, 0.30, 0.22)
INK = (0.12, 0.12, 0.12)
COMMON = (0.78, 0.06, 0.18)
CRATES = (0.55, 0.40, 0.24)
BLOCKED = (0.85, 0.15, 0.10)
WHERE_UNKNOWN = (0.40, 0.40, 0.40)


def rgba(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def read_pgm(yaml_path):
    """The floor plan as an OccupancyGrid, read the way the robots read it."""
    meta = {}
    with open(yaml_path) as fh:
        for line in fh:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    image = meta['image']
    if not image.startswith('/'):
        image = os.path.join(os.path.dirname(yaml_path), image)
    with open(image, 'rb') as fh:
        data = fh.read()
    parts = data.split(maxsplit=4)
    width, height, maxval = int(parts[1]), int(parts[2]), int(parts[3])
    raster = np.frombuffer(parts[4][:width * height], dtype=np.uint8).reshape(height, width)
    occ = (maxval - raster.astype(float)) / maxval
    grid = np.full(raster.shape, -1, dtype=np.int8)
    grid[occ > float(meta.get('occupied_thresh', 0.65))] = 100
    grid[occ < float(meta.get('free_thresh', 0.196))] = 0
    grid = grid[::-1]
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = float(meta['resolution'])
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg


class Model:
    """The pointed Kripke model, as the epistemic state publishes it."""

    def __init__(self, payload):
        self.worlds = payload.get('worlds', [])
        self.designated = payload.get('designated', [])
        self.labels = {w: set(a) for w, a in payload.get('labels', {}).items()}
        self.relations = payload.get('relations', {})

    def sees(self, agent, world):
        return self.relations.get(agent, {}).get(world, [world])

    def settled(self, atom):
        return bool(self.designated) and all(atom in self.labels.get(w, ())
                                             for w in self.designated)

    def knows(self, agent, atom):
        return all(atom in self.labels.get(v, ())
                   for w in self.designated for v in self.sees(agent, w))

    def depth(self, atom, agents):
        """(k, chain): E^k atom and not E^(k+1); k is None for C."""
        parent = {w: None for w in self.designated}
        todo = deque((w, 0) for w in self.designated)
        while todo:
            u, d = todo.popleft()
            for a in agents:
                for v in self.sees(a, u):
                    if v in parent:
                        continue
                    parent[v] = (u, a)
                    if atom not in self.labels.get(v, ()):
                        chain, x = [], v
                        while parent[x] is not None:
                            x, who = parent[x]
                            chain.append(who)
                        return d, chain[::-1]
                    todo.append((v, d + 1))
        return None, []


class KnowledgeView(Node):

    def __init__(self):
        super().__init__('knowledge_view')
        self.declare_parameter('floorplan', '')
        self.declare_parameter('agents', ['south', 'north'])
        self.declare_parameter('namespaces', ['r1', 'r2'])
        self.declare_parameter('colours', [0.2, 0.45, 0.8, 0.9, 0.5, 0.15])
        self.declare_parameter('stands', ['s1', 's2'])
        self.declare_parameter('load_boxes', [0.0] * 8)
        self.declare_parameter('beacon', [0.0, 0.0])
        self.declare_parameter('viewpoints', [0.0] * 4)
        self.declare_parameter('terminal', [0.0, 0.0])
        self.declare_parameter('has_beacon', True)
        self.declare_parameter('positions', False)
        self.declare_parameter('crates', [0.0])

        self.agents = list(self.get_parameter('agents').value)
        self.namespaces = dict(zip(self.agents, self.get_parameter('namespaces').value))
        flat = list(self.get_parameter('colours').value)
        self.colour = {a: tuple(flat[3 * i:3 * i + 3]) for i, a in enumerate(self.agents)}
        names = list(self.get_parameter('stands').value)
        boxes = list(self.get_parameter('load_boxes').value)
        self.loads = {n: tuple(boxes[4 * i:4 * i + 4]) for i, n in enumerate(names)}
        self.beacon = tuple(self.get_parameter('beacon').value)
        views = list(self.get_parameter('viewpoints').value)
        self.views = {a: (views[2 * i], views[2 * i + 1]) for i, a in enumerate(self.agents)}
        self.terminal = tuple(self.get_parameter('terminal').value)
        self.has_beacon = bool(self.get_parameter('has_beacon').value)
        self.positions = bool(self.get_parameter('positions').value)
        crates = list(self.get_parameter('crates').value)
        self.crates = tuple(crates) if len(crates) == 4 else None
        self.where = {}
        self.sighting = {}

        self.model = None
        self.shape = (0, 0)
        self.poses = {}
        self.radio = {}
        self.radio_until = 0.0
        self.lit = ''
        self.lift = {}
        self.last_line = None
        self.facts = {}

        self.markers = self.create_publisher(MarkerArray, '/coordinated_attack/markers', LATCHED)
        self.table_pub = self.create_publisher(String, '/coordinated_attack/knowledge', LATCHED)
        plan_pub = self.create_publisher(OccupancyGrid, '/coordinated_attack/floorplan', LATCHED)
        path = self.get_parameter('floorplan').value
        if path:
            plan_pub.publish(read_pgm(path))
        self.plan_pub = plan_pub

        self.create_subscription(String, '/epistemic_state/state', self.on_state, HISTORY)
        self.create_subscription(String, '/coordinated_attack/radio', self.on_radio, HISTORY)
        self.create_subscription(String, '/coordinated_attack/beacon', self.on_beacon, LATCHED)
        self.create_subscription(String, '/coordinated_attack/lift', self.on_lift, LATCHED)
        self.create_subscription(String, '/coordinated_attack/sight', self.on_sight, LATCHED)
        for agent, ns in self.namespaces.items():
            self.create_subscription(
                Odometry, f'/{ns}/odom', lambda m, a=agent: self.on_odom(a, m), 10)

        self.create_timer(0.2, self.draw)
        self.get_logger().info(f'[view] drawing {len(self.agents)} agents, '
                               f'{len(self.loads)} stands')

    # ── inputs ────────────────────────────────────────────────────────────

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get('model'):
            self.model = Model(payload['model'])
            self.shape = (payload.get('worlds', len(self.model.worlds)),
                          payload.get('designated', len(self.model.designated)))
            self.assess()

    def on_radio(self, msg):
        try:
            self.radio = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.radio_until = now + (2.5 if self.radio.get('state') == 'delivered' else 1e9)

    def on_beacon(self, msg):
        self.lit = msg.data.split()[1] if msg.data.startswith('lit ') else ''

    def on_lift(self, msg):
        try:
            self.lift = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_sight(self, msg):
        try:
            self.sighting = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_odom(self, agent, msg):
        p = msg.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.poses[agent] = (p.position.x, p.position.y, yaw)

    # ── what is known ─────────────────────────────────────────────────────

    def assess(self):
        m = self.model
        facts = {}
        for s in self.loads:
            atom = f'job_{s}'
            if not m.settled(atom):
                continue
            k, chain = m.depth(atom, self.agents)
            facts[s] = {'knows': {a: m.knows(a, atom) for a in self.agents},
                        'depth': 'C' if k is None else k, 'chain': chain}
        self.facts = facts
        # Who knows where the other stands: K_j at-view(i), for i != j.
        where = {}
        if self.positions:
            for i in self.agents:
                for j in self.agents:
                    if i != j:
                        where[(j, i)] = m.knows(j, f'at-view_{i}')
        self.where = where
        parts = []
        for s, f in facts.items():
            for a in self.agents:
                parts.append(f'K_{a} {"" if f["knows"][a] else "~"}job_{s}')
            d = f['depth']
            parts.append(f'depth job_{s} ' + ('C' if d == 'C' else f'E^{d}'))
            if f['chain']:
                parts.append('chain ' + '>'.join(f['chain']))
        for (j, i), known in where.items():
            parts.append(f'K_{j} {"" if known else "~"}at-view_{i}')
        body = ' · '.join(parts) if parts else 'the order has not been read'
        line = f'[knows] {self.shape[0]} worlds, {self.shape[1]} designated: {body}'
        if line != self.last_line:
            self.last_line = line
            self.get_logger().info(line)
            self.table_pub.publish(String(data=json.dumps(
                {'worlds': self.shape[0], 'designated': self.shape[1], 'facts': facts})))

    # ── drawing ───────────────────────────────────────────────────────────

    def marker(self, mid, ns, kind, rgb, a=1.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id = ns, mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = rgba(rgb, a)
        return m

    def text(self, mid, ns, x, y, z, words, size, rgb, a=1.0):
        m = self.marker(mid, ns, Marker.TEXT_VIEW_FACING, rgb, a)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.z = size
        m.text = words
        return m

    def cube(self, mid, ns, x, y, z, sx, sy, sz, rgb, a=1.0):
        m = self.marker(mid, ns, Marker.CUBE, rgb, a)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        return m

    def gone(self, mid, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.action = ns, mid, Marker.DELETE
        return m

    def draw(self):
        out = MarkerArray()

        # The stands: the load, its name, a tile per robot, and the depth.
        for k, (s, (x0, y0, x1, y1)) in enumerate(self.loads.items()):
            cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
            raised = self.lift.get('stand') == s and self.lift.get('phase') in ('raise', 'lifted')
            out.markers.append(self.cube(k, 'load', cx, cy, 0.6 if raised else 0.3,
                                         x1 - x0, y1 - y0, 0.5, LOAD, 0.9))
            # Inside the block, which the floor plan draws black: the name in white.
            out.markers.append(self.text(k, 'stand_name', cx - 2.0, cy, 1.0, s, 1.8,
                                         (1.0, 1.0, 1.0)))
            f = self.facts.get(s)
            for j, agent in enumerate(self.agents):
                knows = bool(f and f['knows'][agent])
                ty = cy + (-1.0 if j == 0 else 1.0) * 0.55
                out.markers.append(self.cube(10 * k + j, 'tile', cx + 1.9, ty, 0.6, 0.9, 0.9, 0.05,
                                             KNOWS if knows else UNSURE, 0.95))
                out.markers.append(self.cube(10 * k + j, 'tile_rim', cx + 1.9, ty, 0.58, 1.15, 1.15,
                                             0.04, self.colour[agent], 1.0))
            if f:
                # On the storage floor beside the stand, clear of the block.
                d = f['depth']
                words = 'C job' if d == 'C' else f'E^{d} job'
                out.markers.append(self.text(k, 'depth', cx, y0 - 3.4, 1.0, words, 2.4,
                                             COMMON if d == 'C' else KNOWS))
            else:
                out.markers.append(self.gone(k, 'depth'))

        # The beacon, its sight lines and the two viewpoints.
        if self.has_beacon:
            bx, by = self.beacon
            on = bool(self.lit)
            lamp = self.marker(0, 'beacon', Marker.SPHERE, BEACON_ON if on else BEACON_OFF, 1.0)
            lamp.pose.position.x, lamp.pose.position.y, lamp.pose.position.z = bx, by, 1.0
            lamp.scale.x = lamp.scale.y = lamp.scale.z = 1.3 if on else 0.8
            out.markers.append(lamp)
            label = f'beacon: {self.lit}' if on else 'beacon'
            out.markers.append(self.text(0, 'beacon_name', bx - 1.6, by, 1.2, label, 1.3,
                                         BEACON_ON if on else INK))
            for j, agent in enumerate(self.agents):
                vx, vy = self.views[agent]
                there = agent in self.poses and math.hypot(self.poses[agent][0] - vx,
                                                           self.poses[agent][1] - vy) < 0.6
                line = self.marker(j, 'sight', Marker.LINE_STRIP, BEACON_ON if there else UNSURE,
                                   0.9 if there else 0.5)
                line.points = [Point(x=vx, y=vy, z=0.4), Point(x=bx, y=by, z=0.4)]
                line.scale.x = 0.18 if there else 0.08
                out.markers.append(line)
                tri = self.marker(j, 'viewpoint', Marker.CYLINDER, self.colour[agent], 0.6)
                tri.pose.position.x, tri.pose.position.y, tri.pose.position.z = vx, vy, 0.05
                tri.scale.x = tri.scale.y = 0.9
                tri.scale.z = 0.05
                out.markers.append(tri)

        # Unannounced positions: whether the other robot knows this one is at
        # its viewpoint, written in the open aisle on this robot's side of t2.
        if self.positions:
            for j, agent in enumerate(self.agents):
                vx, vy = self.views[agent]
                other = [a for a in self.agents if a != agent][0]
                known = self.where.get((other, agent), False)
                side = 1.0 if vy > 0 else -1.0
                words = f'K_{other} at-view({agent}): {"yes" if known else "no"}'
                out.markers.append(self.text(j, 'where', vx, side * 9.6, 1.0, words,
                                             1.0, KNOWS if known else WHERE_UNKNOWN))
            if self.crates:
                x0, y0, x1, y1 = self.crates
                out.markers.append(self.cube(0, 'crates', 0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.7,
                                             x1 - x0, y1 - y0, 1.4, CRATES, 1.0))
            a, b = self.sighting.get('a'), self.sighting.get('b')
            if a in self.poses and b in self.poses:
                clear = bool(self.sighting.get('clear'))
                line = self.marker(0, 'robots_sight', Marker.LINE_STRIP,
                                   KNOWS if clear else BLOCKED, 0.95)
                line.points = [Point(x=self.poses[a][0], y=self.poses[a][1], z=0.6),
                               Point(x=self.poses[b][0], y=self.poses[b][1], z=0.6)]
                line.scale.x = 0.22
                out.markers.append(line)
            else:
                out.markers.append(self.gone(0, 'robots_sight'))

        # The terminal.
        tx, ty = self.terminal
        out.markers.append(self.cube(0, 'terminal', tx, ty, 0.5, 0.7, 1.0, 1.0, (0.15, 0.3, 0.65)))
        out.markers.append(self.text(0, 'terminal_name', tx + 1.6, ty, 1.0, 'work order', 1.1,
                                     (0.15, 0.3, 0.65)))

        # The robots.
        for j, agent in enumerate(self.agents):
            if agent not in self.poses:
                continue
            x, y, yaw = self.poses[agent]
            disc = self.marker(j, 'robot', Marker.CYLINDER, self.colour[agent])
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.25
            disc.scale.x = disc.scale.y = 1.1
            disc.scale.z = 0.3
            out.markers.append(disc)
            arrow = self.marker(j, 'heading', Marker.ARROW, INK)
            arrow.points = [Point(x=x, y=y, z=0.45),
                            Point(x=x + 1.1 * math.cos(yaw), y=y + 1.1 * math.sin(yaw), z=0.45)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.18, 0.4, 0.4
            out.markers.append(arrow)
            out.markers.append(self.text(j, 'robot_name', x - 1.5, y, 1.0, agent, 1.4,
                                         self.colour[agent]))

        # A radio message: an arc over the block from sender to receiver.
        now = self.get_clock().now().nanoseconds * 1e-9
        r = self.radio
        if r and r.get('state') in ('sending', 'delivered') and now < self.radio_until \
                and r.get('from') in self.poses and r.get('to') in self.poses:
            a, b = self.poses[r['from']], self.poses[r['to']]
            arc = self.marker(0, 'radio', Marker.LINE_STRIP, self.colour[r['from']], 0.9)
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(dx, dy) or 1.0
            bulge = 0.18 * n
            for i in range(31):
                t = i / 30.0
                off = 4.0 * t * (1.0 - t) * bulge
                arc.points.append(Point(x=a[0] + t * dx - dy / n * off,
                                        y=a[1] + t * dy + dx / n * off, z=1.5))
            arc.scale.x = 0.3 if r['state'] == 'delivered' else 0.18
            out.markers.append(arc)
            # The label goes in the storage floor's central lane, the one long
            # clear band on screen, and not on the arc, which crosses the block.
            label = f'{r["kind"]}({r["from"]} to {r["to"]}, {r["stand"]})'
            if r['state'] == 'delivered':
                label += '  delivered'
            out.markers.append(self.text(0, 'radio_text', 0.0, -12.5, 2.0, label, 1.5,
                                         self.colour[r['from']]))
            out.markers.append(self.gone(1, 'radio_text'))
        else:
            out.markers += [self.gone(0, 'radio'), self.gone(0, 'radio_text'),
                            self.gone(1, 'radio_text')]

        self.markers.publish(out)


def main():
    rclpy.init()
    node = KnowledgeView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
