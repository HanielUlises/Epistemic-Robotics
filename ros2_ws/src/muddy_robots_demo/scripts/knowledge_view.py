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
Draws what the muddy robots know about their own lamps, for RViz.

Everything here is computed from the model the epistemic state publishes
after every product update:

  each robot   whether it knows it is faulty, knows it is not, or does not
               know: faulty(i) in every world i considers possible from the
               actual world, or in none of them, or neither. The actual world
               is the designated one whose lit lamps are the lamps lit; the
               other designated worlds are the patterns the executor, which
               does not see the lamps, cannot rule out, and no robot's
  depth        the largest k with E^k of "some lamp is lit", where E is "every
               robot knows": a breadth-first walk over the robots' relations
               from the actual world to the nearest world with no lamp lit.
               None is common knowledge, C.
  worlds       how many worlds a chain of the robots' relations reaches from
               the actual world: what the robots can still consider

These are logged as a [knows] line whenever they change, which the video's
captions are timed from.

On the floor: each robot with its lamp, lit or dark, as the others see it,
and under it what it knows of its own; the muster ring, with the depth beside
it; the public address, lit once the supervisor has spoken; the calibration
bay; and the bell, while it rings.
"""

import json
import math
import os
from collections import deque

import numpy as np

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

INK = (0.12, 0.12, 0.12)
KNOWS_FAULTY = (0.85, 0.12, 0.10)
KNOWS_CLEAN = (0.10, 0.62, 0.30)
UNSURE = (0.45, 0.45, 0.45)
COMMON = (0.78, 0.06, 0.18)
MUSTER = (0.95, 0.78, 0.10)
BAY = (0.15, 0.35, 0.75)
PA_ON = (1.0, 0.72, 0.10)
PA_OFF = (0.25, 0.25, 0.28)
BELL = (0.25, 0.75, 1.0)
LAMP_LIT = (1.0, 0.08, 0.05)
LAMP_DARK = (0.22, 0.22, 0.22)


def rgba(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def pgm_raster(path):
    """The raster of a binary PGM, its header read token by token so that the
    comment line warehouse_scenario writes is skipped."""
    with open(path, 'rb') as fh:
        data = fh.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w), maxval


def read_pgm(yaml_path):
    meta = {}
    with open(yaml_path) as fh:
        for line in fh:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    image = meta['image']
    if not image.startswith('/'):
        image = os.path.join(os.path.dirname(yaml_path), image)
    raster, maxval = pgm_raster(image)
    height, width = raster.shape
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

    def __init__(self, payload):
        self.worlds = payload.get('worlds', [])
        self.designated = payload.get('designated', [])
        self.labels = {w: set(a) for w, a in payload.get('labels', {}).items()}
        self.relations = payload.get('relations', {})

    def sees(self, agent, world):
        return self.relations.get(agent, {}).get(world, [world])

    def actual(self, lit):
        """The designated world with exactly these lamps lit, as a list of
        one; all of the designated worlds when none matches."""
        for w in self.designated:
            if {a[7:] for a in self.labels.get(w, ()) if a.startswith('faulty_')} == lit:
                return [w]
        return list(self.designated)

    def status(self, agent, points):
        atom = f'faulty_{agent}'
        seen = {atom in self.labels.get(v, ()) for w in points
                for v in self.sees(agent, w)}
        if seen == {True}:
            return 'faulty'
        if seen == {False}:
            return 'clean'
        return '?'

    def reachable(self, agents, points):
        seen, todo = set(points), list(points)
        while todo:
            u = todo.pop()
            for a in agents:
                for v in self.sees(a, u):
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
        return len(seen)

    def depth(self, agents, points):
        """(k, chain) for "some lamp is lit", from the given worlds; k is None
        for C."""
        def lit(w):
            return any(a.startswith('faulty_') for a in self.labels.get(w, ()))
        if not points or not all(lit(w) for w in points):
            return -1, []
        parent = {w: None for w in points}
        todo = deque((w, 0) for w in points)
        while todo:
            u, d = todo.popleft()
            for a in agents:
                for v in self.sees(a, u):
                    if v in parent:
                        continue
                    parent[v] = (u, a)
                    if not lit(v):
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
        self.declare_parameter('agents', ['r1', 'r2', 'r3', 'r4'])
        self.declare_parameter('colours', [0.5] * 12)
        self.declare_parameter('faults', [''])
        self.declare_parameter('muster', [0.0, 0.0, 1.2])
        self.declare_parameter('pa', [0.0, 0.0])
        self.declare_parameter('bay', [0.0, 0.0, 3.0, 1.0])

        self.agents = list(self.get_parameter('agents').value)
        flat = list(self.get_parameter('colours').value)
        self.colour = {a: tuple(flat[3 * i:3 * i + 3]) for i, a in enumerate(self.agents)}
        self.faults = {f for f in self.get_parameter('faults').value if f}
        self.muster = tuple(self.get_parameter('muster').value)
        self.pa = tuple(self.get_parameter('pa').value)
        self.bay = tuple(self.get_parameter('bay').value)

        self.model = None
        self.poses = {}
        self.announced = False
        self.bell = {}
        self.bell_until = 0.0
        self.bells = 0
        self.facts = {}
        self.last_line = None

        self.markers = self.create_publisher(MarkerArray, '/muddy_robots/markers', LATCHED)
        self.table_pub = self.create_publisher(String, '/muddy_robots/knowledge', LATCHED)
        plan_pub = self.create_publisher(OccupancyGrid, '/muddy_robots/floorplan', LATCHED)
        path = self.get_parameter('floorplan').value
        if path:
            plan_pub.publish(read_pgm(path))
        self.plan_pub = plan_pub

        self.create_subscription(String, '/epistemic_state/state', self.on_state, HISTORY)
        self.create_subscription(String, '/muddy_robots/pa', self.on_pa, LATCHED)
        self.create_subscription(String, '/muddy_robots/bell', self.on_bell, HISTORY)
        for agent in self.agents:
            self.create_subscription(
                Odometry, f'/{agent}/odom', lambda m, a=agent: self.on_odom(a, m), 10)
        self.create_timer(0.2, self.draw)
        self.get_logger().info(f'[view] drawing {len(self.agents)} robots; lamps lit: '
                               f'{", ".join(sorted(self.faults)) or "none"}')

    # ── inputs ────────────────────────────────────────────────────────────

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get('model'):
            self.model = Model(payload['model'])
            self.assess()

    def on_pa(self, msg):
        self.announced = bool(msg.data)

    def on_bell(self, msg):
        try:
            self.bell = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.bells = int(self.bell.get('bell', self.bells))
        self.bell_until = self.get_clock().now().nanoseconds * 1e-9 + 3.0

    def on_odom(self, agent, msg):
        p = msg.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.poses[agent] = (p.position.x, p.position.y, yaw)

    # ── what is known ─────────────────────────────────────────────────────

    def assess(self):
        # What the robots know is what they know at the actual world, the one
        # the lamps make the case. The designated worlds are the fault patterns
        # the executor, which does not see the lamps, cannot yet rule out.
        m = self.model
        points = m.actual(self.faults)
        k, chain = m.depth(self.agents, points)
        facts = {'status': {a: m.status(a, points) for a in self.agents},
                 'depth': 'C' if k is None else k, 'chain': chain,
                 'reachable': m.reachable(self.agents, points),
                 'designated': len(m.designated)}
        self.facts = facts
        parts = [f'{a} {facts["status"][a]}' for a in self.agents]
        d = facts['depth']
        parts.append('some-lit ' + ('C' if d == 'C' else f'E^{d}' if d >= 0 else 'false'))
        if chain:
            parts.append('chain ' + '>'.join(chain))
        line = (f'[knows] {facts["reachable"]} worlds the robots can consider, '
                f'{facts["designated"]} patterns the executor cannot rule out: '
                + ' · '.join(parts))
        if line != self.last_line:
            self.last_line = line
            self.get_logger().info(line)
            self.table_pub.publish(String(data=json.dumps(facts)))

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

    def draw(self):
        out = MarkerArray()
        mx, my, ring = self.muster
        now = self.get_clock().now().nanoseconds * 1e-9

        # The muster ring.
        circle = self.marker(0, 'muster', Marker.LINE_STRIP, MUSTER, 0.9)
        r = ring + 0.55
        circle.points = [Point(x=mx + r * math.cos(2 * math.pi * k / 48),
                               y=my + r * math.sin(2 * math.pi * k / 48), z=0.05)
                         for k in range(49)]
        circle.scale.x = 0.08
        out.markers.append(circle)

        # The depth of "some lamp is lit", beside the ring.
        f = self.facts
        d = f.get('depth')
        if d is None and not f:
            words, colour = '', INK
        elif d == 'C':
            words, colour = 'C  some lamp lit', COMMON
        elif isinstance(d, int) and d >= 0:
            words, colour = f'E^{d}  some lamp lit', KNOWS_CLEAN
        else:
            words, colour = '', INK
        out.markers.append(self.text(0, 'depth', mx - 2.6, my, 1.0, words, 0.95, colour))
        if f:
            # Beside the depth, clear of the racks above it.
            out.markers.append(self.text(0, 'worlds', mx - 2.6, my + 7.3, 1.0,
                                         f'{f["reachable"]} worlds still possible', 0.45, INK))

        # The public address.
        px, py = self.pa
        mast = self.marker(0, 'pa', Marker.CYLINDER, PA_ON if self.announced else PA_OFF)
        mast.pose.position.x, mast.pose.position.y, mast.pose.position.z = px, py, 0.4
        mast.scale.x = mast.scale.y = 0.6 if self.announced else 0.35
        mast.scale.z = 0.8
        out.markers.append(mast)
        # Off the ring, beyond the mast: the view runs x down the screen and
        # y across it, and anything nearer the ring prints over the robots.
        out.markers.append(self.text(0, 'pa_text', px + 0.9, py - 1.3, 1.0,
                                     '"at least one of you is faulty"' if self.announced
                                     else 'public address', 0.5,
                                     PA_ON if self.announced else INK))

        # The bell, while it rings.
        if now < self.bell_until:
            flash = self.marker(0, 'bell', Marker.CYLINDER, BELL, 0.35)
            flash.pose.position.x, flash.pose.position.y, flash.pose.position.z = mx, my, 0.06
            flash.scale.x = flash.scale.y = 2 * (ring + 0.4)
            flash.scale.z = 0.04
            out.markers.append(flash)
        else:
            gone = Marker()
            gone.header.frame_id = 'map'
            gone.ns, gone.id, gone.action = 'bell', 0, Marker.DELETE
            out.markers.append(gone)
        if self.bells:
            out.markers.append(self.text(0, 'bell_text', mx + 2.5, my, 1.0,
                                         f'bell {self.bells}', 0.8, BELL))

        # The calibration bay.
        bx, by, bl, bd = self.bay
        pad = self.marker(0, 'bay', Marker.CUBE, BAY, 0.5)
        pad.pose.position.x, pad.pose.position.y, pad.pose.position.z = bx, by, 0.02
        pad.scale.x, pad.scale.y, pad.scale.z = bl, bd, 0.02
        out.markers.append(pad)
        out.markers.append(self.text(0, 'bay_text', bx, by - 1.0, 0.6, 'calibration bay', 0.5,
                                     BAY))

        # The robots, their lamps, and what each knows of its own.
        status = f.get('status', {})
        for j, agent in enumerate(self.agents):
            if agent not in self.poses:
                continue
            x, y, yaw = self.poses[agent]
            disc = self.marker(j, 'robot', Marker.CYLINDER, self.colour[agent])
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.15
            disc.scale.x = disc.scale.y = 0.55
            disc.scale.z = 0.2
            out.markers.append(disc)
            arrow = self.marker(j, 'heading', Marker.ARROW, INK)
            arrow.points = [Point(x=x, y=y, z=0.3),
                            Point(x=x + 0.55 * math.cos(yaw), y=y + 0.55 * math.sin(yaw), z=0.3)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.18, 0.18
            out.markers.append(arrow)
            lit = agent in self.faults
            lamp = self.marker(j, 'lamp', Marker.SPHERE, LAMP_LIT if lit else LAMP_DARK)
            lamp.pose.position.x, lamp.pose.position.y, lamp.pose.position.z = x, y, 0.6
            lamp.scale.x = lamp.scale.y = lamp.scale.z = 0.38 if lit else 0.26
            out.markers.append(lamp)
            # On the ring, the name and what the robot knows go out along its
            # radius, so that four robots a metre and a half apart do not
            # print over each other; elsewhere above and below it.
            dx, dy = x - mx, y - my
            reach = math.hypot(dx, dy)
            if 0.3 < reach < 2.5:
                ux, uy = dx / reach, dy / reach
                name_at = (x + 0.75 * ux, y + 0.75 * uy)
                knows_at = (x + 1.5 * ux, y + 1.5 * uy)
            else:
                name_at, knows_at = (x, y + 0.75), (x, y - 0.75)
            out.markers.append(self.text(j, 'name', *name_at, 1.0, agent, 0.6,
                                         self.colour[agent]))
            s = status.get(agent)
            words, colour = {'faulty': ('knows: faulty', KNOWS_FAULTY),
                             'clean': ('knows: not faulty', KNOWS_CLEAN),
                             '?': ('does not know', UNSURE)}.get(s, ('', INK))
            out.markers.append(self.text(j, 'knows', *knows_at, 1.0, words, 0.5, colour))

        self.markers.publish(out)


def main():
    rclpy.init()
    node = KnowledgeView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
