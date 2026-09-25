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
Draws what each agent knows, on the floor it is about.

Two things are drawn and they come from different places, deliberately.

What each agent has *observed or been sent* comes from its knowledge map: the
cells it holds, tinted in its colour. That is the SLAM half, and it is a
region of floor.

What each agent *knows* about the three bays comes from the epistemic model the
state publishes after every product update, evaluated here: K_i open_t when
open_t holds in every world agent i considers possible from every designated
world, K_i not open_t when it holds in none, and neither otherwise. That is the
DEL half, and it is a table. Each bay carries three small tiles, one per
agent, in the colour of that agent's answer, so the two halves can be read
against each other on the map: the carrier's tile on t2 turns open while its
knowledge map still holds nothing of t2.

The table the node publishes on /pass_through/knowledge, and logs as a
[knows] line whenever it changes, adds one row the tiles cannot show:
distributed knowledge of the two scouts, D{west,east}, what they would know
if they pooled what they know. In the elimination branch it settles t2 before
anyone knows it, and it is the content the two announcements to the carrier
deliver. The recording sets the table under the two panes.

The floor plan is published here as well, from the same file the robots load,
so RViz draws the plan they plan on and not a copy of it.
"""

import json
import math
import os

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

OPEN = (0.10, 0.62, 0.30)
SHUT = (0.80, 0.16, 0.12)
UNSURE = (0.55, 0.55, 0.55)


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
    # P5, width, height, maxval, then the raster.
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

    def reach(self, agents, world):
        """Worlds reachable from `world` by every agent of the group at once."""
        sets = [set(self.relations.get(a, {}).get(world, [world])) for a in agents]
        return set.intersection(*sets) if sets else {world}

    def status(self, agents, atom):
        """'open' if the group knows it, 'shut' if it knows the negation, else '?'.

        One agent is K, several are D: the worlds none of them can rule out
        together are the intersection of what each cannot rule out.
        """
        values = set()
        for w in self.designated:
            for v in self.reach(agents, w):
                values.add(atom in self.labels.get(v, set()))
        if values == {True}:
            return 'open'
        if values == {False}:
            return 'shut'
        return '?'


class KnowledgeView(Node):

    def __init__(self):
        super().__init__('knowledge_view')
        self.declare_parameter('floorplan', '')
        self.declare_parameter('agents', ['west', 'east', 'carrier'])
        self.declare_parameter('namespaces', ['r1', 'r2', 'r3'])
        self.declare_parameter('colours', [0.2, 0.45, 0.8, 0.9, 0.5, 0.15, 0.18, 0.6, 0.35])
        self.declare_parameter('bays', ['t1', 't2', 't3'])
        self.declare_parameter('bay_boxes', [0.0] * 12)
        self.declare_parameter('scouts', ['west', 'east'])
        self.declare_parameter('coverage_resolution', 0.3)

        self.agents = list(self.get_parameter('agents').value)
        self.namespaces = dict(zip(self.agents, self.get_parameter('namespaces').value))
        flat = list(self.get_parameter('colours').value)
        self.colour = {a: tuple(flat[3 * i:3 * i + 3]) for i, a in enumerate(self.agents)}
        names = list(self.get_parameter('bays').value)
        boxes = list(self.get_parameter('bay_boxes').value)
        self.bays = {n: tuple(boxes[4 * i:4 * i + 4]) for i, n in enumerate(names)}
        self.scouts = list(self.get_parameter('scouts').value)
        self.coarse = float(self.get_parameter('coverage_resolution').value)

        self.model = None
        self.shape = (0, 0)
        self.poses = {}
        self.known = {}
        self.link = ''
        self.acting = ''
        self.last_table = None

        self.markers = self.create_publisher(MarkerArray, '/pass_through/markers', LATCHED)
        self.coverage = self.create_publisher(MarkerArray, '/pass_through/coverage', LATCHED)
        self.table_pub = self.create_publisher(String, '/pass_through/knowledge', LATCHED)
        plan_pub = self.create_publisher(OccupancyGrid, '/pass_through/floorplan', LATCHED)
        path = self.get_parameter('floorplan').value
        if path:
            plan_pub.publish(read_pgm(path))
        self.plan_pub = plan_pub

        self.create_subscription(String, '/epistemic_state/state', self.on_state,
                                 QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/pass_through/link', self.on_link,
                                 QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/pass_through/acting', self.on_acting,
                                 QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        for agent, ns in self.namespaces.items():
            self.create_subscription(
                Odometry, f'/{ns}/odom', lambda m, a=agent: self.on_odom(a, m), 10)
            self.create_subscription(
                OccupancyGrid, f'/{ns}/known_map', lambda m, a=agent: self.on_known(a, m), LATCHED)

        self.create_timer(0.25, self.draw)
        self.create_timer(2.0, self.draw_coverage)
        self.get_logger().info(
            f'[view] drawing {len(self.agents)} agents over {len(self.bays)} bays')

    # ── inputs ────────────────────────────────────────────────────────────

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get('model'):
            self.model = Model(payload['model'])
            self.shape = (payload.get('worlds', 0), payload.get('designated', 0))

    def on_link(self, msg):
        self.link = msg.data

    def on_acting(self, msg):
        self.acting = msg.data

    def on_odom(self, agent, msg):
        p = msg.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.poses[agent] = (p.position.x, p.position.y, yaw)

    def on_known(self, agent, msg):
        self.known[agent] = msg

    # ── the table ─────────────────────────────────────────────────────────

    def table(self):
        if self.model is None:
            return None
        rows = []
        for agent in self.agents:
            rows.append((agent, [self.model.status([agent], f'open_{t}') for t in self.bays]))
        group = 'D{' + ','.join(self.scouts) + '}'
        rows.append((group, [self.model.status(self.scouts, f'open_{t}') for t in self.bays]))
        return rows

    def log_table(self, rows):
        facts = []
        for who, cells in rows:
            op = who if who.startswith('D{') else f'K_{who}'
            for t, s in zip(self.bays, cells):
                if s == 'open':
                    facts.append(f'{op} open_{t}')
                elif s == 'shut':
                    facts.append(f'{op} ~open_{t}')
        line = ' · '.join(facts) if facts else 'nobody knows anything about the bays'
        self.get_logger().info(
            f'[knows] {self.shape[0]} worlds, {self.shape[1]} designated: {line}')
        self.table_pub.publish(String(data=json.dumps(
            {'worlds': self.shape[0], 'designated': self.shape[1],
             'rows': [[w, c] for w, c in rows], 'bays': list(self.bays)})))

    # ── drawing ───────────────────────────────────────────────────────────

    def text(self, mid, ns, x, y, z, words, size, rgb, a=1.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id = ns, mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.z = size
        m.color = rgba(rgb, a)
        m.text = words
        return m

    def cube(self, mid, ns, x, y, z, sx, sy, sz, rgb, a=1.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id = ns, mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.color = rgba(rgb, a)
        return m

    def draw(self):
        out = MarkerArray()
        rows = self.table()
        if rows is not None and rows != self.last_table:
            self.last_table = rows
            self.log_table(rows)
        status = {who: dict(zip(self.bays, cells)) for who, cells in (rows or [])}

        # Each bay: its name and one tile per agent. The bay itself is left clear, so
        # that the cells the carrier's safe set lifts show through in cyan.
        for k, (t, (x0, y0, x1, y1)) in enumerate(self.bays.items()):
            cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
            out.markers.append(self.text(k, 'bay_name', x0 - 1.1, cy, 0.5, t, 1.5,
                                         (0.15, 0.15, 0.15)))
            for j, agent in enumerate(self.agents):
                s = status.get(agent, {}).get(t, '?')
                rgb = OPEN if s == 'open' else SHUT if s == 'shut' else UNSURE
                ty = y0 + 0.55 + j * ((y1 - y0 - 1.1) / max(1, len(self.agents) - 1))
                out.markers.append(self.cube(10 * k + j, 'tile', cx, ty, 0.3, 0.9, 0.7, 0.05,
                                             rgb, 0.95))
                # The agent's colour as a rim, so a tile says whose it is.
                out.markers.append(self.cube(10 * k + j, 'tile_rim', cx, ty, 0.28, 1.15, 0.95,
                                             0.04, self.colour[agent], 1.0))

        # The robots: a disc in the agent's colour, a heading, and its name.
        for j, agent in enumerate(self.agents):
            if agent not in self.poses:
                continue
            x, y, yaw = self.poses[agent]
            disc = Marker()
            disc.header.frame_id = 'map'
            disc.ns, disc.id = 'robot', j
            disc.type = Marker.CYLINDER
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.25
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 1.1
            disc.scale.z = 0.3
            disc.color = rgba(self.colour[agent])
            out.markers.append(disc)
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.ns, arrow.id = 'heading', j
            arrow.type = Marker.ARROW
            arrow.points = [Point(x=x, y=y, z=0.45),
                            Point(x=x + 1.1 * math.cos(yaw), y=y + 1.1 * math.sin(yaw), z=0.45)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.18, 0.4, 0.4
            arrow.color = rgba((0.1, 0.1, 0.1))
            out.markers.append(arrow)
            label = agent + ('  (acting)' if agent == self.acting else '')
            out.markers.append(self.text(j, 'robot_name', x, y + 1.6, 1.0, label, 1.4,
                                         self.colour[agent]))

        # A map exchange in progress: a line from sender to receiver.
        link = Marker()
        link.header.frame_id = 'map'
        link.ns, link.id = 'link', 0
        parts = self.link.split()
        if len(parts) >= 2 and parts[0] in self.poses and parts[1] in self.poses:
            a, b = self.poses[parts[0]], self.poses[parts[1]]
            link.type = Marker.LINE_STRIP
            link.action = Marker.ADD
            link.points = [Point(x=a[0], y=a[1], z=0.8), Point(x=b[0], y=b[1], z=0.8)]
            link.scale.x = 0.35
            link.color = rgba(self.colour[parts[0]], 0.85)
            out.markers.append(link)
            what = f'share-{parts[3]}({parts[0]}, {parts[1]}, {parts[2]})' if len(parts) >= 4 else ''
            out.markers.append(self.text(1, 'link_text', 0.5 * (a[0] + b[0]) + 1.5,
                                         0.5 * (a[1] + b[1]), 1.2, what, 1.5,
                                         self.colour[parts[0]]))
        else:
            link.action = Marker.DELETE
            out.markers.append(link)
            gone = Marker()
            gone.header.frame_id = 'map'
            gone.ns, gone.id, gone.action = 'link_text', 1, Marker.DELETE
            out.markers.append(gone)

        self.markers.publish(out)

    def draw_coverage(self):
        """Each agent's knowledge map, coarsened, tinted in its colour.

        Everything the agent holds is drawn, observed or received. A scout's
        region grows as it drives, which is its SLAM map being built; the
        carrier's grows in steps, at each exchange, by the scout's region.
        """
        out = MarkerArray()
        for j, agent in enumerate(self.agents):
            msg = self.known.get(agent)
            if msg is None:
                continue
            w, h, res = msg.info.width, msg.info.height, msg.info.resolution
            ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
            grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
            step = max(1, int(round(self.coarse / res)))
            hh, ww = (h // step) * step, (w // step) * step
            blocks = grid[:hh, :ww].reshape(hh // step, step, ww // step, step)
            seen = (blocks >= 0).any(axis=(1, 3))
            occupied = (blocks > 65).any(axis=(1, 3))
            free_m = Marker()
            free_m.header.frame_id = 'map'
            free_m.ns, free_m.id = 'seen', j
            free_m.type = Marker.CUBE_LIST
            free_m.pose.orientation.w = 1.0
            free_m.scale.x = free_m.scale.y = step * res
            free_m.scale.z = 0.02
            free_m.color = rgba(self.colour[agent], 0.22)
            occ_m = Marker()
            occ_m.header.frame_id = 'map'
            occ_m.ns, occ_m.id = 'seen_occupied', j
            occ_m.type = Marker.CUBE_LIST
            occ_m.pose.orientation.w = 1.0
            occ_m.scale.x = occ_m.scale.y = step * res
            occ_m.scale.z = 0.05
            occ_m.color = rgba(tuple(0.6 * c for c in self.colour[agent]), 0.9)
            z = 0.03 + 0.02 * j
            rows, cols = np.nonzero(seen)
            for r, c in zip(rows, cols):
                x = ox + (c + 0.5) * step * res
                y = oy + (r + 0.5) * step * res
                # An occupied cell outside the bays is racking the floor plan
                # already draws; inside a bay it is a load, and it is drawn.
                if occupied[r, c]:
                    if any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in self.bays.values()):
                        occ_m.points.append(Point(x=x, y=y, z=z + 0.1))
                    continue
                free_m.points.append(Point(x=x, y=y, z=z))
            out.markers.extend([free_m, occ_m])
        self.coverage.publish(out)


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
