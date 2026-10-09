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
Draws every robot's map of the three bays, beside the robot, against the
floor.

Each robot carries a row of three tiles, t1 t2 t3, in the colour its own map
reads the bay: green clear, red blocked. A tile whose reading the floor
contradicts is a stale entry, and is ringed in yellow. With a model, on the
epistemic fleet, a second row under the first is what the model says the
robot believes, evaluated here on the model the epistemic state publishes
after every update: B_i blocked(t) when blocked(t) holds at every world i
considers possible from every designated world. The two rows are read
against each other on screen: the model's row is the domain's claim about
the robot's map.

What the floor is comes from the crew's events: the shift map's loads, and
each change the forklift makes. A map exchange is drawn as a line from
sender to receiver while it is held.

The table the node publishes on /stale_maps/table, and logs as a [maps] line
whenever it changes, is the same thing in text: every robot's map, its stale
entries, and the model's row.
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
EVENTS = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE)

CLEAR = (0.10, 0.62, 0.30)
BLOCKED = (0.80, 0.16, 0.12)
UNSURE = (0.55, 0.55, 0.55)
STALE = (1.00, 0.85, 0.05)

TILE = 0.62


def rgba(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def read_pgm(yaml_path):
    """A map_server map as an OccupancyGrid, read the way the robots read it."""
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
        self.designated = payload.get('designated', [])
        self.labels = {w: set(a) for w, a in payload.get('labels', {}).items()}
        self.relations = payload.get('relations', {})

    def belief(self, agent, atom):
        """'blocked' if the agent believes it, 'clear' if it believes the
        negation, '?' if neither, '!' if it has no world."""
        values = set()
        for w in self.designated:
            seen = self.relations.get(agent, {}).get(w, [])
            if not seen:
                return '!'
            values |= {atom in self.labels.get(v, set()) for v in seen}
        if values == {True}:
            return 'blocked'
        if values == {False}:
            return 'clear'
        return '?'


class KnowledgeView(Node):

    def __init__(self):
        super().__init__('stale_maps_view')
        self.declare_parameter('floorplan', '')
        self.declare_parameter('agents', [''])
        self.declare_parameter('haulers', [''])
        self.declare_parameter('colours', [0.0])
        self.declare_parameter('bays', [''])
        self.declare_parameter('load_boxes', [0.0])
        self.declare_parameter('shift_blocked', [''])
        self.declare_parameter('fleet', 'epistemic')

        self.agents = self.get_parameter('agents').value
        self.haulers = set(self.get_parameter('haulers').value)
        c = self.get_parameter('colours').value
        self.colours = {a: tuple(c[3 * k:3 * k + 3]) for k, a in enumerate(self.agents)}
        self.bays = self.get_parameter('bays').value
        b = self.get_parameter('load_boxes').value
        self.load_boxes = {t: tuple(b[4 * k:4 * k + 4]) for k, t in enumerate(self.bays)}
        self.blocked = set(self.get_parameter('shift_blocked').value)
        self.fleet = self.get_parameter('fleet').value

        self.readings = {}
        self.poses = {}
        self.model = None
        self.link = ''
        self.sends = 0
        self.delivered = []
        self.last_table = None

        self.floor_pub = self.create_publisher(OccupancyGrid, '/stale_maps/floorplan', LATCHED)
        # Latched: RViz's marker display asks for transient local, and a
        # volatile publisher's markers are never delivered to it.
        self.markers = self.create_publisher(MarkerArray, '/stale_maps/markers', LATCHED)
        self.table_pub = self.create_publisher(String, '/stale_maps/table', LATCHED)
        path = self.get_parameter('floorplan').value
        if path:
            self.floor_pub.publish(read_pgm(path))

        for a in self.agents:
            self.create_subscription(String, f'/{a}/readings',
                                     lambda m, a=a: self.on_readings(a, m), LATCHED)
            self.create_subscription(Odometry, f'/{a}/odom',
                                     lambda m, a=a: self.on_odom(a, m), 10)
        self.create_subscription(String, '/stale_maps/events', self.on_event, EVENTS)
        self.create_subscription(String, '/stale_maps/link', self.on_link, LATCHED)
        self.create_subscription(
            String, '/epistemic_state/state', self.on_state,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.5, self.draw)

    # ─── Inputs ──────────────────────────────────────────────────────────────

    def on_readings(self, agent, msg):
        try:
            self.readings[agent] = json.loads(msg.data)
        except ValueError:
            pass

    def on_odom(self, agent, msg):
        p = msg.pose.pose.position
        self.poses[agent] = (p.x, p.y)

    def on_event(self, msg):
        try:
            e = json.loads(msg.data)
        except ValueError:
            return
        if not e.get('ok'):
            return
        verb, args = e.get('verb'), e.get('args', [])
        if verb == 'stage':
            self.blocked.add(args[0])
        elif verb == 'clear':
            self.blocked.discard(args[0])
        elif verb == 'send':
            self.sends += 1
        elif verb == 'haul':
            self.delivered.append(args[0])

    def on_link(self, msg):
        self.link = msg.data

    def on_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if 'model' in payload:
            self.model = Model(payload['model'])

    # ─── The table ───────────────────────────────────────────────────────────

    def reading(self, agent, bay):
        r = self.readings.get(agent, {}).get('bays', {}).get(bay, {})
        return r.get('verdict', '')

    def table(self):
        rows = {}
        stale = 0
        for a in self.agents:
            row = {}
            for t in self.bays:
                v = self.reading(a, t)
                wrong = (v == 'clear' and t in self.blocked) or (v == 'blocked' and t not in self.blocked)
                stale += wrong
                belief = self.model.belief(a, f'blocked_{t}') if self.model else ''
                row[t] = {'map': v, 'stale': bool(wrong), 'model': belief}
            rows[a] = row
        return {'fleet': self.fleet, 'floor': {t: 'blocked' if t in self.blocked else 'clear'
                                               for t in self.bays},
                'maps': rows, 'stale': stale, 'sends': self.sends, 'delivered': self.delivered}

    @staticmethod
    def short(v):
        return {'clear': 'o', 'blocked': 'x', '?': '?', '!': '!', '': '-'}.get(v, '?')

    def log_table(self, t):
        key = json.dumps({k: v for k, v in t.items() if k != 'sends'}, sort_keys=True)
        if key == self.last_table:
            return
        self.last_table = key
        parts = []
        for a, row in t['maps'].items():
            cells = ''.join(self.short(row[b]['map']) + ('*' if row[b]['stale'] else '')
                            for b in self.bays)
            model = ''.join(self.short(row[b]['model']) for b in self.bays) if self.model else ''
            parts.append(f'{a} {cells}' + (f' [{model}]' if model else ''))
        floor = ''.join(self.short(v) for v in t['floor'].values())
        self.get_logger().info(
            f'[maps] floor {floor}; {t["stale"]} stale; ' + '  '.join(parts))

    # ─── Drawing ─────────────────────────────────────────────────────────────

    def marker(self, ns, mid, kind):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def draw(self):
        t = self.table()
        self.table_pub.publish(String(data=json.dumps(t)))
        self.log_table(t)
        out = MarkerArray()

        # The floor: a load where there is one, an outline where there is not.
        for k, bay in enumerate(self.bays):
            x0, y0, x1, y1 = self.load_boxes[bay]
            m = self.marker('floor', k, Marker.CUBE)
            m.pose.position.x, m.pose.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            loaded = bay in self.blocked
            m.pose.position.z = 0.6 if loaded else 0.02
            m.scale.x, m.scale.y = x1 - x0, y1 - y0
            m.scale.z = 1.2 if loaded else 0.04
            m.color = rgba(BLOCKED if loaded else CLEAR, 0.55 if loaded else 0.35)
            out.markers.append(m)
            label = self.marker('floor_label', k, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = (x0 + x1) / 2, 0.0
            label.pose.position.z = 2.2
            label.scale.z = 0.9
            label.color = rgba((1, 1, 1))
            label.text = bay
            out.markers.append(label)

        # Each robot's map, and under it the model's row.
        mid = 0
        for a in self.agents:
            if a not in self.poses:
                continue
            x, y = self.poses[a]
            name = self.marker('name', mid, Marker.TEXT_VIEW_FACING)
            name.pose.position.x, name.pose.position.y = x, y - 0.9
            name.pose.position.z = 0.8
            name.scale.z = 0.6
            name.color = rgba(self.colours.get(a, (1, 1, 1)))
            name.text = a + (' hauls' if a in self.haulers else '')
            out.markers.append(name)
            rows = [('map', 1.0)] + ([('model', 0.35)] if self.model else [])
            for r, (kind, z) in enumerate(rows):
                for k, bay in enumerate(self.bays):
                    cell = t['maps'][a][bay]
                    v = cell[kind]
                    colour = CLEAR if v == 'clear' else BLOCKED if v == 'blocked' else UNSURE
                    tile = self.marker(f'{kind}_tile', mid * 3 + k, Marker.CUBE)
                    tile.pose.position.x = x + (k - 1) * (TILE + 0.08)
                    tile.pose.position.y = y + 0.9 - r * (TILE * 0.7)
                    tile.pose.position.z = z
                    tile.scale.x = TILE
                    tile.scale.y = TILE * (1.0 if kind == 'map' else 0.5)
                    tile.scale.z = 0.08
                    tile.color = rgba(colour, 0.95 if kind == 'map' else 0.7)
                    out.markers.append(tile)
                    if kind == 'map':
                        ring = self.marker('stale', mid * 3 + k, Marker.CUBE)
                        ring.pose.position.x = tile.pose.position.x
                        ring.pose.position.y = tile.pose.position.y
                        ring.pose.position.z = z - 0.06
                        ring.scale.x = ring.scale.y = TILE + 0.22
                        ring.scale.z = 0.04
                        ring.color = rgba(STALE, 1.0 if cell['stale'] else 0.0)
                        out.markers.append(ring)
            mid += 1

        # An exchange in progress.
        line = self.marker('link', 0, Marker.LINE_STRIP)
        line.scale.x = 0.12
        words = self.link.split()
        if len(words) >= 3 and words[0] in self.poses and words[1] in self.poses:
            for a in words[:2]:
                px, py = self.poses[a]
                line.points.append(Point(x=px, y=py, z=1.4))
            line.color = rgba((0.2, 0.6, 1.0), 0.9)
        else:
            line.action = Marker.DELETE
        out.markers.append(line)
        caption = self.marker('link_text', 0, Marker.TEXT_VIEW_FACING)
        if len(words) >= 3 and words[0] in self.poses and words[1] in self.poses:
            (ax, ay), (bx, by) = self.poses[words[0]], self.poses[words[1]]
            caption.pose.position.x, caption.pose.position.y = (ax + bx) / 2, (ay + by) / 2
            caption.pose.position.z = 2.0
            caption.scale.z = 0.7
            caption.color = rgba((0.2, 0.6, 1.0))
            caption.text = f'{words[0]} -> {words[1]}: {words[2]}' + (
                f' ({words[3]})' if len(words) > 3 else '')
        else:
            caption.action = Marker.DELETE
        out.markers.append(caption)

        # The count, along the south wall of the storage floor, where it fits
        # the recording's view.
        board = self.marker('board', 0, Marker.TEXT_VIEW_FACING)
        board.pose.position.x, board.pose.position.y, board.pose.position.z = 13.6, -12.0, 1.0
        board.scale.z = 0.7
        board.color = rgba((1, 1, 1))
        board.text = (f'{self.fleet}: {t["stale"]} stale entries, {t["sends"]} maps of a bay sent, '
                      f'{len(t["delivered"])} of {len(self.haulers)} delivered')
        out.markers.append(board)
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
