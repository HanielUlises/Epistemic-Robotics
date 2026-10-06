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
Draws where the crate is, and where each robot believes it is.

Everything about belief is computed from the model the epistemic state
publishes after every product update. With W* the designated worlds and R_i
agent i's relation:

  B_i crate(b)        crate-at(b) holds at every world i considers possible
                      from every designated world
  B_i B_j crate(b)    the same, one relation further
  consistent(i)       every designated world has a world i considers
                      possible; when one has none, i believes everything and
                      the view says so

For each robot the view gives the bay it believes the crate in, `?` when it
believes it in neither, and `none` when it has no consistent belief. These
are logged as a [believes] line whenever they change, which the video's
captions are timed from, and published on /false_belief/beliefs.

On the floor: the crate where it actually is, from the performers that move
it; for each robot a ghost of the crate in its colour in the bay it believes,
on its own side of the block; each robot's first- and second-order beliefs as
text in the central lane on its side; the dock and the drop; and a report in
flight, as an arc from the mover to the picker.
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
HISTORY = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

CRATE = (0.55, 0.35, 0.15)
INK = (0.12, 0.12, 0.12)
WHITE = (1.0, 1.0, 1.0)
FALSE = (0.78, 0.06, 0.18)
DOCK = (0.16, 0.36, 0.70)
DROP = (0.20, 0.55, 0.30)


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
        return self.relations.get(agent, {}).get(world, [])

    def holds(self, atom):
        return bool(self.designated) and all(atom in self.labels.get(w, ())
                                             for w in self.designated)

    def consistent(self, agent):
        return all(self.sees(agent, w) for w in self.designated)

    def believes(self, chain, atom):
        """B_{chain[0]} ... B_{chain[-1]} atom at every designated world."""
        frontier = set(self.designated)
        for agent in chain:
            frontier = {v for w in frontier for v in self.sees(agent, w)}
        return all(atom in self.labels.get(v, ()) for v in frontier)

    def belief(self, chain, bays):
        """The bay the nested belief places the crate in, '?' when none, and
        'none' when the innermost agent has no consistent belief somewhere
        along the chain."""
        frontier = set(self.designated)
        for agent in chain:
            nxt = set()
            for w in frontier:
                seen = self.sees(agent, w)
                if not seen:
                    return 'none'
                nxt.update(seen)
            frontier = nxt
        held = [b for b in bays if all(f'crate-at_{b}' in self.labels.get(v, ()) for v in frontier)]
        return held[0] if len(held) == 1 else '?'


class KnowledgeView(Node):

    def __init__(self):
        super().__init__('knowledge_view')
        self.declare_parameter('floorplan', '')
        self.declare_parameter('agents', ['picker', 'mover'])
        self.declare_parameter('namespaces', ['r1', 'r2'])
        self.declare_parameter('colours', [0.2, 0.45, 0.8, 0.9, 0.5, 0.15])
        self.declare_parameter('bays', ['t1', 't3'])
        self.declare_parameter('bay_centres', [-10.436, 0.0, 10.436, 0.0])
        self.declare_parameter('crate_start', [-10.436, 0.0])
        self.declare_parameter('crate_size', [0.45, 0.45, 0.32])
        self.declare_parameter('dock', [-13.45, -15.0])
        self.declare_parameter('drop', [-5.2, -6.2])

        self.agents = list(self.get_parameter('agents').value)
        self.namespaces = dict(zip(self.agents, self.get_parameter('namespaces').value))
        flat = list(self.get_parameter('colours').value)
        self.colour = {a: tuple(flat[3 * i:3 * i + 3]) for i, a in enumerate(self.agents)}
        names = list(self.get_parameter('bays').value)
        centres = list(self.get_parameter('bay_centres').value)
        self.bays = {n: (centres[2 * i], centres[2 * i + 1]) for i, n in enumerate(names)}
        start = list(self.get_parameter('crate_start').value)
        self.crate = {'x': start[0], 'y': start[1], 'carried_by': None}
        self.crate_size = list(self.get_parameter('crate_size').value)
        self.dock = tuple(self.get_parameter('dock').value)
        self.drop = tuple(self.get_parameter('drop').value)
        # Which side of the block an agent works: the picker the storage floor
        # (y < 0), the mover the dispatch floor.
        self.side = {self.agents[0]: -1.0, self.agents[1]: 1.0}

        self.model = None
        self.shape = (0, 0)
        self.poses = {}
        self.radio = {}
        self.view = {}
        self.last_line = None

        self.markers = self.create_publisher(MarkerArray, '/false_belief/markers', LATCHED)
        self.table_pub = self.create_publisher(String, '/false_belief/beliefs', LATCHED)
        plan_pub = self.create_publisher(OccupancyGrid, '/false_belief/floorplan', LATCHED)
        path = self.get_parameter('floorplan').value
        if path:
            plan_pub.publish(read_pgm(path))
        self.plan_pub = plan_pub

        self.create_subscription(String, '/epistemic_state/state', self.on_state, HISTORY)
        self.create_subscription(String, '/false_belief/radio', self.on_radio, HISTORY)
        self.create_subscription(String, '/false_belief/crate', self.on_crate, LATCHED)
        for agent, ns in self.namespaces.items():
            self.create_subscription(
                Odometry, f'/{ns}/odom', lambda m, a=agent: self.on_odom(a, m), 10)

        self.create_timer(0.2, self.draw)
        self.get_logger().info(f'[view] drawing {len(self.agents)} agents, {len(self.bays)} bays')

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
        self.radio['until'] = now + (3.0 if self.radio.get('state') == 'delivered' else 1e9)

    def on_crate(self, msg):
        try:
            c = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.crate = {'x': c.get('x', 0.0), 'y': c.get('y', 0.0), 'carried_by': c.get('carried_by')}

    def on_odom(self, agent, msg):
        p = msg.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.poses[agent] = (p.position.x, p.position.y, yaw)

    # ── what is believed ──────────────────────────────────────────────────

    def assess(self):
        m = self.model
        bays = list(self.bays)
        a, b = self.agents
        truth = [x for x in bays if m.holds(f'crate-at_{x}')]
        view = {
            'crate': truth[0] if truth else 'out',
            a: m.belief([a], bays), b: m.belief([b], bays),
            f'{b}>{a}': m.belief([b, a], bays), f'{a}>{b}': m.belief([a, b], bays),
        }
        self.view = view
        body = ' · '.join([f'crate {view["crate"]}', f'B_{a} {view[a]}', f'B_{b} {view[b]}',
                           f'B_{b} B_{a} {view[f"{b}>{a}"]}', f'B_{a} B_{b} {view[f"{a}>{b}"]}'])
        line = f'[believes] {self.shape[0]} worlds, {self.shape[1]} designated: {body}'
        if line != self.last_line:
            self.last_line = line
            self.get_logger().info(line)
            self.table_pub.publish(String(data=json.dumps(
                {'worlds': self.shape[0], 'designated': self.shape[1], **view})))

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
        w, d, h = self.crate_size
        a, b = self.agents

        # The bays, named in white on the block the floor plan draws black.
        for k, (bay, (x, y)) in enumerate(self.bays.items()):
            out.markers.append(self.text(k, 'bay_name', x, y + 2.6, 1.0, bay, 1.6, INK))

        # The dock and the drop.
        for k, (name, (x, y), rgb) in enumerate((('dock', self.dock, DOCK),
                                                ('drop', self.drop, DROP))):
            out.markers.append(self.cube(k, 'pad', x, y, 0.02, 1.0, 1.0, 0.04, rgb, 0.8))
            out.markers.append(self.text(k, 'pad_name', x - 1.2, y, 0.5, name, 1.0, rgb))

        # The crate, where it is.
        c = self.crate
        out.markers.append(self.cube(0, 'crate', c['x'], c['y'], 0.9 if c['carried_by'] else 0.4,
                                     w * 2.0, d * 2.0, 0.6, CRATE, 1.0))

        # Each robot's belief: a ghost of the crate in its colour, in the bay it
        # believes, on its own side of the block; red when the belief is false.
        v = self.view
        truth = v.get('crate')
        for j, agent in enumerate(self.agents):
            bay = v.get(agent)
            if bay in self.bays:
                x, y = self.bays[bay]
                rgb = self.colour[agent]
                ghost = self.cube(j, 'belief', x, y + self.side[agent] * 1.0, 0.7,
                                  w * 2.0, d * 2.0, 0.5, rgb, 0.5)
                out.markers.append(ghost)
                if truth in self.bays and bay != truth:
                    out.markers.append(self.text(j, 'belief_false', x, y + self.side[agent] * 1.0,
                                                 1.6, 'false', 0.9, FALSE))
                else:
                    out.markers.append(self.gone(j, 'belief_false'))
            else:
                out.markers += [self.gone(j, 'belief'), self.gone(j, 'belief_false')]

        # The beliefs as text, in the central lane on each robot's side.
        if v:
            for j, (agent, other) in enumerate(((a, b), (b, a))):
                first = v.get(agent, '?')
                second = v.get(f'{agent}>{other}', '?')
                words = (f'B_{agent} crate: {first}\n'
                         f'B_{agent} B_{other} crate: {second}')
                out.markers.append(self.text(j, 'belief_text', 0.0, self.side[agent] * 9.6, 1.0,
                                             words, 0.95, self.colour[agent]))

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

        # A report in flight: an arc from the mover to the picker.
        now = self.get_clock().now().nanoseconds * 1e-9
        r = self.radio
        if r and r.get('state') in ('sending', 'delivered') and now < r.get('until', 0.0) \
                and r.get('from') in self.poses and r.get('to') in self.poses:
            p, q = self.poses[r['from']], self.poses[r['to']]
            arc = self.marker(0, 'radio', Marker.LINE_STRIP, self.colour[r['from']], 0.9)
            dx, dy = q[0] - p[0], q[1] - p[1]
            n = math.hypot(dx, dy) or 1.0
            bulge = 0.18 * n
            for i in range(31):
                t = i / 30.0
                off = 4.0 * t * (1.0 - t) * bulge
                arc.points.append(Point(x=p[0] + t * dx - dy / n * off,
                                        y=p[1] + t * dy + dx / n * off, z=1.5))
            arc.scale.x = 0.3 if r['state'] == 'delivered' else 0.18
            out.markers.append(arc)
            label = f'report: the crate moved {r.get("moved_from")} to {r.get("moved_to")}'
            if r['state'] == 'delivered':
                label += ', delivered'
            # In the aisle along the far wall, clear of the beliefs in the
            # central lane, which the label is about.
            out.markers.append(self.text(0, 'radio_text', 13.9, 0.0, 2.0, label, 1.1,
                                         self.colour[r['from']]))
        else:
            out.markers += [self.gone(0, 'radio'), self.gone(0, 'radio_text')]

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
