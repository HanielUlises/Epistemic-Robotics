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
Writes the Open-RMF navigation graph for the larger warehouse.

The shape of the graph is the shape of the argument the domain makes. One
service lane runs the length of the building; every aisle opens onto it and
onto nothing else. So an aisle is a place a robot can only see into by going
to its mouth, and the floor has thirty-four of them.

Every waypoint and every lane is checked against the floor before it is
written. The small warehouse demo learnt this the hard way: a lane drawn by
eye along the south wall looked reasonable and passed through the shelving,
and the robot drove into it and stopped with nothing in RMF saying why. Here
the obstacles are known exactly, because `layout.py` placed them, so the check
is analytic rather than a rasterisation, and it is not optional.

    make_nav_graph.py --out maps/nav_graphs/0.yaml
"""

import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402


CLEARANCE = 0.45   # TinyRobot's footprint radius is 0.3; this leaves margin.


def obstacles():
    """Every solid thing on the floor, as (min_x, min_y, max_x, max_y)."""
    boxes = []
    for px in L.PILLAR_X:
        for py in L.PILLAR_Y:
            h = L.PILLAR_SIDE / 2.0
            boxes.append((px - h, py - h, px + h, py + h))
    half = L.SHELF_DEPTH / 2.0
    for y in L.rows():
        for sign in (-1.0, 1.0):
            x0, x1 = sorted((sign * L.RACK_MIN_X, sign * L.RACK_MAX_X))
            boxes.append((x0, y - half, x1, y + half))
    return boxes


def blocked(x, y, clearance=CLEARANCE):
    if not L.inside_hall(x, y, margin=clearance):
        return True
    return any(x0 - clearance < x < x1 + clearance and
               y0 - clearance < y < y1 + clearance
               for x0, y0, x1, y1 in obstacles())


def lane_blocked(a, b, step=0.15):
    """Sample the segment. A lane is only as good as its worst point."""
    (x0, y0), (x1, y1) = a, b
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    for i in range(n + 1):
        t = i / n
        if blocked(x0 + t * (x1 - x0), y0 + t * (y1 - y0)):
            return True
    return False


class Graph:
    def __init__(self):
        self.vertices = []
        self.lanes = []
        self._by_name = {}

    def add(self, x, y, name='', **props):
        if name and name in self._by_name:
            return self._by_name[name]
        if blocked(x, y):
            raise SystemExit(
                f'waypoint {name or "(unnamed)"} at ({x:.2f}, {y:.2f}) '
                f'is not on free floor')
        p = {'name': name} if name else {}
        p.update(props)
        self.vertices.append([round(x, 3), round(y, 3), p])
        idx = len(self.vertices) - 1
        if name:
            self._by_name[name] = idx
        return idx

    def link(self, i, j):
        a = (self.vertices[i][0], self.vertices[i][1])
        b = (self.vertices[j][0], self.vertices[j][1])
        if lane_blocked(a, b):
            raise SystemExit(
                f'lane {self.vertices[i][2].get("name", i)} -> '
                f'{self.vertices[j][2].get("name", j)} crosses an obstacle')
        self.lanes.append([i, j, {}])
        self.lanes.append([j, i, {}])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    g = Graph()
    aisles = L.aisles()

    # The service lane, as a chain. Its nodes sit opposite every aisle mouth so
    # that entering one is a turn and not a detour.
    lane_ys = sorted({round(c, 3) for c, _, _ in aisles})
    lane_ys = [L.LANE_MIN_Y] + lane_ys + [L.LANE_MAX_Y]
    lane_nodes = [g.add(0.0, y, f'lane_{i:02d}') for i, y in enumerate(lane_ys)]
    for a, b in zip(lane_nodes, lane_nodes[1:]):
        g.link(a, b)

    # The aisles. `east_07_mouth` is on the lane side and `east_07` is inside,
    # which is the distinction the domain rests on: standing at the mouth is
    # what makes the aisle observable.
    mouth_x = L.RACK_MIN_X - 0.55
    deep_x = 0.5 * (L.RACK_MIN_X + L.RACK_MAX_X)

    # How far down an aisle a robot may stand is set by the pillars and not by
    # the racks. The pillar rows sit just beyond the far end of every aisle, so
    # the deepest safe point is the pillar face less the clearance rather than
    # the rack end less a guess. Five of the thirty-four aisles end opposite a
    # pillar, and the check refuses the other choice.
    pillar_face = L.PILLAR_X[1] - L.PILLAR_SIDE / 2.0
    end_x = min(L.RACK_MAX_X - 0.35, pillar_face - CLEARANCE - 0.05)

    for n, (cy, _, _) in enumerate(aisles):
        opposite = lane_nodes[lane_ys.index(round(cy, 3))]
        for side, tag in ((1.0, 'east'), (-1.0, 'west')):
            mouth = g.add(side * mouth_x, cy, f'{tag}_{n:02d}_mouth')
            inside = g.add(side * deep_x, cy, f'{tag}_{n:02d}')
            end = g.add(side * end_x, cy, f'{tag}_{n:02d}_end')
            g.link(opposite, mouth)
            g.link(mouth, inside)
            g.link(inside, end)

    # The docks, in the outer bays past the pillars, reached from the ends of
    # the lane by a cross aisle that clears the last rack row.
    south, north = lane_nodes[0], lane_nodes[-1]
    for sign, tag in ((1.0, 'east'), (-1.0, 'west')):
        for ly, dy, end_node, label in ((L.LANE_MIN_Y, L.DOCK_Y[0], south, 'south'),
                                        (L.LANE_MAX_Y, L.DOCK_Y[1], north, 'north')):
            cross = g.add(sign * L.DOCK_X, ly, f'{tag}_{label}_cross')
            dock = g.add(sign * L.DOCK_X, dy, f'dock_{tag}_{label}')
            g.link(end_node, cross)
            g.link(cross, dock)

    # One charger per robot, off the cross aisles where nothing routes through.
    #
    # Distinct waypoints and not a shared one: a waypoint holds exactly one
    # robot, so two robots nominating the same charger is the "Failed
    # negotiation" the fleet config warns about, arriving at the moment the
    # second one tries to go home.
    for sign, tag, label, dy in ((1.0, 'east', 'south', -1.6),
                                 (-1.0, 'west', 'south', -1.6),
                                 (1.0, 'east', 'north', 1.6)):
        y = (L.LANE_MIN_Y if label == 'south' else L.LANE_MAX_Y) + dy
        c = g.add(sign * L.DOCK_X, y, f'charger_{tag}_{label}',
                  is_charger=True, is_parking_spot=True, is_holding_point=True)
        g.link(g._by_name[f'{tag}_{label}_cross'], c)

    graph = {
        'building_name': 'warehouse_xl',
        'levels': {'L1': {'lanes': g.lanes, 'vertices': g.vertices}},
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as out:
        yaml.dump(graph, out, default_flow_style=None, sort_keys=False)

    named = sum(1 for v in g.vertices if v[2].get('name'))
    print(f'{args.out}: {len(g.vertices)} vertices ({named} named), '
          f'{len(g.lanes)} directed lanes, {len(aisles) * 2} aisles')


if __name__ == '__main__':
    main()
