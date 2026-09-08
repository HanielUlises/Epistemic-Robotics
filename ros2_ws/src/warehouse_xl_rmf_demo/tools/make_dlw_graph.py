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
Writes the Open-RMF navigation graph for the dynamic logistics warehouse.

The floor is hand-furnished and not parametric, so the graph is derived rather
than declared: the world is rasterised into an occupancy grid, the grid is
inflated by the robot's radius, a lattice is laid over what remains, and every
lane is kept only if the segment between its ends is clear. Nothing is placed
by eye, which on a floor of 153 obstacles is not a stylistic preference.

Aisle waypoints are found rather than listed. A cell is called an aisle when
it is free, has obstacles close on both sides along one axis and open floor
along the other -- which is exactly the place a robot has to enter before it
can tell anything about what is in there, and so exactly what an epistemic
sensing action wants to name.

    make_dlw_graph.py --world <path>/warehouse.world \
                      --out maps/nav_graphs/0.yaml
"""

import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_world  # noqa: E402


RESOLUTION = 0.25          # metres per cell
ROBOT_RADIUS = 0.25        # the TurtleBot3 Waffle's disc, from the fleet config
CLEARANCE = ROBOT_RADIUS + 0.20
# The lattice is laid finely and thinned afterwards, which is not the same as
# laying it coarsely. Six of the racks are eighteen metres long, so the floor
# is crossed by walls with few gaps, and at two-metre spacing the lattice
# missed enough of those gaps to leave the warehouse in three disconnected
# north-south bands -- 147 waypoints, 80 and 55 -- of which the
# largest-component filter kept one. At 1.25 m every gap is found and the floor
# is a single component; the thinning below then removes what the fine spacing
# cost, since a waypoint in the middle of a straight run carries no
# information a lane does not already carry.
LATTICE = 1.25             # metres between lattice nodes
EDGE_MARGIN = 1.2          # keep off the very edge of the slab
AISLE_SPACING = 4.0        # metres between the aisle waypoints kept

# The nine floor tiles are placed by hand and do not quite abut: the western
# pair leaves a twelve-centimetre seam at x = 6.9, and there are others. A seam
# is not a hole in the building, but a rasteriser cannot tell the difference,
# and the consequence is severe rather than cosmetic -- the floor came out as
# three disconnected north-south bands of 147, 73 and 55 waypoints, and the
# largest-component filter then silently discarded two thirds of the warehouse.
# Growing each tile by a little more than the widest seam closes them without
# inventing floor anywhere a robot could fall.
FLOOR_SEAM = 0.2


class Grid:
    def __init__(self, floor, obstacles):
        xs = [b[0] for b in floor] + [b[2] for b in floor]
        ys = [b[1] for b in floor] + [b[3] for b in floor]
        self.x0, self.x1 = min(xs), max(xs)
        self.y0, self.y1 = min(ys), max(ys)
        self.w = int((self.x1 - self.x0) / RESOLUTION) + 1
        self.h = int((self.y1 - self.y0) / RESOLUTION) + 1

        self.floor = bytearray(self.w * self.h)
        self.blocked = bytearray(self.w * self.h)
        for b in floor:
            self._fill(self.floor, b, FLOOR_SEAM)
        for b in obstacles:
            self._fill(self.blocked, b, CLEARANCE)

    def _cells(self, box, grow):
        x0, y0, x1, y1 = box
        cx0 = max(0, int((x0 - grow - self.x0) / RESOLUTION))
        cx1 = min(self.w - 1, int((x1 + grow - self.x0) / RESOLUTION))
        cy0 = max(0, int((y0 - grow - self.y0) / RESOLUTION))
        cy1 = min(self.h - 1, int((y1 + grow - self.y0) / RESOLUTION))
        return cx0, cy0, cx1, cy1

    def _fill(self, buf, box, grow):
        cx0, cy0, cx1, cy1 = self._cells(box, grow)
        for cy in range(cy0, cy1 + 1):
            row = cy * self.w
            for cx in range(cx0, cx1 + 1):
                buf[row + cx] = 1

    def cell(self, x, y):
        return (int((x - self.x0) / RESOLUTION), int((y - self.y0) / RESOLUTION))

    def free(self, x, y):
        cx, cy = self.cell(x, y)
        if not (0 <= cx < self.w and 0 <= cy < self.h):
            return False
        i = cy * self.w + cx
        return bool(self.floor[i]) and not self.blocked[i]

    def clear_line(self, a, b, step=RESOLUTION / 2):
        (x0, y0), (x1, y1) = a, b
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
        for i in range(n + 1):
            t = i / n
            if not self.free(x0 + t * (x1 - x0), y0 + t * (y1 - y0)):
                return False
        return True

    def enclosure(self, x, y, reach=1.5):
        """How boxed in a free point is, as (along_x, along_y).

        True on an axis means the floor runs out within `reach` on both sides.
        A point boxed in on one axis and open on the other is in an aisle.
        """
        out = []
        for dx, dy in ((RESOLUTION, 0.0), (0.0, RESOLUTION)):
            sides = 0
            for sign in (1, -1):
                d = RESOLUTION
                while d <= reach:
                    if not self.free(x + sign * dx / RESOLUTION * d,
                                     y + sign * dy / RESOLUTION * d):
                        sides += 1
                        break
                    d += RESOLUTION
            out.append(sides == 2)
        return tuple(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--footprints', default=None)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    fp = args.footprints or os.path.join(here, '..', 'config', 'aws_footprints.json')
    floor, obstacles = read_world.read(args.world, fp)
    grid = Grid(floor, obstacles)

    # The lattice, on free floor and off the slab edge.
    nodes, index = [], {}
    nx = int((grid.x1 - grid.x0 - 2 * EDGE_MARGIN) / LATTICE) + 1
    ny = int((grid.y1 - grid.y0 - 2 * EDGE_MARGIN) / LATTICE) + 1
    for iy in range(ny):
        for ix in range(nx):
            x = grid.x0 + EDGE_MARGIN + ix * LATTICE
            y = grid.y0 + EDGE_MARGIN + iy * LATTICE
            if grid.free(x, y):
                index[(ix, iy)] = len(nodes)
                nodes.append([round(x, 3), round(y, 3), {}])

    # Lanes between lattice neighbours whose connecting segment is clear.
    lanes = []
    for (ix, iy), i in index.items():
        for dx, dy in ((1, 0), (0, 1)):
            j = index.get((ix + dx, iy + dy))
            if j is None:
                continue
            if grid.clear_line((nodes[i][0], nodes[i][1]),
                               (nodes[j][0], nodes[j][1])):
                lanes.append([i, j, {}])
                lanes.append([j, i, {}])

    # Keep only what the fleet can actually reach from the largest component:
    # an isolated pocket of floor is a waypoint RMF will accept and never route
    # to, which fails at dispatch rather than here.
    adj = {i: set() for i in range(len(nodes))}
    for a, b, _ in lanes:
        adj[a].add(b)
    seen, best = set(), []
    for start in range(len(nodes)):
        if start in seen:
            continue
        stack, comp = [start], []
        seen.add(start)
        while stack:
            n = stack.pop()
            comp.append(n)
            for m in adj[n]:
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        if len(comp) > len(best):
            best = comp
    keep = sorted(best)
    remap = {old: new for new, old in enumerate(keep)}
    nodes = [nodes[i] for i in keep]
    lanes = [[remap[a], remap[b], p] for a, b, p in lanes
             if a in remap and b in remap]

    # Thin the roadmap.
    #
    # The fine lattice is laid for connectivity and is far too dense to route
    # on: at 1.25 m the warehouse came to 693 waypoints and 2200 lanes, and
    # RMF's planner did not return a path for a sixty-metre errand in seventy
    # seconds. What follows removes a node whenever its neighbours can be
    # joined directly without it, which leaves every route that existed before
    # and costs a waypoint. It is the degree-two case generalised: a node is
    # droppable when every pair of its neighbours has a clear straight run.
    #
    # Nodes are visited from the most open floor inwards, so the middle of a
    # hall goes before the mouth of an aisle, and aisle nodes are kept: they
    # are the places the domain needs to be able to name.
    def thin(nodes, lanes):
        adj = {i: set() for i in range(len(nodes))}
        for a, b, _ in lanes:
            adj[a].add(b)
            adj[b].add(a)

        # Which aisle nodes to protect. Every cell down a rack aisle answers
        # to the test, so protecting all of them protects a fifth of the floor
        # and the thinning has nothing left to remove. One per aisle is what
        # the domain needs -- a name for the place -- so they are taken
        # greedily with a minimum separation.
        candidates = []
        for i, (x, y, _) in enumerate(nodes):
            along_x, along_y = grid.enclosure(x, y)
            if along_x != along_y:
                candidates.append(i)
        keep_named = []
        for i in candidates:
            if all(math.dist(nodes[i][:2], nodes[j][:2]) > AISLE_SPACING
                   for j in keep_named):
                keep_named.append(i)
        keep_named = set(keep_named)

        order = sorted(adj, key=lambda i: len(adj[i]), reverse=True)
        for i in order:
            if i in keep_named or i not in adj:
                continue
            nbrs = sorted(adj[i])
            if len(nbrs) > 6:
                continue
            joins = []
            ok = True
            for a_i in range(len(nbrs)):
                for b_i in range(a_i + 1, len(nbrs)):
                    a, b = nbrs[a_i], nbrs[b_i]
                    if b in adj[a]:
                        continue
                    if grid.clear_line((nodes[a][0], nodes[a][1]),
                                       (nodes[b][0], nodes[b][1])):
                        joins.append((a, b))
                    else:
                        ok = False
                        break
                if not ok:
                    break
            if not ok:
                continue
            for a, b in joins:
                adj[a].add(b)
                adj[b].add(a)
            for n in nbrs:
                adj[n].discard(i)
            del adj[i]

        keep = sorted(adj)
        remap = {old: new for new, old in enumerate(keep)}
        out_nodes = [nodes[i] for i in keep]
        out_lanes = []
        for i in keep:
            for j in adj[i]:
                if j in remap:
                    out_lanes.append([remap[i], remap[j], {}])
        return out_nodes, out_lanes

    before = len(nodes)
    nodes, lanes = thin(nodes, lanes)
    print(f'  thinned {before} -> {len(nodes)} waypoints')

    # Name the aisles: boxed in on one axis, open on the other.
    aisles = []
    for i, (x, y, props) in enumerate(nodes):
        along_x, along_y = grid.enclosure(x, y)
        if along_x != along_y:
            if all(math.dist((x, y), nodes[j][:2]) > AISLE_SPACING for j in aisles):
                aisles.append(i)
    for n, i in enumerate(aisles):
        nodes[i][2]['name'] = f'aisle_{n:02d}'

    # Chargers, on the three most open nodes far apart from one another, so the
    # fleet does not begin the run already negotiating with itself.
    def openness(i):
        x, y, _ = nodes[i]
        return sum(1 for a in (0, 1) if not grid.enclosure(x, y)[a])
    free_nodes = [i for i in range(len(nodes)) if 'name' not in nodes[i][2]]
    free_nodes.sort(key=lambda i: (-openness(i), nodes[i][1]))
    chargers = []
    for i in free_nodes:
        if all(math.dist(nodes[i][:2], nodes[j][:2]) > 8.0 for j in chargers):
            chargers.append(i)
        if len(chargers) == 3:
            break
    for n, i in enumerate(chargers):
        nodes[i][2].update({'name': f'charger_{n + 1}', 'is_charger': True,
                            'is_parking_spot': True, 'is_holding_point': True})

    # Everything else gets a name too: RMF routes to names, and an unnamed
    # waypoint cannot be the target of a task.
    for i, (x, y, props) in enumerate(nodes):
        props.setdefault('name', f'w{i:03d}')

    graph = {'building_name': 'dynamic_logistics_warehouse',
             'levels': {'L1': {'lanes': lanes, 'vertices': nodes}}}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as fh:
        yaml.dump(graph, fh, default_flow_style=None, sort_keys=False)

    print(f'{args.out}: {len(nodes)} waypoints ({len(aisles)} aisles, '
          f'{len(chargers)} chargers), {len(lanes)} directed lanes')
    print(f'  floor {grid.x1 - grid.x0:.1f} x {grid.y1 - grid.y0:.1f} m, '
          f'{len(obstacles)} obstacles, clearance {CLEARANCE:.2f} m')


if __name__ == '__main__':
    main()
