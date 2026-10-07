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
Checks the floor the muddy robots' domain assumes, on warehouse_scenario's
floor plan of the AWS small warehouse, and draws it.

    check_floor.py [--robots 4] [--plot docs/floorplan.png]

The domain makes two claims about the floor, and each is checked on the plan
by casting rays over the occupied cells, or flooding the free ones:

  * on the muster ring every robot has every other robot in line of sight:
    the initial model, in which each knows whether every other is faulty,
    rests on it;
  * every place a robot is sent, its place on the ring, its place in the
    calibration bay and its station, is reachable from where it is sent
    from, over the plan inflated by the driver's 0.35 m.
"""

import argparse
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

INFLATION = 0.35


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


def load(yaml_path):
    meta = {}
    with open(yaml_path) as fh:
        for line in fh:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    image = os.path.join(os.path.dirname(yaml_path), meta['image'])
    raster, _ = pgm_raster(image)
    occupied = raster < 128
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return occupied[::-1], float(meta['resolution']), origin


class Plan:

    def __init__(self, yaml_path):
        self.occ, self.res, origin = load(yaml_path)
        self.ox, self.oy = origin[0], origin[1]
        k = int(np.ceil(INFLATION / self.res))
        inflated = self.occ.copy()
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc <= k * k:
                    inflated |= np.roll(np.roll(self.occ, dr, axis=0), dc, axis=1)
        self.free = ~inflated

    def cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def sight(self, a, b, skip=0.25):
        """No occupied cell on the segment, short of `skip` of either end:
        the robots themselves are not on the plan, and a lamp is above the
        chassis."""
        n = int(np.hypot(b[0] - a[0], b[1] - a[1]) / (self.res / 3)) + 1
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            if min(np.hypot(x - a[0], y - a[1]), np.hypot(x - b[0], y - b[1])) < skip:
                continue
            r, c = self.cell(x, y)
            if self.occ[r, c]:
                return False
        return True

    def reachable(self, start):
        seen = np.zeros_like(self.free)
        r, c = self.cell(*start)
        if not self.free[r, c]:
            return seen
        seen[r, c] = True
        todo = deque([(r, c)])
        while todo:
            r, c = todo.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < seen.shape[0] and 0 <= nc < seen.shape[1] and \
                        self.free[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    todo.append((nr, nc))
        return seen


def check(plan, n):
    problems, facts = [], []
    fleet = list(L.robots(n))
    spots = {a: L.muster_spot(k, n)[:2] for k, a in enumerate(fleet)}
    for i, a in enumerate(fleet):
        for b in fleet[i + 1:]:
            if not plan.sight(spots[a], spots[b]):
                problems.append(f'{a} and {b} do not see each other on the muster ring')
    facts.append(f'on the muster ring: each of the {n} robots sees every other')
    for k, a in enumerate(fleet):
        station = L.FLEET[a][1:3]
        bay = L.bay_spot(k)[:2]
        for name, start, goal in (('its place on the ring', station, spots[a]),
                                  ('the calibration bay', spots[a], bay),
                                  ('its station', spots[a], station)):
            region = plan.reachable(start)
            r, c = plan.cell(*goal)
            if not region[r, c]:
                problems.append(f'{a} cannot reach {name} at {goal} from {start}')
    facts.append('every place a robot is sent is reachable over the inflated plan')
    if problems:
        raise SystemExit('floor check failed:\n  ' + '\n  '.join(problems))
    return facts


def draw(plan, n, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    fig, ax = plt.subplots(figsize=(6.4, 9.0))
    shade = np.where(plan.occ[..., None], np.array([0.55, 0.58, 0.62]), np.array([1.0, 1.0, 1.0]))
    h, w = plan.occ.shape
    ext = (plan.ox, plan.ox + w * plan.res, plan.oy, plan.oy + h * plan.res)
    ax.imshow(shade, origin='lower', extent=ext, interpolation='nearest')
    mx, my = L.MUSTER
    ax.add_patch(Circle((mx, my), L.RING + 0.55, fill=False, ec=(0.9, 0.7, 0.05), lw=2, ls='--'))
    x, y, length, depth = L.BAY_PAD
    ax.add_patch(Rectangle((x - length / 2, y - depth / 2), length, depth,
                           fc=(0.15, 0.35, 0.75), alpha=0.5))
    ax.text(x, y - 0.9, 'calibration bay', ha='center', fontsize=8, color=(0.15, 0.35, 0.75))
    ax.plot(*L.PA, marker='^', ms=9, color=(0.95, 0.6, 0.05))
    ax.text(L.PA[0] + 0.3, L.PA[1], 'PA', fontsize=8, va='center')
    spots = [L.muster_spot(k, n) for k in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            ax.plot([spots[i][0], spots[j][0]], [spots[i][1], spots[j][1]],
                    color=(0.9, 0.2, 0.1), lw=0.6, alpha=0.6)
    for k, (agent, (ns, sx, sy, rgb)) in enumerate(L.robots(n).items()):
        ax.add_patch(Circle((sx, sy), 0.25, color=rgb))
        ax.text(sx + 0.35, sy + 0.25, f'{agent} station', fontsize=7, color=rgb)
        ax.add_patch(Circle(spots[k][:2], 0.22, color=rgb))
        bx, by, _ = L.bay_spot(k)
        ax.add_patch(Rectangle((bx - 0.2, by - 0.2), 0.4, 0.4, fc='none', ec=rgb, lw=1.2))
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('the muddy robots on the AWS small warehouse, 14 x 21 m\n'
                 f'{n} stations, the muster ring with its sight lines, the PA and the bay',
                 fontsize=10)
    fig.savefig(path, dpi=110, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', type=int, default=4)
    ap.add_argument('--plot')
    args = ap.parse_args()
    plan = Plan(L.floorplan_yaml())
    for fact in check(plan, args.robots):
        print('  ' + fact)
    print(f'floor check passed for {args.robots} robots')
    if args.plot:
        draw(plan, args.robots, args.plot)
        print(args.plot)


if __name__ == '__main__':
    main()
