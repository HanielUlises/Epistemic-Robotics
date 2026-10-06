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
Writes the floor plan both robots are given, and checks the geometry the
domain assumes.

    make_floorplan.py --out maps --plot docs/floorplan.png

Unlike the pass-through floor plan, this one has nothing unknown in it: the
loads, the beacon and the terminal are drawn in, because where things are is
common knowledge here. The uncertainty is in the work order.

The domain makes four claims about this floor, and each is checked on the
plan before anything is written, by casting rays over the occupied cells:

  * from where they start, neither robot sees the other or the beacon;
  * from its viewpoint, each robot sees the beacon;
  * two robots under the two ends of one load cannot see each other, the
    load being between them;
  * every place a robot is sent is reachable over the inflated plan.

The first is why the radio is needed at all, the second is the observability
condition of signal, and the third is why meeting at the stand does not
replace the beacon: under the load the robots are as blind to each other as
they were at the start.
"""

import argparse
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

FREE, OCCUPIED = 0, 100
INFLATION = 0.35


def to_cell(x, y):
    return (int((x - L.GRID_ORIGIN[0]) / L.RESOLUTION),
            int((y - L.GRID_ORIGIN[1]) / L.RESOLUTION))


def fill(grid, box, value):
    x0, y0, x1, y1 = box
    c0, r0 = to_cell(x0, y0)
    c1, r1 = to_cell(x1, y1)
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, L.GRID_WIDTH - 1), min(r1, L.GRID_HEIGHT - 1)
    grid[r0:r1 + 1, c0:c1 + 1] = value


def floorplan(beacon=True):
    grid = np.full((L.GRID_HEIGHT, L.GRID_WIDTH), OCCUPIED, dtype=np.int8)
    fill(grid, (L.HALL_MIN_X, L.HALL_MIN_Y, L.HALL_MAX_X, L.HALL_MAX_Y), FREE)
    for b in L.static_obstacles(beacon=beacon):
        fill(grid, b, OCCUPIED)
    return grid


def inflate(grid, metres):
    k = int(np.ceil(metres / L.RESOLUTION))
    occ = grid == OCCUPIED
    out = occ.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            if dr * dr + dc * dc <= k * k:
                out |= np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
    return out


def reachable(passable, start):
    seen = np.zeros_like(passable)
    c, r = to_cell(*start)
    if not passable[r, c]:
        return seen
    seen[r, c] = True
    queue = deque([(r, c)])
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < passable.shape[0] and 0 <= nc < passable.shape[1] \
                    and passable[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                queue.append((nr, nc))
    return seen


def sight(grid, a, b, ignore=()):
    """True when the segment a-b crosses no occupied cell, other than the
    cells of the boxes in @p ignore (the thing being looked at)."""
    skip = np.zeros(grid.shape, dtype=bool)
    for box in ignore:
        x0, y0, x1, y1 = box
        c0, r0 = to_cell(x0, y0)
        c1, r1 = to_cell(x1, y1)
        skip[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = True
    n = int(np.hypot(b[0] - a[0], b[1] - a[1]) / (L.RESOLUTION / 3)) + 1
    for k in range(n + 1):
        t = k / n
        c, r = to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if grid[r, c] == OCCUPIED and not skip[r, c]:
            return False
    return True


def start_of(agent):
    return L.ROBOTS[agent][1], L.ROBOTS[agent][2]


def check(grid):
    """The four claims. Raises on any that fails; returns the facts it read."""
    problems, facts = [], []
    free = (grid == FREE) & ~inflate(grid, INFLATION)

    def at(region, point):
        c, r = to_cell(*point)
        return bool(region[r, c])

    beacon = L.BEACON_POST
    post = L.beacon_box()

    # 1. Out of sight at the start.
    a, b = start_of('south'), start_of('north')
    if sight(grid, a, b):
        problems.append('south and north see each other from where they start')
    for agent in L.ROBOTS:
        if sight(grid, start_of(agent), beacon, ignore=[post]):
            problems.append(f'{agent} sees the beacon from where it starts')
    facts.append('from the start: neither robot sees the other or the beacon')

    # 2. The viewpoints see the beacon, and each other.
    for agent in L.ROBOTS:
        if not sight(grid, L.viewpoint(agent), beacon, ignore=[post]):
            problems.append(f'{agent} does not see the beacon from its viewpoint')
    both = sight(grid, L.viewpoint('south'), L.viewpoint('north'), ignore=[post])
    facts.append('from the viewpoints: each robot sees the beacon'
                 + (', and the two see each other through t2' if both else ''))
    # And from no other place a robot is sent.
    for agent in L.ROBOTS:
        for stand in L.STANDS:
            for p in (L.mouth(stand, agent), L.under_end(stand, agent)):
                if sight(grid, p, beacon, ignore=[post]):
                    problems.append(f'{agent} sees the beacon at {stand}, not only from t2')
        if agent == L.READS_ORDER and sight(grid, L.TERMINAL_READ, beacon, ignore=[post]):
            problems.append(f'{agent} sees the beacon from the terminal')

    # 3. Under one load, blind to each other.
    for stand in L.STANDS:
        if sight(grid, L.under_end(stand, 'south'), L.under_end(stand, 'north')):
            problems.append(f'the two ends of {stand} see each other past the load')
    facts.append('under the two ends of a load: the robots do not see each other')

    # 4. Reachable. The last metre into a stand is driven straight, not
    # planned, so it is checked against the uninflated plan along the axis.
    for agent in L.ROBOTS:
        region = reachable(free, start_of(agent))
        goals = [('its viewpoint', L.viewpoint(agent))]
        goals += [(f'the mouth of {s}', L.mouth(s, agent)) for s in L.STANDS]
        if agent == L.READS_ORDER:
            goals.append(('the terminal', L.TERMINAL_READ))
        for name, p in goals:
            if not at(region, p):
                problems.append(f'{agent} cannot reach {name} at {p}')
        for s in L.STANDS:
            mx, my = L.mouth(s, agent)
            ux, uy = L.under_end(s, agent)
            for dx in (-0.2, 0.0, 0.2):
                if not sight(grid, (mx + dx, my), (ux + dx, uy)):
                    problems.append(f'{agent} cannot drive straight in under {s}')
                    break
    facts.append('every place a robot is sent is reachable over the inflated plan')

    if problems:
        raise SystemExit('floor check failed:\n  ' + '\n  '.join(problems))
    return facts


def write(grid, stem):
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == FREE] = 254
    img[grid == OCCUPIED] = 0
    img = img[::-1]
    with open(stem + '.pgm', 'wb') as fh:
        fh.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        fh.write(img.tobytes())
    with open(stem + '.yaml', 'w') as fh:
        fh.write(f'image: {os.path.basename(stem)}.pgm\n'
                 f'mode: trinary\n'
                 f'resolution: {L.RESOLUTION}\n'
                 f'origin: [{L.GRID_ORIGIN[0]}, {L.GRID_ORIGIN[1]}, 0.0]\n'
                 f'negate: 0\n'
                 f'occupied_thresh: 0.65\n'
                 f'free_thresh: 0.196\n')


def plot(grid, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    fig, ax = plt.subplots(figsize=(7.6, 11))
    shade = np.full(grid.shape + (3,), 1.0)
    shade[grid == OCCUPIED] = (0.55, 0.58, 0.62)
    ext = (L.GRID_ORIGIN[0], L.GRID_ORIGIN[0] + L.GRID_WIDTH * L.RESOLUTION,
           L.GRID_ORIGIN[1], L.GRID_ORIGIN[1] + L.GRID_HEIGHT * L.RESOLUTION)
    ax.imshow(shade, origin='lower', extent=ext, interpolation='nearest')

    load = (0.55, 0.2, 0.1)
    for s in L.STANDS:
        x0, y0, x1, y1 = L.load_box(s)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=load, alpha=0.85))
        ax.text(L.stand_x(s), 0.0, s, ha='center', va='center', fontsize=11,
                weight='bold', color='white')
        for agent in L.ROBOTS:
            ax.plot(*L.under_end(s, agent), marker='s', ms=5, color=L.ROBOTS[agent][5])
    bx, by = L.BEACON_POST
    ax.add_patch(Circle((bx, by), 0.35, color=(0.1, 0.7, 0.3)))
    ax.text(bx + 0.6, by, 'beacon', fontsize=9, va='center')
    for agent in L.ROBOTS:
        vx, vy = L.viewpoint(agent)
        ax.plot([vx, bx], [vy, by], color=(0.1, 0.7, 0.3), lw=0.8, ls='--')
        ax.plot(vx, vy, marker='^' if agent == 'south' else 'v', ms=8,
                color=L.ROBOTS[agent][5])
    x0, y0, x1, y1 = L.terminal_box()
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=(0.15, 0.3, 0.6)))
    ax.text(x1 + 0.3, y1 + 0.3, 'work order', fontsize=8)
    for agent, (ns, x, y, _, _, rgb) in L.ROBOTS.items():
        ax.add_patch(Circle((x, y), 0.35, color=rgb))
        ax.text(x + 0.6, y + 0.6, f'{agent} ({ns})', color=rgb, fontsize=9, weight='bold')

    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('coordinated attack on the pass-through floor, 30 x 50 m\n'
                 'stands s1, s2 (loads in t1, t3); beacon in t2, seen from its two mouths',
                 fontsize=10)
    fig.savefig(path, dpi=110, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True,
                    help='directory; writes floorplan_beacon and floorplan_radio')
    ap.add_argument('--plot', help='also draw the beacon floor to this PNG')
    args = ap.parse_args()

    grid = floorplan(beacon=True)
    for fact in check(grid):
        print('  ' + fact)
    os.makedirs(args.out, exist_ok=True)
    write(grid, os.path.join(args.out, 'floorplan_beacon'))
    # The radio floor is the same less the post. Nothing there is checked
    # against the beacon, since there is none.
    write(floorplan(beacon=False), os.path.join(args.out, 'floorplan_radio'))
    print(f'{args.out}: {grid.shape[1]} x {grid.shape[0]} cells at {L.RESOLUTION} m, '
          'both floors; floor check passed')
    if args.plot:
        plot(grid, args.plot)
        print(args.plot)


if __name__ == '__main__':
    main()
