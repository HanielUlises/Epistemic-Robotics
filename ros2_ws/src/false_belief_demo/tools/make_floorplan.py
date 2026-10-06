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

The plan is the pass-through floor's racking and walls. The crate is not in
it: it moves, and where it is is what the robots may disagree about.

The domain makes three claims about this floor, each checked on the plan by
casting rays over the occupied cells before anything is written:

  * from the dock, neither bay is in sight: a relocation while the picker
    charges is one it does not observe;
  * from its mouth of a bay, a robot sees along the bay's axis to the crate's
    place and past it to the far floor, so that looking into a bay tells a
    full bay from an empty one;
  * every place a robot is sent is reachable over the inflated plan.
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


def floorplan():
    grid = np.full((L.GRID_HEIGHT, L.GRID_WIDTH), OCCUPIED, dtype=np.int8)
    fill(grid, (L.HALL_MIN_X, L.HALL_MIN_Y, L.HALL_MAX_X, L.HALL_MAX_Y), FREE)
    for b in L.static_obstacles():
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


def sight(grid, a, b):
    """True when the segment a-b crosses no occupied cell."""
    n = int(np.hypot(b[0] - a[0], b[1] - a[1]) / (L.RESOLUTION / 3)) + 1
    for k in range(n + 1):
        t = k / n
        c, r = to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if grid[r, c] == OCCUPIED:
            return False
    return True


def check(grid):
    problems, facts = [], []

    # 1. The dock sees neither bay.
    for bay in L.BAYS:
        x, _ = L.bay_centre(bay)
        for y in (-1.2, 0.0, 1.2):
            if sight(grid, L.DOCK, (x, y)):
                problems.append(f'the dock sees into {bay}')
                break
    facts.append('from the dock: neither bay is in sight')

    # 2. From either mouth a robot sees along the bay, to the crate's place and
    #    through to the far floor.
    for bay in L.BAYS:
        for agent in L.ROBOTS:
            mx, my = L.mouth(bay, agent)
            far = (mx, -my)
            if not sight(grid, (mx, my), L.crate_rest(bay)) or not sight(grid, (mx, my), far):
                problems.append(f'{agent} does not see along {bay} from its mouth')
    facts.append('from either mouth: the bay is in sight along its axis, to the far floor')

    # 3. Reachability over the inflated plan.
    passable = ~inflate(grid, INFLATION)
    targets = {
        'picker': {'dock': L.DOCK, 'drop': L.DROP,
                   **{f'{b} mouth': L.mouth(b, 'picker') for b in L.BAYS}},
        'mover': {'rest': L.MOVER_REST, **{f'{b} mouth': L.mouth(b, 'mover') for b in L.BAYS}},
    }
    for agent, places in targets.items():
        start = L.ROBOTS[agent][1], L.ROBOTS[agent][2]
        region = reachable(passable, start)
        for name, p in places.items():
            c, r = to_cell(*p)
            if not region[r, c]:
                problems.append(f'{agent} cannot reach {name} at {p}')
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
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(7.6, 11))
    shade = np.full(grid.shape + (3,), 1.0)
    shade[grid == OCCUPIED] = (0.55, 0.58, 0.62)
    ext = (L.GRID_ORIGIN[0], L.GRID_ORIGIN[0] + L.GRID_WIDTH * L.RESOLUTION,
           L.GRID_ORIGIN[1], L.GRID_ORIGIN[1] + L.GRID_HEIGHT * L.RESOLUTION)
    ax.imshow(shade, origin='lower', extent=ext, interpolation='nearest')
    w, d, _ = L.CRATE_SIZE
    for bay in L.BAYS:
        x, y = L.bay_centre(bay)
        ax.text(x, y + 1.0, bay, ha='center', fontsize=11, weight='bold')
    x, y = L.crate_rest(L.CRATE_START)
    ax.add_patch(Rectangle((x - w / 2, y - d / 2), w, d, fc=(0.55, 0.35, 0.15)))
    for name, centre, size, rgb in (('dock', L.DOCK, L.DOCK_PAD, (0.2, 0.45, 0.8)),
                                    ('drop', L.DROP, L.DROP_PAD, (0.3, 0.6, 0.3))):
        x0, y0, x1, y1 = L.pad_box(centre, size)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=rgb, alpha=0.5))
        ax.text(x1 + 0.3, y1, name, fontsize=8)
    for agent, (ns, x, y, _, _, rgb) in L.ROBOTS.items():
        ax.plot(x, y, marker='o', ms=9, color=rgb)
        ax.text(x + 0.6, y + 0.4, agent, fontsize=9, color=rgb)
    ax.set_xlim(L.HALL_MIN_X - 0.5, L.HALL_MAX_X + 0.5)
    ax.set_ylim(L.HALL_MIN_Y - 0.5, L.HALL_MAX_Y + 0.5)
    ax.set_aspect('equal')
    ax.set_title('a false belief on the pass-through floor, 30 x 50 m\n'
                 'the crate starts in t1; the dock is out of sight of both bays', fontsize=10)
    fig.savefig(path, dpi=110, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='directory; writes floorplan.pgm and .yaml')
    ap.add_argument('--plot', help='also draw the floor to this PNG')
    args = ap.parse_args()

    grid = floorplan()
    for fact in check(grid):
        print('  ' + fact)
    os.makedirs(args.out, exist_ok=True)
    write(grid, os.path.join(args.out, 'floorplan'))
    print(f'{args.out}: {grid.shape[1]} x {grid.shape[0]} cells at {L.RESOLUTION} m; '
          'floor check passed')
    if args.plot:
        plot(grid, args.plot)
        print(args.plot)


if __name__ == '__main__':
    main()
