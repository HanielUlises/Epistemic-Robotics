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
The floor's claims, checked on the plan, and the two maps written from it.

    check_floor.py                                  check, and print the sight lines
    check_floor.py --maps maps --plot docs/floorplan.png

The domain makes three claims about the floor. Each is checked here, by
casting rays over the occupied cells of the plan with the other robots
standing at their stations as discs, or by flooding the free cells of the plan
inflated by the navigation's 0.35 m:

  sight    each station sees each of the forklift's two changes so that its
           map's reading of the bay flips, or sees nothing of it, within the
           laser's twelve metres: the observability of the forklift's two
           actions, which the domain takes from here
  route    a hauler whose map is current reaches its drop through t3 and
           through no other bay; a hauler whose map is the shift map's reaches
           it through t1, which is how a stale map sends it the wrong way
  stations every station and every drop is a free cell, reachable from every
           hauler's station over the floor as it is after the forklift

Two maps are written, on pass_through_demo's grid:

  floorplan   the floor plan, with the bays unknown. What the driver plans
              over outside the bays.
  shift_map   the floor plan with the bays as they were at the start of the
              shift, every cell known. What every robot's map starts as.
"""

import argparse
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

FREE, OCCUPIED, UNKNOWN = 0, 100, -1
INFLATION = 0.35


def to_cell(x, y):
    return (int((x - L.GRID_ORIGIN[0]) / L.RESOLUTION),
            int((y - L.GRID_ORIGIN[1]) / L.RESOLUTION))


def centre(c, r):
    return (L.GRID_ORIGIN[0] + (c + 0.5) * L.RESOLUTION,
            L.GRID_ORIGIN[1] + (r + 0.5) * L.RESOLUTION)


def fill(grid, box, value):
    x0, y0, x1, y1 = box
    c0, r0 = to_cell(x0, y0)
    c1, r1 = to_cell(x1, y1)
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, L.GRID_WIDTH - 1), min(r1, L.GRID_HEIGHT - 1)
    grid[r0:r1 + 1, c0:c1 + 1] = value


def plan(blocked=()):
    """The floor with loads in @p blocked, every cell known."""
    grid = np.full((L.GRID_HEIGHT, L.GRID_WIDTH), OCCUPIED, dtype=np.int8)
    fill(grid, (L.HALL_MIN_X, L.HALL_MIN_Y, L.HALL_MAX_X, L.HALL_MAX_Y), FREE)
    for b in L.static_obstacles():
        fill(grid, b, OCCUPIED)
    for t in blocked:
        fill(grid, L.load_box(t), OCCUPIED)
    return grid


def floorplan():
    """The plan with the bays unknown."""
    grid = plan()
    for t in L.BAYS:
        fill(grid, L.bay_box(t), UNKNOWN)
    return grid


def region_cells(bay):
    x0, y0, x1, y1 = L.bay_region(bay)
    out = []
    for r in range(L.GRID_HEIGHT):
        for c in range(L.GRID_WIDTH):
            x, y = centre(c, r)
            if x0 <= x <= x1 and y0 <= y <= y1:
                out.append((c, r))
    return out


def with_robots(grid, but):
    """@p grid with every robot but @p but drawn as a disc."""
    out = grid.copy()
    k = int(np.ceil(L.ROBOT_RADIUS / L.RESOLUTION))
    for a, (x, y, _) in L.ROBOTS.items():
        if a == but:
            continue
        c0, r0 = to_cell(x, y)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if (dr * dr + dc * dc) * L.RESOLUTION ** 2 <= L.ROBOT_RADIUS ** 2:
                    out[r0 + dr, c0 + dc] = OCCUPIED
    return out


def visible(grid, a, cell):
    """True when the ray from @p a reaches the centre of @p cell within the
    laser's range without crossing an occupied cell before it."""
    b = centre(*cell)
    d = np.hypot(b[0] - a[0], b[1] - a[1])
    if d > L.LIDAR_RANGE:
        return False
    n = int(d / (L.RESOLUTION / 3)) + 1
    t = np.arange(n) / n
    xs = a[0] + t * (b[0] - a[0])
    ys = a[1] + t * (b[1] - a[1])
    cs = ((xs - L.GRID_ORIGIN[0]) / L.RESOLUTION).astype(int)
    rs = ((ys - L.GRID_ORIGIN[1]) / L.RESOLUTION).astype(int)
    hit = grid[rs, cs] == OCCUPIED
    hit &= ~((cs == cell[0]) & (rs == cell[1]))
    return not hit.any()


def load_cells(bay):
    """The corridor cells a load in @p bay covers, as the plan rasterises it:
    taken from the grid and not from the box, so that a cell is a load cell
    exactly when the plan marks it occupied."""
    loaded = plan([bay])
    empty = plan()
    return [(c, r) for c, r in region_cells(bay)
            if loaded[r, c] == OCCUPIED and empty[r, c] != OCCUPIED]


# A staged load is read from the first occupied cell, and this many of its
# cells in view leave a margin for the cells a beam grazes.
STAGE_MIN = 15


def sight_lines():
    """For every robot and every bay the forklift changes, what its laser
    would make of the change, read the way its map reads a bay: 'all' when
    the change flips the bay's verdict with a margin, 'none' when no cell
    the change touches is in view, and 'k/n' in between.

    A load staged in a bay is read as soon as any cell of it is occupied, so
    the count is the load's cells in view with the load in place. A load
    taken out is read only when every cell it covered has been observed
    free, since any one left occupied keeps the bay blocked, so the count is
    the covered cells in view with the bay empty. Bays the forklift does not
    touch are 'none' for everyone: nobody's map of them changes."""
    out = {}
    for a, (x, y, _) in L.ROBOTS.items():
        out[a] = {t: 'none' for t in L.BAYS}
        for t, kind in L.CHANGES.items():
            others = [b for b in L.SHIFT_BLOCKED if b != t]
            cells = load_cells(t)
            if kind == 'stage':
                grid = with_robots(plan(others + [t]), a)
                seen = sum(visible(grid, (x, y), c) for c in cells)
                verdict = 'all' if seen >= STAGE_MIN else 'none' if seen == 0 else f'{seen}/{len(cells)}'
            else:
                grid = with_robots(plan(others), a)
                seen = sum(visible(grid, (x, y), c) for c in cells)
                verdict = 'all' if seen == len(cells) else 'none' if seen == 0 \
                    else f'{seen}/{len(cells)}'
            out[a][t] = verdict
    return out


def inflate(grid, metres):
    k = int(np.ceil(metres / L.RESOLUTION))
    occ = grid != FREE
    out = occ.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            if dr * dr + dc * dc <= k * k:
                out |= np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
    return out


def flood(passable, start):
    """Distances in cells from @p start over @p passable, -1 where unreached."""
    dist = np.full(passable.shape, -1, dtype=np.int32)
    c, r = to_cell(*start)
    if not passable[r, c]:
        return dist
    dist[r, c] = 0
    queue = deque([(r, c)])
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < passable.shape[0] and 0 <= nc < passable.shape[1] \
                    and passable[nr, nc] and dist[nr, nc] < 0:
                dist[nr, nc] = dist[r, c] + 1
                queue.append((nr, nc))
    return dist


def route_bays(grid, start, goal):
    """The bays a shortest route from @p start to @p goal over @p grid passes
    through, and its length in metres; None when there is no route."""
    passable = ~inflate(grid, INFLATION)
    dist = flood(passable, goal)
    c, r = to_cell(*start)
    if dist[r, c] < 0:
        return None
    path = [(c, r)]
    while dist[r, c] > 0:
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < dist.shape[0] and 0 <= nc < dist.shape[1] \
                    and dist[nr, nc] == dist[r, c] - 1:
                r, c = nr, nc
                break
        path.append((c, r))
    crossed = []
    for t in L.BAYS:
        x0, y0, x1, y1 = L.bay_box(t)
        if any(x0 <= centre(c, r)[0] <= x1 and y0 <= centre(c, r)[1] <= y1 for c, r in path):
            crossed.append(t)
    return crossed, len(path) * L.RESOLUTION


def believed(agent, sees):
    """The bays a robot's map shows blocked after the forklift, when it has
    seen what its station sees and nothing else."""
    out = set(L.SHIFT_BLOCKED)
    for t, kind in L.CHANGES.items():
        if sees[agent][t] == 'all':
            (out.add if kind == 'stage' else out.discard)(t)
    return sorted(out)


def check():
    problems, facts = [], []
    sees = sight_lines()

    # sight: all or nothing, and the forklift seen by someone.
    for a in L.ROBOTS:
        for t in L.BAYS:
            if sees[a][t] not in ('all', 'none'):
                problems.append(f'{a} sees {sees[a][t]} of the cells of {t}')
    for t in L.CHANGES:
        if not any(sees[a][t] == 'all' for a in L.ROBOTS):
            problems.append(f'nobody sees the change in {t}')
    facts.append('who sees each change, so that its map flips:')
    for t in L.BAYS:
        facts.append(f'  {t}: ' + (' '.join(a for a in L.ROBOTS if sees[a][t] == 'all') or 'nobody'))

    # route: current maps cross t3; the shift map crosses t1.
    after = plan(L.after_forklift())
    for h in L.HAULERS:
        x, y, _ = L.ROBOTS[h]
        start, drop = (x, y), L.DROPS[h]
        got = route_bays(after, start, drop)
        if got is None or got[0] != ['t3']:
            problems.append(f'{h} with a current map: route {got}, expected through t3')
        else:
            facts.append(f'{h} with a current map: {got[1]:.1f} m through t3')
        held = believed(h, sees)
        mine = route_bays(plan(held), start, drop)
        facts.append(f'{h} with its own map (blocked: {" ".join(held)}): ' +
                     (f'{mine[1]:.1f} m through {" ".join(mine[0])}' if mine else 'no route'))
        shift = route_bays(plan(L.SHIFT_BLOCKED), start, drop)
        if shift is None or shift[0] != ['t1']:
            problems.append(f'{h} with the shift map: route {shift}, expected through t1')

    # stations: free, and reachable from every hauler over the floor after.
    passable = ~inflate(after, INFLATION)
    for h in L.HAULERS:
        x, y, _ = L.ROBOTS[h]
        dist = flood(passable, (x, y))
        for name, (px, py) in [(a, L.ROBOTS[a][:2]) for a in L.ROBOTS] + list(L.DROPS.items()):
            c, r = to_cell(px, py)
            if not passable[r, c]:
                problems.append(f'{name} at ({px}, {py}) is inside the inflated plan')
            elif dist[r, c] < 0:
                problems.append(f'{name} is not reachable from {h}')
    facts.append('every station and drop is free and reachable from every hauler')
    return problems, facts, sees


def write(grid, stem):
    """A map_server map, trinary: 0 occupied, 254 free, 205 unknown."""
    os.makedirs(os.path.dirname(os.path.abspath(stem)), exist_ok=True)
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == FREE] = 254
    img[grid == OCCUPIED] = 0
    img = img[::-1]
    with open(stem + '.pgm', 'wb') as fh:
        fh.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        fh.write(img.tobytes())
    with open(stem + '.yaml', 'w') as fh:
        fh.write(f'image: {os.path.basename(stem)}.pgm\nmode: trinary\n'
                 f'resolution: {L.RESOLUTION}\n'
                 f'origin: [{L.GRID_ORIGIN[0]}, {L.GRID_ORIGIN[1]}, 0.0]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def plot(path, sees):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    grid = plan()
    fig, ax = plt.subplots(figsize=(6.2, 9.4))
    img = np.where(grid == OCCUPIED, 0.25, 0.97)
    ext = (L.GRID_ORIGIN[0], L.GRID_ORIGIN[0] + L.GRID_WIDTH * L.RESOLUTION,
           L.GRID_ORIGIN[1], L.GRID_ORIGIN[1] + L.GRID_HEIGHT * L.RESOLUTION)
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', extent=ext)

    shade = {'t1': '#c0392b', 't2': '#7f8c8d', 't3': '#16a085'}
    for t in L.BAYS:
        x0, y0, x1, y1 = L.load_box(t)
        shift = t in L.SHIFT_BLOCKED
        after = t in L.after_forklift()
        style = dict(facecolor=shade[t] if after else 'none', edgecolor=shade[t],
                     linewidth=1.4, linestyle='-' if shift else '--', alpha=0.85)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **style))
        bx, _ = L.bay_centre(t)
        label = {'t1': 't1: staged during the shift', 't2': 't2: loaded',
                 't3': 't3: cleared during the shift'}[t]
        ax.text(bx, -2.6 if t != 't2' else 2.4, label, ha='center', fontsize=7, color=shade[t])

    for a, (x, y, colour) in L.ROBOTS.items():
        for t in L.CHANGES:
            if sees[a][t] == 'all':
                bx, _ = L.bay_centre(t)
                ax.plot([x, bx], [y, 0.0], color=shade[t], lw=0.8, alpha=0.6)
        ax.add_patch(Circle((x, y), 0.45, color=colour, zorder=3))
        ax.text(x, y - 1.1, a + (' (hauls)' if a in L.HAULERS else ''), ha='center',
                fontsize=7, zorder=4)
    for h, (x, y) in L.DROPS.items():
        ax.add_patch(Circle((x, y), L.DROP_RADIUS, fill=False, color=L.ROBOTS[h][2], lw=1.2))
    ax.set_xlim(L.HALL_MIN_X - 0.5, L.HALL_MAX_X + 0.5)
    ax.set_ylim(L.HALL_MIN_Y - 0.5, L.HALL_MAX_Y + 0.5)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--maps', help='write floorplan and shift_map here')
    ap.add_argument('--plot', help='draw the floor here')
    args = ap.parse_args()

    problems, facts, sees = check()
    for f in facts:
        print(f)
    for p in problems:
        print('PROBLEM: ' + p)
    if problems:
        return 1
    if args.maps:
        write(floorplan(), os.path.join(args.maps, 'floorplan'))
        write(plan(L.SHIFT_BLOCKED), os.path.join(args.maps, 'shift_map'))
        print(f'wrote {args.maps}/floorplan and {args.maps}/shift_map')
    if args.plot:
        plot(args.plot, sees)
        print(f'drew {args.plot}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
