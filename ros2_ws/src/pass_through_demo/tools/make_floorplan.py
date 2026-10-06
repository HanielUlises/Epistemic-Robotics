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
Writes the floor plan every robot is given, and checks it.

    make_floorplan.py --out maps/floorplan        # .pgm and .yaml
    make_floorplan.py --out maps/floorplan --plot docs/floor.png

The plan is the building as common knowledge: walls, pillars, racking and the
staged clutter are drawn in; the interiors of the three pass-through bays are
not, and are written as unknown. That is the one place the plan says it does
not know, and it is the thing the mission is about.

The file is the format nav2's map_server reads -- trinary, occupied black,
free white, unknown the grey between -- so RViz can draw it and the robots'
own nodes can read it with the same parser.

Before anything is written the floor is checked for the properties the
mission depends on: that every robot can reach its bay's mouth over known
floor, that the carrier can reach the dock through each bay once that bay is
known, and that it cannot reach the dock at all while none is. The last is the
knowledge precondition, made geometric: if some way round the block existed,
the carrier would not need to know anything to cross it, and a demonstration
of the precondition would be a demonstration of nothing.
"""

import argparse
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

FREE, OCCUPIED, UNKNOWN = 0, 100, -1

# What the robots' navigation inflates obstacles by: the Waffle's half-width
# and a margin. The check below has to use the same figure, or it certifies
# routes a robot cannot drive.
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
    for t in L.TUNNEL_BAYS:
        fill(grid, L.tunnel_box(t), UNKNOWN)
    return grid


def inflate(grid, metres):
    """Occupied cells grown by `metres`; unknown cells are left as they are."""
    reach = int(np.ceil(metres / L.RESOLUTION))
    occ = grid == OCCUPIED
    grown = occ.copy()
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if dr * dr + dc * dc > reach * reach:
                continue
            grown |= np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
    return grown


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


def check(grid):
    """The properties the mission depends on. Raises on the first that fails."""
    blocked = inflate(grid, INFLATION)
    known_free = (grid == FREE) & ~blocked

    def at(region, point):
        c, r = to_cell(*point)
        return bool(region[r, c])

    problems = []
    carrier = L.ROBOTS['carrier']
    home = (carrier[1], carrier[2])

    # With no bay known, the dock must be out of reach.
    if at(reachable(known_free, home), L.DOCK):
        problems.append('the carrier reaches the dock without crossing a bay: '
                        'there is a way round the block')

    # With each bay known open, it must be in reach, and through that bay.
    for t in sorted(L.TUNNEL_BAYS):
        lifted = grid.copy()
        fill(lifted, L.tunnel_box(t), FREE)
        free = (lifted == FREE) & ~inflate(lifted, INFLATION)
        if not at(reachable(free, home), L.DOCK):
            problems.append(f'with {t} open the carrier still cannot reach the dock')

    # Every scout reaches the mouth of each bay it covers.
    for agent, bays in L.COVERS.items():
        ns, x, y = L.ROBOTS[agent][:3]
        region = reachable(known_free, (x, y))
        for t in bays:
            if not at(region, L.tunnel_mouth(t)):
                problems.append(f'{agent} cannot reach the mouth of {t}')
            if not at(region, L.tunnel_parking(t)):
                problems.append(f'{agent} cannot reach its parking place beside {t}')

    # A load closes its bay to a robot, not merely narrows it.
    for t in sorted(L.TUNNEL_BAYS):
        loaded = grid.copy()
        fill(loaded, L.tunnel_box(t), FREE)
        fill(loaded, L.load_box(t), OCCUPIED)
        free = (loaded == FREE) & ~inflate(loaded, INFLATION)
        south = reachable(free, L.tunnel_mouth(t))
        x, _ = L.tunnel_centre(t)
        if at(south, (x, L.BLOCK_HALF_DEPTH + L.MOUTH_STANDOFF)):
            problems.append(f'a load in {t} leaves a way through it')

    if problems:
        raise SystemExit('floor check failed:\n  ' + '\n  '.join(problems))


def check_meshes(model_root, rasterizer):
    """Every placed model's collision, at laser height, against the box the plan
    draws for it.

    The plan is written from layout.py's footprints, and a footprint is a claim
    about a mesh. The first floor took the AWS shelf's from a table that had its
    axes permuted, and every unit stood a quarter turn from where the plan put
    it; nothing checked, and it was found by a robot stopping against racking
    the plan said was not there. So the meshes are read here, with their node
    transforms and their units, sliced where a laser sees them, turned and
    placed as the world places them, and the extent of the slice has to lie
    inside the box the plan draws, to within a cell.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location('rasterize_world', rasterizer)
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)

    cache = {}

    def slice_of(model, z=0.17):
        if model not in cache:
            path = os.path.join(model_root, model, 'meshes', f'{model}_collision.DAE')
            tris = np.concatenate([pts[faces] for pts, faces in R.load_collada(path)])
            hit = (tris[:, :, 2].min(1) <= z) & (tris[:, :, 2].max(1) >= z)
            cache[model] = tris[hit].reshape(-1, 3)[:, :2]
        return cache[model]

    placed = [(L.SHELF_MODELS[i % 2], x, y, L.SHELF_YAW, L.shelf_box(x, y))
              for i, (x, y) in enumerate(L.block_shelves() + L.storage_shelves())]
    placed += [(m, x, y, yaw, L.footprint(m, x, y, yaw)) for m, x, y, yaw in L.CLUTTER]
    placed += [(L.LOAD_MODEL, *L.tunnel_centre(t), 0.0, L.load_box(t)) for t in L.TUNNEL_BAYS]

    worst = 0.0
    for model, x, y, yaw, box in placed:
        pts = slice_of(model)
        if not len(pts):
            # Nothing of it at laser height. The plan still marks it, which
            # costs a route a detour and cannot put a robot into it.
            print(f'note: {model} has no collision at laser height; the plan keeps it')
            continue
        c, s_ = np.cos(yaw), np.sin(yaw)
        wx = x + c * pts[:, 0] - s_ * pts[:, 1]
        wy = y + s_ * pts[:, 0] + c * pts[:, 1]
        seen = (wx.min(), wy.min(), wx.max(), wy.max())
        # Only a mesh reaching outside its box is a fault: a box larger than
        # the mesh costs a detour, a mesh outside the box is a collision.
        err = max(box[0] - seen[0], box[1] - seen[1], seen[2] - box[2], seen[3] - box[3], 0.0)
        worst = max(worst, err)
        if err > L.RESOLUTION:
            raise SystemExit(
                f'{model} at ({x:.2f}, {y:.2f}): the plan draws {tuple(round(v, 2) for v in box)} '
                f'and its collision at laser height spans {tuple(round(v, 2) for v in seen)}')
    return len(placed), worst


def write(grid, stem):
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == FREE] = 254
    img[grid == OCCUPIED] = 0
    # PGM rows run top to bottom and the grid's run bottom to top.
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
                 # 0.196 and not 0.25. Unknown is written as grey 205, which is
                 # an occupancy of 0.19608; with a free threshold of 0.25 it
                 # reads back as free, and the three bays come back open in
                 # every robot's floor plan. 0.196 is the threshold map savers
                 # write for exactly this reason.
                 f'free_thresh: 0.196\n')


def plot(grid, path, open_bay=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    fig, ax = plt.subplots(figsize=(7.6, 11))
    shade = np.full(grid.shape + (3,), 1.0)
    shade[grid == OCCUPIED] = (0.55, 0.58, 0.62)
    shade[grid == UNKNOWN] = (0.93, 0.80, 0.55)
    ext = (L.GRID_ORIGIN[0], L.GRID_ORIGIN[0] + L.GRID_WIDTH * L.RESOLUTION,
           L.GRID_ORIGIN[1], L.GRID_ORIGIN[1] + L.GRID_HEIGHT * L.RESOLUTION)
    ax.imshow(shade, origin='lower', extent=ext, interpolation='nearest')

    for t in sorted(L.TUNNEL_BAYS):
        x, y = L.tunnel_centre(t)
        ax.text(x, y, t, ha='center', va='center', fontsize=11, weight='bold')
        rx0, ry0, rx1, ry1 = L.tunnel_region(t)
        ax.add_patch(Rectangle((rx0, ry0), rx1 - rx0, ry1 - ry0, fill=False,
                               ec=(0.6, 0.35, 0.05), lw=0.8, ls='--'))
        mx, my = L.tunnel_mouth(t)
        ax.plot(mx, my, marker='^', color=(0.6, 0.35, 0.05), ms=7)
        if open_bay and t != open_bay:
            lx0, ly0, lx1, ly1 = L.load_box(t)
            ax.add_patch(Rectangle((lx0, ly0), lx1 - lx0, ly1 - ly0,
                                   fc=(0.55, 0.2, 0.1), alpha=0.8))

    for agent, (ns, x, y, rng, _, rgb) in L.ROBOTS.items():
        ax.add_patch(Circle((x, y), 0.35, color=rgb))
        ax.add_patch(Circle((x, y), rng, fill=False, ec=rgb, lw=0.6, ls=':'))
        ax.text(x + 0.6, y + 0.6, f'{agent} ({ns})', color=rgb, fontsize=9,
                weight='bold')
    ax.add_patch(Circle(L.DOCK, L.DOCK_RADIUS, fill=False, ec='k', lw=1.2))
    ax.text(L.DOCK[0] + 0.9, L.DOCK[1], 'dock', fontsize=9)

    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('pass-through floor, 30 x 50 m: the floor plan every robot holds\n'
                 'amber = left unknown in the plan (the three bays)', fontsize=10)
    fig.savefig(path, dpi=110, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='path stem for .pgm and .yaml')
    ap.add_argument('--plot', help='also draw the plan to this PNG')
    ap.add_argument('--open', help='with --plot, draw the loads for this world')
    ap.add_argument('--models', help='the AWS models directory, to check the plan against '
                                     'the collision meshes')
    ap.add_argument('--rasterizer', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'scenarios',
        'warehouse', 'tools', 'rasterize_world.py'),
        help="the warehouse scenario's mesh reader, which applies node transforms and units")
    args = ap.parse_args()

    grid = floorplan()
    check(grid)
    if args.models:
        n, worst = check_meshes(args.models, args.rasterizer)
        print(f'{n} placed models agree with their collision meshes to {worst:.3f} m')
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write(grid, args.out)
    known = int((grid == FREE).sum())
    unknown = int((grid == UNKNOWN).sum())
    print(f'{args.out}.pgm: {grid.shape[1]} x {grid.shape[0]} cells at '
          f'{L.RESOLUTION} m, {known} free, {unknown} unknown (the bays); '
          'floor check passed')
    if args.plot:
        plot(grid, args.plot, args.open)
        print(f'{args.plot}')


if __name__ == '__main__':
    main()
