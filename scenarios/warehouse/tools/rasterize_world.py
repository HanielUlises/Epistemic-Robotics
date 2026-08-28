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
The floor plan of the RoboticsAcademy multi-robot Amazon warehouse, taken from
the world that exercise runs on rather than drawn to look like it.

The exercise is built on AWS RoboMaker's small warehouse world, whose obstacles
are collision meshes -- the building shell, six rack rows, a tall shelf along
the west wall, and the clutter between them.  This reads those meshes, slices
them at the heights a ground robot can collide with, and rasterises the slice
into the occupancy grid the fixed point runs over.

The two maps AWS ships beside that world are SLAM captures: sparse, noisy, and
missing most of the racks.  Planning on them would make every result an
artefact of whoever drove the robot that day.  The meshes are the ground truth
the simulator itself uses for collision, so a rack the planner drives around is
the rack the laser hits.

Output, all of it checked in so no build step needs this script or the AWS
package:

  ros2_ws/src/warehouse_scenario/maps/aws_small_warehouse.pgm   for rviz/nav2
  ros2_ws/src/warehouse_scenario/maps/aws_small_warehouse.yaml
  ros2_ws/src/warehouse_scenario/src/floorplan.gen.cpp          for the library

Run it again after upgrading the AWS package; the diff is the floor moving.

  python3 scenarios/warehouse/tools/rasterize_world.py [--world-package DIR]
"""

import argparse
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'

# The grid, fixed rather than derived: the origin and resolution are quoted in
# the scenario, in the snapshots and in the C++ header, and a rasterisation
# that silently moved them would move every zone with them.
RESOLUTION = 0.05
ORIGIN_X = -7.00
ORIGIN_Y = -10.50
WIDTH = 281
HEIGHT = 421

# The slice.  Below the floor is the ground plane and above kZHigh are the roof
# and the lamps, neither of which a robot can drive into.
Z_LOW = 0.03
Z_HIGH = 2.00

# Modelled above the robot or under the floor; excluded by name rather than by
# height so that the reason is on the page.
SKIP = ('Roof', 'GroundB', 'Lamp')


def find_world_package(explicit):
    """Where the AWS package is checked out."""
    if explicit:
        return explicit
    for var in ('AWS_SMALL_WAREHOUSE_DIR',):
        if os.environ.get(var):
            return os.environ[var]
    try:
        out = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'aws_robomaker_small_warehouse_world'],
            capture_output=True, text=True, check=True).stdout.strip()
        share = os.path.join(
            out, 'share', 'aws_robomaker_small_warehouse_world')
        if os.path.isdir(share):
            return share
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    raise SystemExit(
        'cannot find aws_robomaker_small_warehouse_world.  Source the '
        'workspace that has it, pass --world-package, or set '
        'AWS_SMALL_WAREHOUSE_DIR.')


def load_collada(path):
    """Triangles of a collision mesh, in metres, in the model frame."""
    root = ET.parse(path).getroot()
    unit = 1.0
    node = root.find(COLLADA + 'asset/' + COLLADA + 'unit')
    if node is not None:
        unit = float(node.get('meter'))

    geometries = {}
    for geometry in root.iter(COLLADA + 'geometry'):
        mesh = geometry.find(COLLADA + 'mesh')
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall(COLLADA + 'source'):
            array = source.find(COLLADA + 'float_array')
            if array is not None:
                sources['#' + source.get('id')] = np.fromstring(
                    array.text, sep=' ')
        vertices = mesh.find(COLLADA + 'vertices')
        if vertices is None:
            continue
        position = None
        for inp in vertices.findall(COLLADA + 'input'):
            if inp.get('semantic') == 'POSITION':
                position = sources.get(inp.get('source'))
        if position is None:
            continue
        points = position.reshape(-1, 3)

        faces = []
        for triangles in mesh.findall(COLLADA + 'triangles'):
            inputs = triangles.findall(COLLADA + 'input')
            stride = max(int(i.get('offset')) for i in inputs) + 1
            offset = next(int(i.get('offset')) for i in inputs
                          if i.get('semantic') == 'VERTEX')
            index = np.fromstring(
                triangles.find(COLLADA + 'p').text, sep=' ', dtype=np.int64)
            faces.append(index.reshape(-1, stride)[:, offset].reshape(-1, 3))
        if faces:
            geometries[geometry.get('id')] = (points, np.vstack(faces))

    out = []
    for node in root.iter(COLLADA + 'node'):
        instance = node.find(COLLADA + 'instance_geometry')
        if instance is None:
            continue
        transform = np.eye(4)
        matrix = node.find(COLLADA + 'matrix')
        if matrix is not None:
            transform = np.fromstring(matrix.text, sep=' ').reshape(4, 4)
        key = instance.get('url').lstrip('#')
        if key not in geometries:
            continue
        points, faces = geometries[key]
        placed = np.c_[points, np.ones(len(points))] @ transform.T
        # The unit applies to the whole scene, node transforms included.
        out.append((placed[:, :3] * unit, faces))
    return out


def world_models(world_path):
    """Every <model> of the world that includes a model:// URI, with its pose.

    Written against the text rather than the element tree because the world
    leaves one desk commented out, and a comment is not a model.
    """
    text = open(world_path).read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    for match in re.finditer(
            r"<model name=['\"]([^'\"]+)['\"]>(.*?)</model>", text, re.S):
        body = match.group(2)
        uri = re.search(r'<uri>model://([^<]+)</uri>', body)
        pose = re.search(r'<pose[^>]*>([^<]+)</pose>', body)
        if uri and pose:
            yield match.group(1), uri.group(1), [
                float(v) for v in pose.group(1).split()]


def rasterise(triangles):
    """Cells any triangle covers.  Faces and edges both, since a rack wall is
    one quad seen edge-on and filling only its interior leaves it open."""
    grid = np.zeros((HEIGHT, WIDTH), np.uint8)
    for tri in triangles:
        c0 = int((tri[:, 0].min() - ORIGIN_X) / RESOLUTION)
        c1 = int((tri[:, 0].max() - ORIGIN_X) / RESOLUTION) + 1
        r0 = int((tri[:, 1].min() - ORIGIN_Y) / RESOLUTION)
        r1 = int((tri[:, 1].max() - ORIGIN_Y) / RESOLUTION) + 1
        c0, c1 = max(0, c0), min(WIDTH, c1)
        r0, r1 = max(0, r0), min(HEIGHT, r1)
        if c1 <= c0 or r1 <= r0:
            continue

        px = ORIGIN_X + (np.arange(c0, c1) + 0.5) * RESOLUTION
        py = ORIGIN_Y + (np.arange(r0, r1) + 0.5) * RESOLUTION
        gx, gy = np.meshgrid(px, py)
        (x1, y1), (x2, y2), (x3, y3) = tri[:, :2]
        det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if abs(det) > 1e-12:
            a = ((y2 - y3) * (gx - x3) + (x3 - x2) * (gy - y3)) / det
            b = ((y3 - y1) * (gx - x3) + (x1 - x3) * (gy - y3)) / det
            inside = (a >= -1e-9) & (b >= -1e-9) & (1 - a - b >= -1e-9)
            grid[r0:r1, c0:c1] |= inside.astype(np.uint8)

        for (ax, ay), (bx, by) in ((tri[0, :2], tri[1, :2]),
                                   (tri[1, :2], tri[2, :2]),
                                   (tri[2, :2], tri[0, :2])):
            steps = int(max(abs(bx - ax), abs(by - ay)) / RESOLUTION * 2) + 2
            t = np.linspace(0.0, 1.0, steps)
            cx = ((ax + (bx - ax) * t - ORIGIN_X) / RESOLUTION).astype(int)
            cy = ((ay + (by - ay) * t - ORIGIN_Y) / RESOLUTION).astype(int)
            ok = (cx >= 0) & (cx < WIDTH) & (cy >= 0) & (cy < HEIGHT)
            grid[cy[ok], cx[ok]] = 1
    return grid


def build(share):
    world = os.path.join(
        share, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')
    if not os.path.exists(world):
        raise SystemExit('no world at ' + world)

    triangles = []
    used = 0
    for name, model, pose in world_models(world):
        if any(tag in model for tag in SKIP):
            continue
        mesh = os.path.join(
            share, 'models', model, 'meshes', model + '_collision.DAE')
        if not os.path.exists(mesh):
            print('  no collision mesh for ' + model, file=sys.stderr)
            continue
        x, y, z = pose[0], pose[1], pose[2]
        yaw = pose[5] if len(pose) > 5 else 0.0
        cos, sin = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])
        for points, faces in load_collada(mesh):
            triangles.append((points @ rotation.T + [x, y, z])[faces])
        used += 1
        print(f'  {name:46s} {model:40s} ({x:6.2f}, {y:6.2f})',
              file=sys.stderr)

    tris = np.vstack(triangles)
    band = (tris[:, :, 2].max(1) > Z_LOW) & (tris[:, :, 2].min(1) < Z_HIGH)
    print(f'  {used} models, {band.sum()} of {len(tris)} triangles in '
          f'[{Z_LOW}, {Z_HIGH}] m', file=sys.stderr)
    return rasterise(tris[band])


def write_pgm(grid, path):
    """Row zero of a PGM is the north edge; row zero of the grid is the south."""
    image = np.where(grid.astype(bool), 0, 254).astype(np.uint8)
    with open(path, 'wb') as out:
        out.write(b'P5\n# the AWS RoboMaker small warehouse, from its collision'
                  b' meshes\n')
        out.write(f'{WIDTH} {HEIGHT}\n255\n'.encode())
        out.write(np.flipud(image).tobytes())


def write_yaml(path, image):
    with open(path, 'w') as out:
        out.write(f'image: {image}\n'
                  f'resolution: {RESOLUTION:.6f}\n'
                  f'origin: [{ORIGIN_X:.6f}, {ORIGIN_Y:.6f}, 0.000000]\n'
                  'negate: 0\n'
                  'occupied_thresh: 0.65\n'
                  'free_thresh: 0.196\n')


def write_cpp(grid, path):
    """The same grid as run lengths, so the library carries the floor with it.

    A 281 x 421 bitmap is 118 kB written out a cell at a time and under 4 kB
    written as runs, and the floor is mostly runs.
    """
    flat = grid.reshape(-1).astype(bool)
    runs = []
    value = False
    count = 0
    for bit in flat:
        if bool(bit) == value:
            count += 1
        else:
            runs.append(count)
            value = bool(bit)
            count = 1
    runs.append(count)

    body = []
    line = ' '
    for run in runs:
        piece = f' {run},'
        if len(line) + len(piece) > 78:
            body.append(line)
            line = ' '
        line += piece
    body.append(line.rstrip(','))

    with open(path, 'w') as out:
        out.write(
            '// Generated by scenarios/warehouse/tools/rasterize_world.py.\n'
            '// Do not edit: this is the AWS RoboMaker small warehouse world,\n'
            '// rasterised from the collision meshes Gazebo itself uses, and\n'
            '// the way to change it is to change the world and run that\n'
            '// script again.\n'
            '//\n'
            f'// {WIDTH} x {HEIGHT} cells at {RESOLUTION} m, origin '
            f'({ORIGIN_X}, {ORIGIN_Y}) m, row 0 at the south edge.\n'
            '// Alternating run lengths over the cells in row-major order,\n'
            '// starting with a run of free cells.\n\n'
            '#include "warehouse_scenario/warehouse.hpp"\n\n'
            'namespace warehouse_scenario\n{\n\n'
            f'// {len(runs)} runs over {len(flat)} cells.\n'
            'const uint32_t kFloorplanRuns[] = {\n')
        out.write('\n'.join(body))
        out.write('\n};\n\n'
                  'const std::size_t kFloorplanRunCount =\n'
                  '  sizeof(kFloorplanRuns) / sizeof(kFloorplanRuns[0]);\n\n'
                  '}  // namespace warehouse_scenario\n')
    return len(runs)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, '..', '..', '..'))
    package = os.path.join(repo, 'ros2_ws', 'src', 'warehouse_scenario')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-package', default=None,
                        help='share directory of '
                             'aws_robomaker_small_warehouse_world')
    parser.add_argument('--out-package', default=package)
    args = parser.parse_args()

    share = find_world_package(args.world_package)
    print('reading ' + share, file=sys.stderr)
    grid = build(share)

    maps = os.path.join(args.out_package, 'maps')
    os.makedirs(maps, exist_ok=True)
    write_pgm(grid, os.path.join(maps, 'aws_small_warehouse.pgm'))
    write_yaml(os.path.join(maps, 'aws_small_warehouse.yaml'),
               'aws_small_warehouse.pgm')
    runs = write_cpp(grid, os.path.join(
        args.out_package, 'src', 'floorplan.gen.cpp'))

    occupied = int(grid.astype(bool).sum())
    print(f'{WIDTH} x {HEIGHT} cells at {RESOLUTION} m: '
          f'{occupied} occupied, {grid.size - occupied} free, {runs} runs',
          file=sys.stderr)


if __name__ == '__main__':
    main()
