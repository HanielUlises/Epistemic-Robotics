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
Draws the navigation graph over the warehouse floor.

The floor comes from the occupancy grid `warehouse_scenario` rasterises from
the world's own collision meshes, and the graph from the nav graph the fleet
adapter is given, so the figure shows the two objects that have to agree.

    floor_figure.py --map ...aws_small_warehouse.yaml \\
                    --graph ...nav_graphs/0.yaml --out docs/img/warehouse-floor.svg
"""

import argparse
import base64
import io
import math
import os

import yaml


def load_grid(path):
    meta = yaml.safe_load(open(path))
    image = os.path.join(os.path.dirname(path), meta['image'])
    with open(image, 'rb') as handle:
        assert handle.readline().strip() == b'P5'
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = map(int, line.split())
        handle.readline()
        pixels = handle.read()
    return meta, width, height, pixels


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--graph', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--rejected', nargs=4, type=float, default=None,
                        metavar=('X1', 'Y1', 'X2', 'Y2'),
                        help='a lane to draw as refused, for the record')
    args = parser.parse_args()

    meta, gw, gh, pixels = load_grid(args.map)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    graph = yaml.safe_load(open(args.graph))
    level = graph['levels']['L1']
    vertices = level['vertices']

    # Metres to figure pixels, with y increasing upward in the world.
    scale = 26.0
    pad = 54
    world_w, world_h = gw * res, gh * res
    W = max(int(world_w * scale) + 2 * pad, 660)
    H = int(world_h * scale) + 2 * pad + 64

    def sx(x): return pad + (x - ox) * scale
    def sy(y): return pad + (world_h - (y - oy)) * scale

    # The occupied cells, as one path. Coarse, since this is a backdrop.
    step = 2
    boxes = []
    for row in range(0, gh, step):
        run = None
        for col in range(0, gw, step):
            occupied = pixels[row * gw + col] < 200
            if occupied and run is None:
                run = col
            elif not occupied and run is not None:
                x0 = ox + run * res
                x1 = ox + col * res
                y1 = oy + (gh - row) * res
                boxes.append((sx(x0), sy(y1), (x1 - x0) * scale, step * res * scale))
                run = None
        if run is not None:
            x0 = ox + run * res
            x1 = ox + gw * res
            y1 = oy + (gh - row) * res
            boxes.append((sx(x0), sy(y1), (x1 - x0) * scale, step * res * scale))

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
        f'width="{W}" height="{H}" font-family="Arial, Helvetica, sans-serif" '
        f'role="img" aria-label="The navigation graph over the warehouse floor">',
        '<style>'
        '.wall{fill:#DCDCDC}'
        '.lane{stroke:#1A52A0;stroke-width:2.4;stroke-linecap:round}'
        '.bad{stroke:#C8102E;stroke-width:2.4;stroke-dasharray:7 5;stroke-linecap:round}'
        '.node{fill:#FFFFFF;stroke:#111111;stroke-width:1.6}'
        '.zone{fill:#FFFFFF;stroke:#C8102E;stroke-width:2.2}'
        '.lbl{font-size:11px;fill:#111111}'
        # Drawn under every label so it stays legible over a lane. paint-order
        # would be tidier and is not honoured by every renderer.
        '.halo{font-size:11px;fill:none;stroke:#FFFFFF;stroke-width:3.5;'
        'stroke-linejoin:round}'
        '.key{font-size:11.5px;fill:#5A5A5A}'
        '.cap{font-size:11.5px;fill:#5A5A5A;font-style:italic}'
        '</style>',
        f'<rect x="0" y="0" width="{W}" height="{H}" fill="#FFFFFF"/>',
    ]
    for x, y, w, h in boxes:
        out.append(f'<rect class="wall" x="{x:.1f}" y="{y:.1f}" '
                   f'width="{max(w,1):.1f}" height="{max(h,1):.1f}"/>')

    if args.rejected:
        x1, y1, x2, y2 = args.rejected
        out.append(f'<line class="bad" x1="{sx(x1):.1f}" y1="{sy(y1):.1f}" '
                   f'x2="{sx(x2):.1f}" y2="{sy(y2):.1f}"/>')

    seen = set()
    for a, b, _ in level['lanes']:
        if (b, a) in seen:
            continue
        seen.add((a, b))
        ax, ay, _ = vertices[a]
        bx, by, _ = vertices[b]
        out.append(f'<line class="lane" x1="{sx(ax):.1f}" y1="{sy(ay):.1f}" '
                   f'x2="{sx(bx):.1f}" y2="{sy(by):.1f}"/>')

    # The zones the domain names, distinguished from the corners the route needs.
    ZONES = {'dock_south', 'corridor', 'dock_north', 'lane', 'bay2', 'bay3'}
    for x, y, props in vertices:
        name = props.get('name', '')
        cls = 'zone' if name in ZONES else 'node'
        r = 7 if name in ZONES else 4.5
        out.append(f'<circle class="{cls}" cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="{r}"/>')
        if name:
            anchor = 'start' if x < 0 else 'end'
            dx = 12 if x < 0 else -12
            for cls in ('halo', 'lbl'):
                out.append(f'<text class="{cls}" x="{sx(x) + dx:.1f}" '
                           f'y="{sy(y) + 4:.1f}" text-anchor="{anchor}">{name}</text>')

    base = H - 40
    out.append(f'<circle class="zone" cx="{pad + 6}" cy="{base - 4}" r="7"/>')
    out.append(f'<text class="key" x="{pad + 20}" y="{base}">zone named by the domain</text>')
    out.append(f'<circle class="node" cx="{pad + 210}" cy="{base - 4}" r="4.5"/>')
    out.append(f'<text class="key" x="{pad + 222}" y="{base}">corner the route requires</text>')
    if args.rejected:
        out.append(f'<line class="bad" x1="{pad + 410}" y1="{base - 4}" '
                   f'x2="{pad + 448}" y2="{base - 4}"/>')
        out.append(f'<text class="key" x="{pad + 456}" y="{base}">lane refused by the check</text>')
    out.append(f'<text class="cap" x="{pad}" y="{H - 26}">'
               'Grey is occupied floor, from the occupancy grid rasterised from the'
               '</text>')
    out.append(f'<text class="cap" x="{pad}" y="{H - 11}">'
               'world\'s own collision meshes.</text>')
    out.append('</svg>')

    open(args.out, 'w').write('\n'.join(out))
    print(f'{len(vertices)} vertices, {len(seen)} undirected lanes -> {args.out}')


if __name__ == '__main__':
    main()
