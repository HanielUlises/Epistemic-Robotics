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

"""Draws the warehouse floor, its roadmap, and optionally a recorded run."""

import argparse
import csv
import collections
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib.patches import Rectangle   # noqa: E402
import yaml                                # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_world                          # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--nav-graph', required=True)
    ap.add_argument('--run')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    floor, obstacles = read_world.read(
        args.world, os.path.join(here, '..', 'config', 'aws_footprints.json'))
    g = yaml.safe_load(open(args.nav_graph))['levels']['L1']
    V, lanes = g['vertices'], g['lanes']

    fig, ax = plt.subplots(figsize=(8.0, 11.0))
    for x0, y0, x1, y1 in floor:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc='#f4f2ee', ec='#ddd8cf', lw=0.6, zorder=0))
    for x0, y0, x1, y1 in obstacles:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc='#c2c8d0', ec='#8f97a3', lw=0.35, zorder=1))

    for a, b, _ in lanes:
        ax.plot([V[a][0], V[b][0]], [V[a][1], V[b][1]],
                color='#2b6cb0', lw=0.7, alpha=0.55, zorder=2)

    for x, y, p in V:
        n = p.get('name', '')
        if p.get('is_charger'):
            ax.scatter([x], [y], s=48, c='#b91c1c', zorder=4, linewidths=0)
        elif n.startswith('aisle_'):
            ax.scatter([x], [y], s=16, c='#d97706', zorder=4, linewidths=0)
        else:
            ax.scatter([x], [y], s=5, c='#1a365d', zorder=3, linewidths=0)

    for name, pts in read_world.actors(args.world):
        if len(pts) > 1:
            xs = [p[0] for p in pts] + [pts[0][0]]
            ys = [p[1] for p in pts] + [pts[0][1]]
            ax.plot(xs, ys, color='#16a34a', lw=0.9, ls=':', alpha=0.75, zorder=2)

    if args.run:
        tracks = collections.defaultdict(list)
        with open(args.run) as fh:
            for row in csv.DictReader(fh):
                tracks[row['robot']].append((float(row['x']), float(row['y'])))
        colours = {'r1': '#dc2626', 'r2': '#7c3aed', 'r3': '#0891b2'}
        for name in sorted(tracks):
            pts = tracks[name]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], lw=2.4,
                    color=colours.get(name, '#111'), zorder=6,
                    solid_capstyle='round', label=name)
        ax.legend(loc='upper left', fontsize=8, framealpha=.9)

    xs = [b[0] for b in floor] + [b[2] for b in floor]
    ys = [b[1] for b in floor] + [b[3] for b in floor]
    ax.set_aspect('equal')
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    aisles = sum(1 for _, _, p in V if p.get('name', '').startswith('aisle_'))
    ax.set_title(f'dynamic logistics warehouse — {max(xs)-min(xs):.0f} × '
                 f'{max(ys)-min(ys):.0f} m, {len(obstacles)} obstacles\n'
                 f'{len(V)} waypoints, {aisles} in aisles, {len(lanes)} directed lanes',
                 fontsize=10)
    ax.grid(alpha=.12, lw=.4)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print(f'{args.out}: {len(V)} waypoints, {len(lanes)} lanes, {len(obstacles)} obstacles')


if __name__ == '__main__':
    main()
