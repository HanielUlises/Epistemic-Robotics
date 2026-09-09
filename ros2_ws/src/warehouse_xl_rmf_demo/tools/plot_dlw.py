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
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import Rectangle   # noqa: E402
import yaml                                # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_world                          # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--nav-graph', required=True)
    ap.add_argument('--run')
    ap.add_argument('--site', action='append', default=[], metavar='NAME',
                    help='a survey site to mark and label. These are pinned '
                         'waypoints, so naming them here cannot pick out a '
                         'different place than the task map does.')
    ap.add_argument('--pallet', action='append', default=[], metavar='X,Y',
                    help='where a sensed pallet stands in one of the world '
                         'variants. Drawn so the figure shows what the laser '
                         'is looking for and how little room it has.')
    ap.add_argument('--paper', action='store_true',
                    help='draw for the report: the palette of the written '
                         'reports, and no title, since a figure in a paper '
                         'carries its caption below it and repeating it inside '
                         'the axes prints the same sentence twice.')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    # One palette for the pages and one for the reports. The reports are set in
    # ink, slate, mist and a single signal red; a figure carrying web blues and
    # ambers into them reads as an import.
    if args.paper:
        C = {'floor': '#FAFAFB', 'floor_edge': '#D5D9DE',
             'obstacle': '#CDD3DA', 'obstacle_edge': '#96A0AB',
             'lane': '#2F4866', 'charger': '#A0202D', 'aisle': '#5C6570',
             'plain': '#1A1E24', 'actor': '#96A0AB',
             'site': '#A0202D', 'site_text': '#A0202D', 'pallet': '#1A1E24'}
    else:
        C = {'floor': '#f4f2ee', 'floor_edge': '#ddd8cf',
             'obstacle': '#c2c8d0', 'obstacle_edge': '#8f97a3',
             'lane': '#2b6cb0', 'charger': '#b91c1c', 'aisle': '#d97706',
             'plain': '#1a365d', 'actor': '#16a34a',
             'site': '#b45309', 'site_text': '#7c2d12', 'pallet': '#7c2d12'}

    here = os.path.dirname(os.path.abspath(__file__))
    floor, obstacles = read_world.read(
        args.world, os.path.join(here, '..', 'config', 'aws_footprints.json'))
    g = yaml.safe_load(open(args.nav_graph))['levels']['L1']
    V, lanes = g['vertices'], g['lanes']

    fig, ax = plt.subplots(figsize=(8.0, 11.0))
    for x0, y0, x1, y1 in floor:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc=C['floor'], ec=C['floor_edge'], lw=0.6,
                               zorder=0))
    for x0, y0, x1, y1 in obstacles:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc=C['obstacle'], ec=C['obstacle_edge'], lw=0.35,
                               zorder=1))

    # One collection, not one call per lane. At 1 870 directed lanes a
    # per-lane plot writes 1 870 separate elements into an SVG, which is half a
    # megabyte and enough to hang a browser tab that has to lay it out. A
    # LineCollection is one element and draws the same thing.
    lane_art = LineCollection(
        [[(V[a][0], V[a][1]), (V[b][0], V[b][1])] for a, b, _ in lanes],
        colors=C['lane'], linewidths=0.45 if args.paper else 0.7,
        alpha=0.75 if args.paper else 0.55, zorder=2)
    # Rasterised for the web and left vector for print. Grouping the lanes into
    # one collection is not enough on its own for SVG -- matplotlib still
    # writes a path per segment, and at 1 870 of them the file is 430 kB and a
    # browser tab laying it out goes unresponsive; rasterising the one dense
    # layer puts it in as a single image and leaves the labels vector. A PDF
    # viewer has no such trouble, and a rasterised layer there loses the lanes
    # entirely: at 0.45 pt they fall below one pixel of the raster.
    lane_art.set_rasterized(args.out.lower().endswith('.svg'))
    ax.add_collection(lane_art)

    # Three scatters rather than one per waypoint, for the same reason.
    groups = {'charger': ([], [], 48, C['charger'], 4),
              'aisle': ([], [], 16, C['aisle'], 4),
              'plain': ([], [], 5, C['plain'], 3)}
    for x, y, p in V:
        n = p.get('name', '')
        key = ('charger' if p.get('is_charger')
               else 'aisle' if n.startswith('aisle_') else 'plain')
        groups[key][0].append(x)
        groups[key][1].append(y)
    for xs_, ys_, size, colour, z in groups.values():
        if xs_:
            ax.scatter(xs_, ys_, s=size, c=colour, zorder=z, linewidths=0)

    # The survey sites, drawn over everything: they are the subject of the
    # figure and the rest is the floor they stand on.
    by_name = {p.get('name'): (x, y) for x, y, p in V}
    for name in args.site:
        if name not in by_name:
            raise SystemExit(f'{name}: no such waypoint in {args.nav_graph}')
        x, y = by_name[name]
        ax.scatter([x], [y], s=150, facecolors='none', edgecolors=C['site'],
                   linewidths=2.0, zorder=7)
        ax.annotate(name, (x, y), textcoords='offset points', xytext=(9, 6),
                    fontsize=9, weight='bold', color=C['site_text'], zorder=8)

    for spec in args.pallet:
        px, py = (float(v) for v in spec.split(','))
        ax.scatter([px], [py], s=70, marker='s', c=C['pallet'], zorder=7,
                   linewidths=0)

    for name, pts in read_world.actors(args.world):
        if len(pts) > 1:
            xs = [p[0] for p in pts] + [pts[0][0]]
            ys = [p[1] for p in pts] + [pts[0][1]]
            ax.plot(xs, ys, color=C['actor'], lw=0.9, ls=':', alpha=0.9, zorder=2)

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
    # Aisle waypoints, counting the pinned sites: pinning a site overrides the
    # aisle name it would otherwise have carried, so counting the `aisle_`
    # prefix alone loses one waypoint per site and reports 32 where there are
    # 35.
    aisles = sum(1 for _, _, p in V
                 if p.get('name', '').startswith('aisle_')) + len(args.site)
    if not args.paper:
        ax.set_title(f'dynamic logistics warehouse — {max(xs)-min(xs):.0f} × '
                     f'{max(ys)-min(ys):.0f} m, {len(obstacles)} obstacles\n'
                     f'{len(V)} waypoints, {aisles} in aisles, '
                     f'{len(lanes)} directed lanes', fontsize=10)
    ax.grid(alpha=.12, lw=.4)
    fig.tight_layout()
    fig.savefig(args.out, dpi=200)
    print(f'{args.out}: {len(V)} waypoints, {len(lanes)} lanes, {len(obstacles)} obstacles')


if __name__ == '__main__':
    main()
