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

"""Draws the floor: the racks, the pillars and the graph the fleet routes over."""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt              # noqa: E402
from matplotlib.patches import Rectangle     # noqa: E402
import yaml                                  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L                           # noqa: E402
from make_nav_graph import obstacles         # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nav-graph', default='maps/nav_graphs/0.yaml')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    g = yaml.safe_load(open(args.nav_graph))['levels']['L1']
    V, lanes = g['vertices'], g['lanes']

    fig, ax = plt.subplots(figsize=(7.2, 11.0))
    ax.add_patch(Rectangle((L.HALL_MIN_X, L.HALL_MIN_Y),
                           L.HALL_MAX_X - L.HALL_MIN_X,
                           L.HALL_MAX_Y - L.HALL_MIN_Y,
                           fill=False, lw=2.0, ec='#333'))
    for x0, y0, x1, y1 in obstacles():
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc='#c8cdd4', ec='#8b929c', lw=0.4))

    for a, b, _ in lanes:
        ax.plot([V[a][0], V[b][0]], [V[a][1], V[b][1]],
                color='#2b6cb0', lw=0.6, alpha=0.5, zorder=2)

    style = {
        'lane': ('#1a365d', 9), 'mouth': ('#d97706', 7),
        'dock': ('#047857', 34), 'charger': ('#b91c1c', 34),
    }
    for x, y, p in V:
        n = p.get('name', '')
        if n.startswith('lane'):
            c, s = style['lane']
        elif n.endswith('_mouth'):
            c, s = style['mouth']
        elif n.startswith('dock'):
            c, s = style['dock']
        elif n.startswith('charger'):
            c, s = style['charger']
        else:
            c, s = '#4a5568', 5
        ax.scatter([x], [y], s=s, c=c, zorder=3, linewidths=0)

    ax.set_aspect('equal')
    ax.set_xlim(L.HALL_MIN_X - 1, L.HALL_MAX_X + 1)
    ax.set_ylim(L.HALL_MIN_Y - 1, L.HALL_MAX_Y + 1)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'warehouse_xl — {L.HALL_MAX_X - L.HALL_MIN_X:.0f} x '
                 f'{L.HALL_MAX_Y - L.HALL_MIN_Y:.0f} m, '
                 f'{2 * len(L.aisles())} aisles, {len(V)} waypoints')
    ax.grid(alpha=0.15, lw=0.4)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f'{args.out}: {len(V)} waypoints, {len(lanes)} directed lanes')


if __name__ == '__main__':
    main()
