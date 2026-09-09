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
Draws the disagreement between a world's declared poses and its saved state.

A Gazebo world may carry a `<state>` block recording where each model actually
is, and Gazebo applies it at load in preference to the pose declared on the
model. Where the two disagree, a floor plan rasterised from the declarations
describes a building the simulator does not load.

The figure is the measurement: every model whose two poses differ by more than
a centimetre, ordered by the magnitude of the difference, on a logarithmic
axis because the differences span three orders.

    plot_state_drift.py --world <path>/warehouse.world --out drift.pdf
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt            # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_world                          # noqa: E402


# The palette of the written reports.
INK, SLATE, MIST = '#1A1E24', '#5C6570', '#E9EBEE'
RULE, STEEL, SIGNAL = '#BCC2C9', '#2F4866', '#A0202D'

# Below this the two poses are the same pose written twice.
TOLERANCE = 0.01


def short(name):
    """`aws_robomaker_warehouse_ClutteringD_01_17` -> `ClutteringD_01_17`."""
    return name.replace('aws_robomaker_warehouse_', '')


def drift(world_path):
    """(name, declared, recorded, distance) for every model the state moves."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')

    declared = {}

    def walk(element, ox, oy):
        for model in element.findall('model'):
            x, y, _ = read_world.pose_of(model)
            declared[model.get('name')] = (ox + x, oy + y)
            walk(model, ox + x, oy + y)

    walk(world, 0.0, 0.0)
    recorded = read_world.recorded_state(world)

    out = []
    for name, (dx, dy) in declared.items():
        if name not in recorded:
            continue
        sx, sy, _ = recorded[name]
        distance = math.dist((dx, dy), (sx, sy))
        if distance > TOLERANCE:
            out.append((name, (dx, dy), (sx, sy), distance))
    out.sort(key=lambda row: row[3])
    return out, len(declared), len(recorded)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=os.path.join(
        here, '..', '..', '..', 'third_party',
        'dynamic_logistics_warehouse', 'worlds', 'warehouse.world'))
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rows, declared, recorded = drift(args.world)
    if not rows:
        raise SystemExit(
            f'{args.world}: no model is placed differently by its <state> '
            f'block, so there is nothing here to draw.')

    names = [short(r[0]) for r in rows]
    distances = [r[3] for r in rows]
    # A floor slab misplaced moves the floor; anything else moves an obstacle.
    slab = ['GroundB' in r[0] for r in rows]

    fig, ax = plt.subplots(figsize=(6.4, 0.30 * len(rows) + 1.0))
    positions = range(len(rows))
    ax.barh(list(positions), distances, height=0.6,
            color=[SIGNAL if is_slab else STEEL for is_slab in slab],
            edgecolor='none', zorder=3)

    for y, (distance, is_slab) in enumerate(zip(distances, slab)):
        ax.text(distance * 1.16, y, f'{distance:.2f}', va='center',
                fontsize=7, color=SIGNAL if is_slab else SLATE, zorder=4)

    ax.set_xscale('log')
    ax.set_xlim(0.1, max(distances) * 3.2)
    ax.set_yticks(list(positions))
    ax.set_yticklabels(names, fontsize=7, color=INK)
    ax.set_xlabel('displacement between the declared pose and the recorded '
                  'state (m, log scale)', fontsize=8, color=INK)
    ax.tick_params(axis='x', labelsize=7, colors=SLATE)
    ax.tick_params(axis='y', length=0)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    ax.spines['bottom'].set_color(RULE)
    ax.grid(axis='x', color=MIST, lw=0.6, zorder=0)
    ax.set_axisbelow(True)

    handles = [plt.Rectangle((0, 0), 1, 1, color=SIGNAL),
               plt.Rectangle((0, 0), 1, 1, color=STEEL)]
    ax.legend(handles, ['floor slab', 'obstacle'], fontsize=7,
              frameon=False, loc='lower right', labelcolor=SLATE)

    fig.tight_layout()
    fig.savefig(args.out)
    print(f'{args.out}: {len(rows)} of {declared} models moved by the state '
          f'block ({recorded} poses recorded); '
          f'{min(distances):.2f}-{max(distances):.2f} m')


if __name__ == '__main__':
    main()
