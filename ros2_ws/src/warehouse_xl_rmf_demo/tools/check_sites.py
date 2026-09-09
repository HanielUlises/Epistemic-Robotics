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
Decides which waypoints can be survey sites, and where their pallet may stand.

A site in the multi-site domain is a place a robot is sent to look at, so it
has to satisfy three things at once, and none of them is a matter of taste.

  1. The waypoint must read *open* to a laser standing on it. The sensing
     action reports contamination when the nearest finite return falls below a
     threshold, so a waypoint with a rack 0.6 m away reports contamination in
     every world, including the ones with nothing there. This is the failure
     mode that makes a green run meaningless, and it is measured here rather
     than discovered in a recording.

  2. The pallet -- the object that makes one world differ from another -- must
     stand on floor, clear of the racks, and clear of every lane the fleet may
     drive. The last of those is the defect the single-site demo shipped and
     then found the hard way: a prop overlapping a lane by 20 cm held a robot
     motionless while RMF reported its task underway and nothing anywhere
     reported a collision. That check is this file.

  3. The nearest face of the pallet must fall unambiguously on the near side of
     the threshold, and the empty waypoint unambiguously on the far side. A
     site where the two readings straddle the threshold by centimetres is a
     site whose branch is decided by where the fleet happened to stop.

Everything is computed from the world's own collision extents -- the same
`config/aws_footprints.json` the navigation graph is rasterised from -- so the
numbers here and the numbers the graph was built on cannot disagree.

How well this predicts what a laser returns is worth stating in both
directions, because it is not uniform.

The pallet face is predicted well: 0.35 m here against 0.32-0.34 m measured,
because the pallet is a box whose height the model and the laser agree on.

The empty-world reading is not. The model says 1.71 m at `a06` where the laser
returned 2.25 m, and 1.48 m at `a17` where the laser returned 0.97 and 1.08 m. Both differences have the same source: this is a two-dimensional model
and the laser is a plane at 0.26 m, so an obstacle shorter than that is real
here and invisible there; and the nine actors are excluded here deliberately
and entirely visible there.

So what this file is good for is refusing a site rather than predicting a
reading. It admits nothing with less than THRESHOLD + MARGIN of clearance, and
that margin is what absorbs half a metre of disagreement.

    check_sites.py --world <path>/warehouse.world --graph maps/nav_graphs/0.yaml
    check_sites.py ... --site a17 --site a31 --site a06    # just these, and fail
                                                           # if any is refused
"""

import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_world  # noqa: E402


# The sensed object, as `with_camera.py --pallet` builds it: a 0.9 x 0.5 x 0.9
# box. Explicit geometry rather than an AWS model, because the asset pack this
# world ships carries absolute mesh paths from its author's machine and an
# included model can load with no collision at all.
PALLET = (0.9, 0.5)

# How far from the waypoint the pallet centre stands. At this offset, with the
# long axis pointing at the waypoint, the nearest face is 0.35 m away -- which
# is the geometry the single-site demo measured 0.32 m on.
PALLET_OFFSET = 0.80

# What the perception node compares the nearest return against.
THRESHOLD = 0.70

# How much of a margin either side of the threshold is enough. The single-site
# demo ran with 0.18 m below and 0.19 m above; a site is refused here if it
# offers less than this, since below it the branch is decided by where the
# fleet stopped rather than by what is in the aisle.
MARGIN = 0.15

# A robot's disc, from the fleet config, and how much room past it a lane needs
# for the pallet not to be in the way.
ROBOT_RADIUS = 0.25
LANE_MARGIN = 0.10

# Where the pallet may be tried, and how the box is turned. The long axis
# always points at the waypoint, so whichever side is used the nearest face is
# the same distance away and the sensed reading does not depend on the choice.
DIRECTIONS = {
    'west': ((-1, 0), (PALLET[0] / 2, PALLET[1] / 2)),
    'east': ((1, 0), (PALLET[0] / 2, PALLET[1] / 2)),
    'south': ((0, -1), (PALLET[1] / 2, PALLET[0] / 2)),
    'north': ((0, 1), (PALLET[1] / 2, PALLET[0] / 2)),
}


def box_gap(a, b):
    """Separation between two axis-aligned boxes. Negative means they overlap."""
    dx = max(a[0] - b[2], b[0] - a[2])
    dy = max(a[1] - b[3], b[1] - a[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def point_gap(px, py, box):
    return box_gap((px, py, px, py), box)


def box_segment_gap(box, a, b, samples=64):
    """Closest approach of an axis-aligned box to a segment.

    Sampled rather than solved, and screened first. The screen is what makes it
    affordable: surveying the whole floor is every aisle against every lane,
    which is four figures times four figures times the samples, and at 400
    samples a survey took over two minutes. Almost every pair is nowhere near,
    and the triangle inequality settles those without sampling anything.
    """
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    reach = math.hypot(box[2] - box[0], box[3] - box[1]) / 2

    # Distance from the box's centre to the segment, exactly. Anything further
    # than that minus the box's own reach cannot be closer than the difference,
    # so a segment well away from the box is dismissed in a few operations.
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx or dy:
        t = ((cx - a[0]) * dx + (cy - a[1]) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
    else:
        t = 0.0
    centre_gap = math.hypot(cx - (a[0] + t * dx), cy - (a[1] + t * dy))
    if centre_gap - reach > ROBOT_RADIUS + LANE_MARGIN:
        return centre_gap - reach

    closest = math.inf
    for i in range(samples + 1):
        s = i / samples
        px = a[0] + s * (b[0] - a[0])
        py = a[1] + s * (b[1] - a[1])
        closest = min(closest, point_gap(px, py, box))
        if closest == 0.0:
            break
    return closest


class Floor:
    def __init__(self, world, footprints):
        self.tiles, self.obstacles = read_world.read(world, footprints)

    def open_reading(self, px, py):
        """What a laser standing at (px, py) returns in a world with no pallet."""
        return min(point_gap(px, py, o) for o in self.obstacles)

    def on_floor(self, box):
        return any(t[0] <= box[0] and t[1] <= box[1] and
                   box[2] <= t[2] and box[3] <= t[3] for t in self.tiles)

    def rack_gap(self, box):
        return min(box_gap(box, o) for o in self.obstacles)


def pallet_box(sx, sy, direction):
    (ux, uy), (hx, hy) = DIRECTIONS[direction]
    px, py = sx + ux * PALLET_OFFSET, sy + uy * PALLET_OFFSET
    return (px, py), (px - hx, py - hy, px + hx, py + hy)


def nearest_lane(box, vertices, lanes):
    closest, which = math.inf, None
    for a, b, _ in lanes:
        gap = box_segment_gap(
            box, (vertices[a][0], vertices[a][1]), (vertices[b][0], vertices[b][1]))
        if gap < closest:
            closest, which = gap, (vertices[a][2].get('name', f'#{a}'),
                                   vertices[b][2].get('name', f'#{b}'))
    return closest, which


def assess(floor, vertices, lanes, index):
    """Everything measurable about one candidate waypoint."""
    sx, sy = vertices[index][0], vertices[index][1]
    name = vertices[index][2].get('name', f'#{index}')
    empty = floor.open_reading(sx, sy)

    report = {'name': name, 'x': sx, 'y': sy, 'empty': empty, 'placements': [],
              'rejected': [], 'refusals': []}

    if empty < THRESHOLD + MARGIN:
        report['refusals'].append(
            f'a laser at the waypoint reads {empty:.2f} m in an empty world, '
            f'which is not {MARGIN:.2f} m clear of the {THRESHOLD:.2f} m '
            f'threshold: this waypoint reports contamination in every world')

    for direction in DIRECTIONS:
        (px, py), box = pallet_box(sx, sy, direction)
        face = point_gap(sx, sy, box)
        why = []
        if not floor.on_floor(box):
            why.append('off the floor slabs')
        rack = floor.rack_gap(box)
        if rack < 0.10:
            why.append(f'{"overlaps a rack" if rack < 0 else "within 0.10 m of a rack"}'
                       f' ({rack:+.2f} m)')
        lane, which = nearest_lane(box, vertices, lanes)
        if lane < ROBOT_RADIUS + LANE_MARGIN - 1e-6:
            why.append(f'leaves {lane:.2f} m beside lane {which[0]}->{which[1]}, '
                       f'and a robot needs {ROBOT_RADIUS + LANE_MARGIN:.2f} m')
        if face > THRESHOLD - MARGIN:
            why.append(f'its near face reads {face:.2f} m, not {MARGIN:.2f} m '
                       f'clear of the threshold')

        entry = {'direction': direction, 'centre': (px, py), 'face': face,
                 'rack': rack, 'lane': lane, 'lane_between': which, 'why': why}
        if why:
            report['rejected'].append(f'pallet {direction}: ' + '; '.join(why))
        else:
            report['placements'].append(entry)

    # One workable side is all a site needs. Which side is not a property of
    # the site but of the racks around it, and the aisles of this floor run in
    # both directions, so requiring a particular one would refuse half the
    # floor for no reason a robot can tell.
    if not report['placements']:
        report['refusals'].append(
            'no side admits the pallet: ' + '; '.join(report['rejected']))

    return report


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=os.path.join(
        here, '..', '..', '..', 'third_party',
        'dynamic_logistics_warehouse', 'worlds', 'warehouse.world'))
    ap.add_argument('--graph', default=os.path.join(
        here, '..', 'maps', 'nav_graphs', '0.yaml'))
    ap.add_argument('--footprints', default=os.path.join(
        here, '..', 'config', 'aws_footprints.json'))
    ap.add_argument('--site', action='append', default=[],
                    help='a waypoint that must pass. Given any of these, the '
                         'survey is skipped and the exit status is what the '
                         'named sites did.')
    args = ap.parse_args()

    floor = Floor(args.world, args.footprints)
    graph = yaml.safe_load(open(args.graph))
    level = graph['levels'][next(iter(graph['levels']))]
    vertices, lanes = level['vertices'], level['lanes']
    by_name = {v[2].get('name'): i for i, v in enumerate(vertices)}

    if args.site:
        failed = 0
        for name in args.site:
            if name not in by_name:
                print(f'{name}: no such waypoint in {args.graph}')
                failed += 1
                continue
            report = assess(floor, vertices, lanes, by_name[name])
            ok = not report['refusals']
            print(f'{name} at ({report["x"]:.2f}, {report["y"]:.2f}): '
                  f'{"ok" if ok else "REFUSED"}')
            print(f'    empty world reads {report["empty"]:.2f} m '
                  f'(threshold {THRESHOLD:.2f})')
            for p in report['placements']:
                print(f'    pallet {p["direction"]:>5} at '
                      f'({p["centre"][0]:.2f}, {p["centre"][1]:.2f}): '
                      f'reads {p["face"]:.2f} m, {p["rack"]:.2f} m off the racks, '
                      f'{p["lane"]:.2f} m off lane '
                      f'{p["lane_between"][0]}->{p["lane_between"][1]}')
            for r in report['rejected']:
                print(f'    not used: {r}')
            for r in report['refusals']:
                print(f'    REFUSED: {r}')
            failed += 0 if ok else 1
        return 1 if failed else 0

    # No sites named: survey every aisle and say which could serve.
    candidates = [i for i, v in enumerate(vertices)
                  if v[2].get('name', '').startswith(('aisle', 'a0', 'a1', 'a2', 'a3'))]
    usable = []
    for i in candidates:
        report = assess(floor, vertices, lanes, i)
        if not report['refusals']:
            usable.append(report)

    print(f'{len(usable)} of {len(candidates)} aisle waypoints can serve as a site')
    print(f'{"name":>10} {"x":>7} {"y":>7} {"empty":>7}   pallet')
    for r in usable:
        p = r['placements'][0]
        print(f'{r["name"]:>10} {r["x"]:7.2f} {r["y"]:7.2f} {r["empty"]:7.2f}   '
              f'{p["direction"]:>5} at ({p["centre"][0]:6.2f},{p["centre"][1]:6.2f}), '
              f'reads {p["face"]:.2f} m, {p["lane"]:.2f} m off the nearest lane')
    return 0


if __name__ == '__main__':
    sys.exit(main())
