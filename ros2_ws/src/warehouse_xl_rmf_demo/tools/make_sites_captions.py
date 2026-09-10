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
Reads the caption track of a multi-site run off the mission's own log.

`make_captions.py` does this for the single-site survey, whose events are a
fixed sequence of three. This one has two robots, two sensing actions and a
branch, so the caption track differs between runs of the same mission and
cannot be a fixed list: which site is scanned second, and whether it is scanned
at all, depends on what the first laser returned.

Every event is therefore matched wherever it occurs and timed by its own ROS
timestamp, offset against the moment the screen capture opened. A caption
saying a robot sensed something appears at the frame in which it did. Timing
captions by hand against a recording is how a video comes to assert things the
run did not do.

    make_sites_captions.py --log run.log --t0 $(cat raw.t0) --out caps.tsv
"""

import argparse
import re

# (regex, template, once). `once` keeps only the first match, for events that
# repeat. Everything with a site or a robot in it recurs legitimately and is
# matched every time.
EVENTS = [
    (r'\[survey_sites_mission\]: policy with (\d+) nodes, (\d+) leaves',
     'the planner returns {0} nodes and {1} leaves  ·  838 168 expansions, depth 6',
     True),
    (r'goto: (\w+) -> \S+/(r\d) heading for (\w+)',
     '{0} ({1}) is sent to {2}  ·  a site is reached only through its own lane',
     False),
    (r'\[site_perception\]: (r\d) at (\w+) \(([\d.]+) m from it\), '
     r'nearest return ([\d.]+) m',
     '{0} at {1}, {2} m from it  ·  the laser reads {3} m',
     False),
    (r'applied (scan_\w+) -> (e-scan-\w+): (\d+) worlds, (\d+) designated',
     '{0} sensed {1}  ·  {2} worlds, {3} designated',
     False),
    (r'(\w+) says (e-scan-\w+) on /eplansys/channel/private/(\w+)',
     '{0} tells {2} privately  ·  the observer is not on this channel',
     False),
    (r'applied (relay-\w+): (\d+) worlds, (\d+) designated',
     'private announcement  ·  the model grows to {1} worlds, {2} designated',
     False),
    (r'\[survey_sites_mission\]: mission complete',
     'mission complete',
     True),
    (r'came out as specified: (\d+) formulas',
     'goal holds  ·  {0} formulas checked and two transcripts read',
     True),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--t0', type=float, required=True,
                    help='unix time at which the screen capture opened')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    lines = open(args.log, errors='replace').read().splitlines()
    caps = []
    for pattern, template, once in EVENTS:
        for line in lines:
            hit = re.search(pattern, line)
            if not hit:
                continue
            stamp = re.search(r'\[(\d{10})\.(\d+)\]', line)
            if not stamp:
                continue
            t = float(f'{stamp.group(1)}.{stamp.group(2)}') - args.t0
            if t < 0:
                continue
            caps.append((round(t, 1), template.format(*hit.groups())))
            if once:
                break

    # Two events of the same instant are one caption's worth of screen. The
    # later of the pair is the more informative -- a product update follows the
    # reading that produced it -- so the earlier is dropped.
    caps.sort()
    thinned = []
    for t, text in caps:
        if thinned and t - thinned[-1][0] < 1.5:
            thinned[-1] = (thinned[-1][0], text)
            continue
        thinned.append((t, text))

    with open(args.out, 'w') as fh:
        for t, text in thinned:
            fh.write(f'{t}\t{text}\n')
    for t, text in thinned:
        print(f'{t:8.1f}  {text}')
    if not thinned:
        raise SystemExit('no events matched; the log is from a different run')


if __name__ == '__main__':
    main()
