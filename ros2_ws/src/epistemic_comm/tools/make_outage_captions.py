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
Read the caption track of an outage run off the run's own log.

Timing captions by hand against a recording is how a film comes to assert what
the run did not do. Every line here is matched wherever it occurs and timed by
its own ROS timestamp against the moment the screen capture opened, so a
caption saying the link fell appears on the frame where it fell.

    make_outage_captions.py --log run.log --t0 $(cat raw.t0) --out caps.tsv
"""

import argparse
import re

# (regex, template, once).
EVENTS = [
    (r'\[comm_monitor\]: watching the link between (\w+) and (\w+); it falls '
     r'at t=([\d.]+) s and returns at t=([\d.]+) s',
     'the radio between {0} and {1} is up  ·  it will fall at t={2} s',
     True),
    (r'\[belief_update_\w+\]: (\w+) believes about (\w+); bound is '
     r'([\d.]+)\*dt \+ ([\d.]+) m',
     '{0} tracks {1}  ·  RF-05 admits {2}·dt + {3} m of error',
     True),
    (r'link between (\w+) and (\w+) is DOWN at t=[\d.]+ s',
     'THE LINK FALLS  ·  nothing arrives from {0} any more',
     True),
    (r'link down; propagating (\w+) from \(([-\d.]+), ([-\d.]+)\) under '
     r'v=([-\d.]+) m/s',
     'the belief is propagated from ({1}, {2}) at {3} m/s  ·  the disc is its uncertainty',
     True),
    (r'trace ([\d.]+) m\^2 is past sigma_max\^2 = ([\d.]+) after ([\d.]+) s',
     'past sigma_max after {2} s  ·  where {0} m2 of doubt is too much to believe',
     True),
    (r'link between \w+ and \w+ is UP at t=[\d.]+ s, after ([\d.]+) s of outage',
     'THE LINK RETURNS after {0} s  ·  the belief snaps back to the robot',
     True),
    (r'(\d+) samples over ([\d.]+) s of outage; worst error ([\d.]+) m',
     '{0} samples  ·  worst error {2} m against a bound that reached 13.2 m',
     True),
    (r'a learned (\d+) cells, b learned (\d+), (\d+) in conflict',
     'the two maps are reconciled  ·  {0} and {1} cells learned, {2} in conflict',
     True),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    lines = open(args.log, errors='replace').read().splitlines()
    caps = []
    for pattern, template, once in EVENTS:
        for index, line in enumerate(lines):
            hit = re.search(pattern, line)
            if not hit:
                continue
            stamp = re.search(r'\[(\d{10})\.(\d+)\]', line)
            if not stamp:
                continue
            t = float(f'{stamp.group(1)}.{stamp.group(2)}') - args.t0
            if t < 0:
                continue
            # The line number breaks ties. Several events share a timestamp,
            # and sorting those by their text would put the consequence before
            # the cause.
            caps.append((round(t, 1), index, template.format(*hit.groups())))
            if once:
                break

    # Several events share an instant: the link returns, the criterion is
    # decided and the maps are reconciled all at the same timestamp, because
    # the first causes the other two within the same callback. Dropping all but
    # one would lose the result the film exists to show, so they are shown in
    # sequence instead, each held back to the minimum a reader needs. A caption
    # can therefore trail its event by a second or two; none precedes one, and
    # the order is the order the log has.
    GAP = 1.6
    caps.sort()
    spread = []
    for t, _, text in caps:
        if spread and t - spread[-1][0] < GAP:
            t = spread[-1][0] + GAP
        spread.append((t, text))

    with open(args.out, 'w') as fh:
        for t, text in spread:
            fh.write(f'{round(t, 1)}\t{text}\n')
    for t, text in spread:
        print(f'{t:8.1f}  {text}')
    if not spread:
        raise SystemExit('no events matched; the log is from a different run')


if __name__ == '__main__':
    main()
