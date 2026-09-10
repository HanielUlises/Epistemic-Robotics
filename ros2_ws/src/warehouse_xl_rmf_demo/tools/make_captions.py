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
Reads the caption track off the mission's own log.

Every line the nodes emit carries a ROS timestamp. Subtracting the moment the
screen capture opened turns those into offsets into the video, so a caption
saying the robot sensed something appears at the frame in which it did. The
alternative is to time captions by hand against a recording, which is how a
video comes to assert things the run did not do.

    make_captions.py --log run.log --t0 $(cat rec_t0) --out captions.tsv
"""

import argparse
import re

# Each entry is (regex over a log line, caption). The first match of each wins,
# because the interesting event is the first time it happens: the bridge
# repeats its websocket tally every ten seconds and the executor re-reports a
# policy node it is still waiting on.
EVENTS = [
    (r'robot is up; request sent',
     'goto-site_relay  —  the scout is dispatched across the floor'),
    (r'heading for a07',
     'scan_relay  —  an aisle cannot be seen into from outside it'),
    (r'at the site \(([\d.]+) m from it\), nearest return ([\d.]+) m',
     'inside the site, {0} m from it  —  the laser reads {1} m'),
    (r'applied scan_relay -> (e-scan-\w+): (\d+) worlds, (\d+) designated',
     '{0} sensed  —  product update leaves {1} worlds, {2} designated'),
    (r'applied relay-(\w+)_relay_scout',
     'relay-{0}_relay_scout  —  private channel and no word to the observer'),
    (r'came out as specified',
     'goal holds  —  three formulas checked, two transcripts read'),
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
    for pattern, template in EVENTS:
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
            break

    caps.sort()
    with open(args.out, 'w') as fh:
        for t, text in caps:
            fh.write(f'{t}\t{text}\n')
    for t, text in caps:
        print(f'{t:7.1f}  {text}')
    if not caps:
        raise SystemExit('no events matched; the log is from a different run')


if __name__ == '__main__':
    main()
