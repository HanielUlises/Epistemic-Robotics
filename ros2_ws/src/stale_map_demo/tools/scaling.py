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
The stale-maps domain for fleets of many robots, without a simulator.

    scaling.py --robots 4 8 16 32 64 --seeds 1 2 3 --out /tmp/stale_maps_scaling

For each size and seed a random fleet (tools/stale_maps.py: each robot sees
each changed bay with probability 0.3, a third of the fleet hauls) is
grounded by plank and solved by the planner the executor uses, on the radio
floor and the resync floor, with consistent beliefs required. Each policy is
replayed by tools/trace.py, which checks the goal at its leaf and counts the
stale entries the policy leaves. The table it prints:

  N            robots, H haulers
  saw          robots whose station sees t1, and t3; with nobody at t3 no
               robot can come to know it is open, and there is no policy
  stale        entries of the robots' maps the forklift left false
  radio        maps of a bay the radio policy sends, and the planner's time
  resync       the same for the resync floor's goal, every map current
  broadcast    maps of a bay every robot sending every other sends: N(N-1)B

The radio policy sends a map only to a hauler that needs it; the resync
policy sends one per stale entry, which is the least any policy can send,
since a message repairs one robot's reading of one bay.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from stale_maps import random_fleet, write  # noqa: E402

PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER',
                         os.path.expanduser('~/eplansys_ws/install/aletheia/bin/epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
MAPS = os.path.join(HERE, '..', 'epddl', 'maps.epddl')


def solve(out, floor, timeout):
    task_dir = os.path.join(out, floor)
    subprocess.run([PLANK, 'export', '-d', os.path.join(out, 'stale-maps.epddl'),
                    '-p', os.path.join(out, f'{floor}.epddl'), '-l', LIB, MAPS, '-o', task_dir],
                   check=True, capture_output=True)
    task = os.path.join(task_dir, f'{floor}.json')
    plan = os.path.join(task_dir, 'plan.json')
    started = time.time()
    run = subprocess.run([PLANNER, '--task', task, '--plan', plan, '--timeout', str(timeout),
                          '--consistent-beliefs'], capture_output=True, text=True)
    seconds = time.time() - started
    if 'Solution found' not in run.stdout + run.stderr:
        return {'solved': False, 'seconds': seconds}
    steps = os.path.join(task_dir, 'steps.json')
    trace = subprocess.run([sys.executable, os.path.join(HERE, 'trace.py'), '--task', task,
                            '--plan', plan, '--json', steps], capture_output=True, text=True)
    sends = len(re.findall(r'^\s*send-(open|blocked)_', trace.stdout, re.M))
    left = json.load(open(steps))[-1]['stale']
    return {'solved': True, 'seconds': seconds, 'sends': sends, 'stale_left': left,
            'goal': 'goal holds' in trace.stdout and trace.returncode == 0}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', type=int, nargs='+', default=[4, 8, 16, 32])
    ap.add_argument('--seeds', type=int, nargs='+', default=[1, 2, 3])
    ap.add_argument('--timeout', type=int, default=300)
    ap.add_argument('--out', default='/tmp/stale_maps_scaling')
    args = ap.parse_args()

    rows = []
    print(f'{"N":>3} {"H":>3} {"seed":>4} {"saw":>5} {"stale":>5} | {"radio":>12} | '
          f'{"resync":>14} | {"broadcast":>9}')
    for n in args.robots:
        for seed in args.seeds:
            fleet = random_fleet(n, max(1, n // 3), seed)
            out = os.path.join(args.out, f'n{n}_s{seed}')
            write(out, fleet, floors=('radio', 'resync'))
            stale = len(fleet.stale())
            radio = solve(out, 'radio', args.timeout)
            resync = solve(out, 'resync', args.timeout)
            saw = f'{len(fleet.sees["t1"])}/{len(fleet.sees["t3"])}'
            row = {'robots': n, 'haulers': len(fleet.haulers), 'seed': seed, 'stale': stale,
                   'saw': {t: len(fleet.sees[t]) for t in ('t1', 't3')},
                   'radio': radio, 'resync': resync, 'broadcast': n * (n - 1) * len(fleet.bays)}
            rows.append(row)

            def cell(r):
                if not r['solved']:
                    return f'none {r["seconds"]:5.1f}s'
                return f'{r["sends"]:3d} {r["seconds"]:5.2f}s' + ('' if r['goal'] else ' !')
            print(f'{n:3d} {row["haulers"]:3d} {seed:4d} {saw:>5} {stale:5d} | {cell(radio):>12} | '
                  f'{cell(resync):>14} | {row["broadcast"]:9d}', flush=True)
            if resync['solved'] and resync['sends'] != stale:
                print(f'    resync sent {resync["sends"]} for {stale} stale entries')
    with open(os.path.join(args.out, 'scaling.json'), 'w') as fh:
        json.dump(rows, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
