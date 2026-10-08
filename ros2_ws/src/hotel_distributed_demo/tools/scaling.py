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
The hotel's distributed leak as the building grows.

    scaling.py --cells 2x2 2x3 3x2 3x3 --out /tmp/hotel_scaling --budget 1800 --jobs 4

For each cell, F floors by K columns, writes the domain with
tools/hotel_domain.py, grounds every fleet with plank, solves each with
Aletheia's AO* at one thread, and replays every policy with tools/trace.py
against the whole goal. Prints one table row per cell and fleet: the size of
the grounded task, the policy's depth and leaves, the expansions and the
time, the most lift rides, inspections and messages along any branch, and at
how many leaves each conjunct of the whole goal holds.

AO* deepens one action at a time, so a policy it returns has the fewest
actions in its deepest branch, and a search cut off at depth d has shown
that no policy of depth below d exists. Where a search was cut off nothing
else is claimed from it.
"""

import argparse
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hotel_domain  # noqa: E402

PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser(
    '~/eplansys_ws/install/aletheia/bin/epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.expanduser(
    '~/plank/benchmarks/libraries/intermediate.epddl'))
FLEETS = ('epistemic', 'filter', 'broadcast', 'siloed')


def ground(out, fleet):
    target = os.path.join(out, fleet)
    subprocess.run([PLANK, 'export', '-d', os.path.join(out, 'hotel-distributed.epddl'),
                    '-p', os.path.join(out, f'{fleet}.epddl'), '-l', LIB,
                    os.path.join(out, 'earshot.epddl'), '-o', target],
                   check=True, capture_output=True)
    return os.path.join(target, f'{fleet}.json')


def solve(task, budget):
    plan = os.path.join(os.path.dirname(task), 'plan.json')
    log = os.path.join(os.path.dirname(task), 'search.log')
    with open(log, 'w') as fh:
        subprocess.run(['/usr/bin/time', '-f', 'wall %e', PLANNER, '--task', task, '--plan', plan,
                        '--timeout', str(budget), '--strategy', 'aostar', '--no-portfolio',
                        '--threads', '1'], stdout=fh, stderr=subprocess.STDOUT,
                       timeout=budget + 120)
    text = open(log).read()
    found = re.search(r'Solution found at depth (\d+)\s+Expanded=(\d+)', text)
    wall = re.search(r'wall ([\d.]+)', text)
    tried = re.findall(r'Trying depth (\d+)', text)
    return {'depth': int(found.group(1)) if found else None,
            'expanded': int(found.group(2)) if found else None,
            'time': float(wall.group(1)) if wall else None,
            'cut': None if found else (int(tried[-1]) if tried else 0),
            'plan': plan if found else None}


def judge(task, plan, whole):
    out = subprocess.run([sys.executable, os.path.join(HERE, 'trace.py'), '--task', task,
                          '--whole', whole, '--plan', plan],
                         capture_output=True, text=True, check=True).stdout
    leaves = [ln for ln in out.splitlines() if ln.strip().startswith('leaf')]
    count = {k: sum(f'{k} holds' in ln for ln in leaves)
             for k in ('safe', 'stand-down', 'secret')}
    worst = {k: max(int(m) for m in re.findall(rf'(\d+) {k}', out) or ['0'])
             for k in ('lift rides', 'inspections')}
    private = max(int(m) for m in re.findall(r'(\d+) private', out) or ['0'])
    public = max(int(m) for m in re.findall(r'(\d+) public', out) or ['0'])
    return len(leaves), count, worst, private, public


def cell(spec, root, budget):
    nf, nk = (int(x) for x in spec.split('x'))
    out = os.path.join(root, spec)
    hotel_domain.write(out, nf, nk, fleets=FLEETS)
    tasks = {f: ground(out, f) for f in FLEETS}
    sizes = {}
    for f, t in tasks.items():
        data = json.load(open(t))
        sizes[f] = (len(data['initial-state']['worlds']), len(data['language']['atoms'])
                    if 'atoms' in data.get('language', {}) else None, len(data['actions']))
    rows = []
    for f in FLEETS:
        r = solve(tasks[f], budget)
        if r['plan']:
            r['leaves'], r['count'], r['worst'], r['private'], r['public'] = \
                judge(tasks[f], r['plan'], tasks['epistemic'])
        rows.append((spec, f, sizes[f], r))
        print(row_text(spec, f, sizes[f], r), flush=True)
    return rows


def row_text(spec, fleet, size, r):
    worlds, atoms, actions = size
    if r['plan'] is None:
        return (f'{spec:5s} {fleet:10s} {worlds:3d} worlds {actions:4d} actions   '
                f'cut off at depth {r["cut"]}')
    c = r['count']
    return (f'{spec:5s} {fleet:10s} {worlds:3d} worlds {actions:4d} actions   depth {r["depth"]:2d}, '
            f'{r["leaves"]:2d} leaves, {r["expanded"]:>9,} expanded, {r["time"]:7.1f} s   '
            f'rides {r["worst"]["lift rides"]}, inspections {r["worst"]["inspections"]}, '
            f'messages {r["private"]}+{r["public"]}   '
            f'safe {c["safe"]}/{r["leaves"]}, stand-down {c["stand-down"]}/{r["leaves"]}, '
            f'secret {c["secret"]}/{r["leaves"]}')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--cells', nargs='+', default=['2x2', '2x3', '3x2'])
    p.add_argument('--out', default='/tmp/hotel_scaling')
    p.add_argument('--budget', type=int, default=1800)
    p.add_argument('--jobs', type=int, default=2)
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda s: cell(s, args.out, args.budget), args.cells))
    print()
    for rows in results:
        for spec, f, size, r in rows:
            print(row_text(spec, f, size, r))
    with open(os.path.join(args.out, 'scaling.json'), 'w') as fh:
        json.dump([[spec, f, size, {k: v for k, v in r.items() if k != 'plan'}]
                   for rows in results for spec, f, size, r in rows], fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
