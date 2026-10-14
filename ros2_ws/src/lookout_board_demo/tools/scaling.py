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
What planning the lookout board costs as the lookouts grow.

    scaling.py --lookouts 2 3 4 5 6 7 8 --out scaling.json [--timeout 600]

For each n the domain is written by make_domain.py, grounded by plank and
solved by Aletheia with the strategy left to its selection policy. Recorded:
the grounded size, the strategy that returned the policy, its depth, leaves
and expansions, the wall-clock time, and the policy's shape: whether it reads
the board first, and how many slots it reads on its longest branch. The depth
of the board-first policy is n + 2 (drive to the board, n - 1 reads, drive,
look), and that of checking the lookouts in turn is 2n.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.environ.get('PLANK_LIBRARY', os.path.expanduser(
    '~/eplansys_ws/install/plansys2_epddl_grounder/share/plansys2_epddl_grounder/'
    'libraries/intermediate.epddl'))
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
ALETHEIA = os.environ.get('ALETHEIA', os.path.expanduser(
    '~/eplansys_ws/install/aletheia/bin/epistemic_planner'))


def shape(tree):
    """First action, and the most reads on any branch."""
    def reads(t):
        if not t:
            return 0
        here = 1 if t['action'].startswith('read-') else 0
        return here + max((reads(b['subtree']) for b in t['branches']), default=0)

    def depth(t):
        if not t:
            return 0
        return 1 + max((depth(b['subtree']) for b in t['branches']), default=0)
    return tree['action'], reads(tree), depth(tree)


def one(n, timeout):
    d = tempfile.mkdtemp()
    subprocess.run([sys.executable, os.path.join(HERE, 'make_domain.py'), '--lookouts', str(n),
                    '--out', d], check=True)
    subprocess.run([PLANK, 'export', '-d', os.path.join(d, 'lookout-board.epddl'),
                    '-p', os.path.join(d, 'instances', 'problem_1.epddl'), '-l', LIB, '-o', d],
                   check=True, capture_output=True)
    t = time.time()
    p = subprocess.run([ALETHEIA, '--task', os.path.join(d, 'problem_1.json'),
                        '--plan', os.path.join(d, 'policy.json'), '--timeout', str(timeout)],
                       capture_output=True, text=True)
    took = time.time() - t
    log = p.stdout + p.stderr
    row = {'lookouts': n, 'seconds': round(took, 2)}
    m = re.search(r'Loaded: (\d+) atoms, \d+ agents, (\d+) worlds \((\d+) designated\), (\d+) actions', log)
    if m:
        row.update(atoms=int(m.group(1)), worlds=int(m.group(2)), actions=int(m.group(4)))
    strat = 'AO*'
    if 'switching to replan' in log or '[replan] Solution' in log:
        strat = 'replan'
    exp = re.findall(r'Expanded=(\d+)', log)
    row['expansions'] = int(exp[-1]) if exp else None
    v = re.search(r'validator\] OK .*?(\d+) leaves', log)
    row['validated'] = bool(v)
    row['leaves'] = int(v.group(1)) if v else None
    if os.path.exists(os.path.join(d, 'policy.json')) and os.path.getsize(os.path.join(d, 'policy.json')) > 4:
        first, reads, depth = shape(json.load(open(os.path.join(d, 'policy.json'))))
        row.update(strategy=strat, first=first, most_reads=reads, depth=depth,
                   board_first_depth=n + 2, check_in_turn_depth=2 * n)
    else:
        row.update(strategy=None, first=None)
    print(json.dumps(row), flush=True)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lookouts', type=int, nargs='+', default=[2, 3, 4, 5, 6, 7, 8])
    ap.add_argument('--timeout', type=int, default=600)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rows = [one(n, a.timeout) for n in a.lookouts]
    json.dump(rows, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
