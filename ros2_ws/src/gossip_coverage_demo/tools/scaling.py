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
What planning the gossip costs, as the team grows.

    scaling.py --out DIR [--sizes 2 3 4 5 6 7 8] [--geometries 5]

For every team size N, every need k in {2, 3, N} not above N, Case I (no
robot busy, every robot must learn of the task) and Case II with at most b in
{1, 2} of the others busy, and a few random geometries of the particles (in
the paper's 50 m area), the instance of gossip/epddl.py is grounded by plank
and planned by Aletheia as the demonstration plans it (--strategy replan
--subjective auto --consistent-beliefs), and for teams of up to four also by
AO*, which returns a policy of least depth. Writes DIR/scaling.json: atoms,
worlds and designated worlds of the initial model, ground actions, seconds,
the policy's depth and leaves, and, where AO* ran, its depth.
"""

import argparse
import json
import os
import random
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gossip import epddl as P  # noqa: E402
from gossip.epistemic import near_relation  # noqa: E402


def shape(policy):
    def depth(n):
        return 0 if n is None else 1 + max(depth(b['subtree']) for b in n['branches'])

    def leaves(n):
        return 1 if n is None else sum(leaves(b['subtree']) for b in n['branches'])
    return depth(policy), leaves(policy)


def plan(task, out, args, timeout):
    pol = os.path.join(out, 'p.json')
    if os.path.exists(pol):
        os.remove(pol)
    t = time.time()
    try:
        r = subprocess.run([P.PLANNER, '--task', task, '--plan', pol, '--consistent-beliefs',
                            '--timeout', str(int(timeout))] + args, capture_output=True, text=True,
                           timeout=timeout + 30)
        log = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        log = 'timeout'
    dt = time.time() - t
    loaded = next((l for l in log.splitlines() if 'Loaded' in l), '')
    if not os.path.exists(pol):
        return None, dt, loaded
    return shape(json.load(open(pol))), dt, loaded


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--sizes', type=int, nargs='+', default=[2, 3, 4, 5, 6, 7, 8])
    ap.add_argument('--geometries', type=int, default=5)
    ap.add_argument('--timeout', type=float, default=60.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for n in a.sizes:
        agents = [f'r{i}' for i in range(1, n + 1)]
        for k in sorted({2, 3, n} & set(range(2, n + 1))):
            for case, b in ((1, 0), (2, 1), (2, 2)):
                if b > n - 1:
                    continue
                for g in range(a.geometries):
                    rng = random.Random(1000 * n + 100 * k + 10 * b + g)
                    pos = {r: (rng.uniform(0, 50), rng.uniform(0, 50)) for r in agents}
                    order = ['r1'] + sorted(agents[1:], key=lambda r: (pos[r][0] - pos['r1'][0]) ** 2
                                            + (pos[r][1] - pos['r1'][1]) ** 2)
                    inst = P.Instance(order, 'r1', k, case, may_be_busy=order[1:] if b else [], max_busy=b,
                                      near=near_relation(pos), name=f'n{n}k{k}c{case}b{b}g{g}')
                    d = os.path.join(a.out, inst.name)
                    task = P.export(inst, d)
                    t = json.load(open(task))
                    row = {'n': n, 'k': k, 'case': case, 'b': b, 'g': g,
                           'atoms': len(t['language']['atoms']),
                           'worlds': len(t['initial-state']['worlds']),
                           'designated': len(t['initial-state']['designated']),
                           'actions': len(t['actions'])}
                    s, dt, _ = plan(task, d, ['--strategy', 'replan', '--subjective', 'auto'], a.timeout)
                    row.update(solved=s is not None, seconds=round(dt, 3), depth=s[0] if s else None,
                               leaves=s[1] if s else None)
                    if n <= 4:
                        s2, dt2, _ = plan(task, d, ['--conditional', '--subjective', 'auto'], a.timeout)
                        row.update(aostar_depth=s2[0] if s2 else None, aostar_seconds=round(dt2, 3))
                    rows.append(row)
                    print(json.dumps(row), flush=True)
    json.dump(rows, open(os.path.join(a.out, 'scaling.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
