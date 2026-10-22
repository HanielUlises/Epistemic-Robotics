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
How the policy's search grows with the team, without a simulator.

    scaling.py OUT [--max 6] [--timeout 900] [--threads 2]

The paper runs teams of two, three, five and eight robots. Here the domain is
belief-empathy.epddl unchanged and the instance has one aerial vehicle and n
ground vehicles, g1 to gn, the first half covering the west and the rest the
east, the site in the east, and the same budget of at most two faults in the
team. The worlds are the fault patterns that budget allows,
1 + 2n + n(n - 1)/2 of them. For each n it writes the instance, grounds it with
plank, plans it with Aletheia's AO* with consistent beliefs, and records the
worlds, the ground actions, the depth of the policy, the nodes expanded and
the wall time, in OUT/scaling.json.
"""

import argparse
import itertools
import json
import os
import re
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
EPDDL = os.environ.get('EPDDL', os.path.realpath(os.path.join(HERE, '..', '..', '..', '..', 'epddl-workspace',
                                                               'belief-empathy')))
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))


def instance(n):
    gs = [f'g{k}' for k in range(1, n + 1)]
    west = set(gs[:n // 2])
    lines = ['([C. All] (uav u))', '([C. All] (not (ugv u)))', '([C. All] (not (west u)))', '([C. All] (free u))',
             '(task)', '([C. All] ([Kw. u] (task)))', '([C. All] (not (done)))',
             '([C. All] (not (p1 u)))', '([C. All] (not (p2 u)))', '([C. All] (not (p3 u)))']
    for g in gs:
        lines += [f'([C. All] (not (uav {g})))', f'([C. All] (ugv {g}))',
                  f'([C. All] ({"west" if g in west else "not (west"} {g}){"" if g in west else ")"})',
                  f'([C. All] (not (free {g})))', f'([C. All] (<Kw. {g}> (task)))',
                  f'([C. All] (or (p1 {g}) (p2 {g}) (p3 {g})))',
                  f'([C. All] (not (and (p1 {g}) (p2 {g}))))', f'([C. All] (not (and (p1 {g}) (p3 {g}))))',
                  f'([C. All] (not (and (p2 {g}) (p3 {g}))))']
        lines += [f'([C. All] ([Kw. {g}] (p{b} {g})))' for b in (1, 2, 3)]
    for p in ('at-site', 'by1', 'by2', 'by3', 'passed1', 'passed2'):
        lines.append(f'(:forall (?i - agent) ([C. All] (not ({p} ?i))))')
    # At most two faults: no two vehicles with a fault where one has two, and
    # no three with one each.
    for a, b in itertools.permutations(gs, 2):
        lines.append(f'([C. All] (not (and (p3 {a}) (not (p1 {b})))))')
    for a, b, c in itertools.combinations(gs, 3):
        lines.append(f'([C. All] (not (and (p2 {a}) (p2 {b}) (p2 {c}))))')
    body = '\n      '.join(lines)
    return f'''(define (problem belief-empathy-n{n})
  (:domain belief-empathy)
  (:requirements
    :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions
  )
  (:agents u {' '.join(gs)})
  (:init
    (:and
      {body}
    )
  )
  (:goal (done))
)
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--min', type=int, default=2)
    ap.add_argument('--max', type=int, default=6)
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--threads', type=int, default=2)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, 'scaling.json')
    results = json.load(open(path)) if os.path.exists(path) else {}
    for n in range(a.min, a.max + 1):
        if str(n) in results:
            continue
        d = os.path.join(a.out, f'n{n}')
        os.makedirs(d, exist_ok=True)
        prob = os.path.join(d, f'problem_n{n}.epddl')
        open(prob, 'w').write(instance(n))
        subprocess.run([PLANK, 'export', '-d', os.path.join(EPDDL, 'belief-empathy.epddl'), '-p', prob, '-l', LIB,
                        os.path.join(EPDDL, 'hailing.epddl'), '-o', d], check=True, capture_output=True)
        task = os.path.join(d, f'problem_n{n}.json')
        t = time.time()
        try:
            out = subprocess.run([PLANNER, '--task', task, '--plan', os.path.join(d, 'policy.json'), '--conditional',
                                  '--consistent-beliefs', '--threads', str(a.threads)],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=a.timeout).stdout
            took = time.time() - t
        except subprocess.TimeoutExpired as e:
            out, took = (e.stdout or b'').decode() if isinstance(e.stdout, bytes) else (e.stdout or ''), None
        loaded = re.search(r'Loaded: (\d+) atoms, (\d+) agents, (\d+) worlds \((\d+) designated\), (\d+) actions', out)
        solved = re.search(r'Solution found at depth (\d+)\s+Expanded=(\d+)', out)
        results[str(n)] = {
            'n_ugv': n, 'atoms': int(loaded.group(1)) if loaded else None,
            'worlds': int(loaded.group(3)) if loaded else None,
            'designated': int(loaded.group(4)) if loaded else None,
            'actions': int(loaded.group(5)) if loaded else None,
            'depth': int(solved.group(1)) if solved else None,
            'expanded': int(solved.group(2)) if solved else None,
            'seconds': round(took, 2) if took is not None else None,
            'result': 'solved' if solved else ('timeout' if took is None else 'none')}
        json.dump(results, open(path, 'w'), indent=1)
        print(n, results[str(n)], flush=True)
        if took is None:
            break


if __name__ == '__main__':
    main()
