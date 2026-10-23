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
The executor's vocabulary for the policies of gossip-coverage.epddl, and the
mapping into it.

    make_pddl.py --out pddl/

gc-vocabulary.pddl  one action per thing a robot does, over (ok ?r), which an
                    action of robot ?r reads and writes, and (count_n): the
                    arrival go_site_n reads (count_n) and writes
                    (count_n+1), and the arrival do_task_k reads (count_k-1),
                    so that arrivals are dispatched in the counter's order, as
                    the epistemic state requires. ePlanSys's executor runs
                    together the actions of a policy of which neither writes
                    what the other reads (schedule.py is its port): the task is
                    never started before every arrival before it, and arrivals
                    and calls of different robots run side by side. What decides
                    the policy is in the EPDDL.
gc-mapping.json     every grounded action of the EPDDL for the three robots
                    as a vocabulary action, with the duration the executor's
                    scheduler reads (declared, as in gossip/epistemic.py).
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as A  # noqa: E402
from gossip.epistemic import DURATION  # noqa: E402

N_SITE = 8       # the domain's counter goes to eight
RELAYED = {'call': 2, 'await': 2, 'leave': 2}


def vocabulary():
    acts = []

    def action(name, params, cond, eff):
        typed = ' '.join(params) + ' - robot' if params else ''
        c = ' '.join(f'(at start {f})' for f in cond)
        e = ' '.join(f'(at end {f})' for f in eff)
        acts.append(f'(:durative-action {name}\n  :parameters ({typed})\n  :duration (= ?duration 10)\n'
                    f'  :condition (and {c})\n  :effect (and {e}))\n')
    for name in ('call', 'await', 'leave'):
        action(name, ['?a', '?b'], ['(ok ?a)', '(ok ?b)'], ['(ok ?a)', '(ok ?b)'])
    for n in range(N_SITE):
        action(f'go_site_{n}', ['?r'], ['(ok ?r)'] + ([f'(count_{n})'] if n else []), ['(ok ?r)', f'(count_{n + 1})'])
    for k in range(1, N_SITE + 1):
        action(f'do_task_{k}', ['?r'], ['(ok ?r)'] + ([f'(count_{k - 1})'] if k > 1 else []), ['(ok ?r)'])
    preds = ' '.join(f'(count_{i})' for i in range(1, N_SITE + 1))
    return (';; GOSSIP OUT OF CONTACT: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain gossip-coverage)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates (ok ?r - robot) {preds})\n' + '\n'.join(acts) + ')\n')


def mapping(robots=A.ROBOTS):
    out = {}
    for i in robots:
        for j in robots:
            if i == j:
                continue
            for name in ('call', 'await', 'leave'):
                out[f'{name}_{i}_{j}'] = {'action': f'({name} {i} {j})', 'duration': float(DURATION[name])}
        for n in range(N_SITE):
            out[f'go-site-{n}_{i}'] = {'action': f'(go_site_{n} {i})', 'duration': float(DURATION['go-site'])}
        for k in range(1, N_SITE + 1):
            out[f'do-task-{k}_{i}'] = {'action': f'(do_task_{k} {i})', 'duration': float(DURATION['do-task'])}
    return out


def relayed(robots=A.ROBOTS):
    """The vocabulary actions a robot's relay serves: the robot is the
    action's first argument. Arrivals and tasks only up to three."""
    names = ['call', 'await', 'leave'] + [f'go_site_{n}' for n in range(len(robots))] + \
        [f'do_task_{k}' for k in range(1, len(robots) + 1)]
    return names


def problem(robots=A.ROBOTS):
    return ('(define (problem gossip-coverage) (:domain gossip-coverage)\n'
            f"(:objects {' '.join(robots)} - robot)\n"
            f"(:init {' '.join(f'(ok {r})' for r in robots)})\n"
            f"(:goal (and (ok {robots[0]}))))\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'gc-vocabulary.pddl'), 'w').write(vocabulary())
    json.dump(mapping(), open(os.path.join(a.out, 'gc-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
