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
Disagreement that every robot knows of.

    agreement.py [--act 1] [--formulation egocentric] [--night AB] [--after tell-b_c]

Replays the act's policy on one night through the product update of the
grounded task (as tools/policy_check.py does) up to the first time the named
action has happened, and there checks, on the model the update produced, the
formula

    C_G ( B_r1 phi_1 AND B_r2 phi_2 AND ... )

for every robot's belief about every door: G the robots, C_G common knowledge
(truth at every world reachable from the designated ones by any robot's
relation), and each robot's belief read off its own grade atoms. It prints the
beliefs every robot commonly knows every robot holds, and the doors on which
they differ.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plan as P  # noqa: E402
import policy_check as C  # noqa: E402
import ranks as R  # noqa: E402


def belief(model, robot, w, door):
    """'open', 'shut' or None: what robot believes of door at w, from the model alone."""
    cell = model.image(robot, w)
    m = min(C.grade(model, robot, v) for v in cell)
    top = [v for v in cell if C.grade(model, robot, v) == m]
    if all(f'open-{door}' in model.labels[v] for v in top):
        return 'open'
    if all(f'open-{door}' not in model.labels[v] for v in top):
        return 'shut'
    return None


def component(model, robots):
    seen, todo = set(model.designated), list(model.designated)
    while todo:
        w = todo.pop()
        for r in robots:
            for v in model.image(r, w):
                if v not in seen:
                    seen.add(v)
                    todo.append(v)
    return seen


def state_after(act, formulation, night, after):
    chk = C.Checker(act, formulation)
    model = C.model_of(chk.task['initial-state'])
    origin = {w: int(w[1:]) for w in model.worlds}
    k = R.worlds().index(night)
    for r, a in P.paths(chk.tree, k):
        name = P.grounded(r, a)
        events = None
        if a[0] == 'look':
            ev = [e for e in chk.task['actions'][name]['events']
                  if e.endswith('-open' if R.open_at(night, a[1]) else '-shut')]
            events = ev
        model, origin = chk.step(model, origin, name, events)
        if name == after:
            return chk, model
    raise ValueError(f'{after} does not happen on {R.name(night)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--act', type=int, default=1)
    ap.add_argument('--formulation', default='egocentric')
    ap.add_argument('--night', default='AB')
    ap.add_argument('--after', default='tell-b_c')
    a = ap.parse_args()
    night = next(w for w in R.worlds() if R.name(w) == a.night)
    chk, model = state_after(a.act, a.formulation, night, a.after)
    robots = chk.team.names
    comp = component(model, robots)
    common = {}
    for r in robots:
        for d in R.DOORS:
            vals = {belief(model, r, w, d) for w in comp}
            common[(r, d)] = vals.pop() if len(vals) == 1 else 'not common'
    print(f'act {a.act}, {a.formulation}, night {a.night}, after {a.after}: '
          f'{len(comp)} worlds reachable, {len(model.designated)} designated')
    for r in robots:
        print(f'  commonly known: {r} believes ' + ', '.join(
            f'{d.upper()} {common[(r, d)] or "neither"}' for d in R.DOORS))
    split = [d.upper() for d in R.DOORS if len({common[(r, d)] for r in robots}) > 1]
    print(f'  they disagree, and commonly know it, on: {", ".join(split) or "nothing"}')


if __name__ == '__main__':
    main()
