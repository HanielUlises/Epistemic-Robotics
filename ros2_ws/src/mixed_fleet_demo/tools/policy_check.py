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
Every policy checked against the domain's own semantics.

    policy_check.py [--acts 1 2 3] [--formulations ...] [--out DIR]

Each act's policy under each formulation is built by tools/plan.py and
replayed, along every branch, through two semantics side by side: the
product update of the task plank grounds from the EPDDL (the repository's
tools/plan_trace.py, an implementation independent of ePlanSys's and
Aletheia's), and the reference semantics the planner reasons in
(tools/kripke.py). At every node:

  1. the action applies: its designated events' preconditions cover the
     designated worlds, the second-order guard of a belief report among them;
  2. for every robot, its cell at the designated worlds is the same set of
     loops in both;
  3. for every robot, its grade of every loop in its cell, read off the
     model's grade atoms, is the rank the reference gives: the belief
     reports' conditional effects raise exactly the grades Spohn conditioning
     on evidence of strength ALPHA raises, and no cap binds;
  4. so every robot believes the same loops in both.

Every leaf meets the goal. With --out, the policies and tasks are written to
DIR/act<k>-<formulation>/ for the AletheiaPlanSolver check of validate.sh.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
for up in ('../../../../tools', '../../../tools'):
    p = os.path.realpath(os.path.join(HERE, up))
    if os.path.exists(os.path.join(p, 'plan_trace.py')):
        sys.path.insert(0, p)
import plan as P  # noqa: E402
import ranks as R  # noqa: E402
from model_check import model_of, extensions  # noqa: E402
from plan_trace import convert, update  # noqa: E402


def epddl_dir():
    for d in (os.path.join(HERE, '..', 'epddl'), os.path.join(HERE, '../../../../epddl-workspace/mixed-fleet')):
        if os.path.exists(os.path.join(d, 'mixed-fleet.epddl')):
            return os.path.realpath(d)
    raise FileNotFoundError('mixed-fleet.epddl')


def load_task(act):
    """The grounded task of an act, exported by plank once and kept."""
    epddl = epddl_dir()
    plank = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
    lib = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
    out = os.path.join(tempfile.gettempdir(), f'mixed-fleet-tasks-{os.getuid()}', f'act{act}')
    path = os.path.join(out, f'act{act}.json')
    src = os.path.join(epddl, 'instances', f'act{act}.epddl')
    dom = os.path.join(epddl, 'mixed-fleet.epddl')
    if not os.path.exists(path) or os.path.getmtime(path) < max(os.path.getmtime(src), os.path.getmtime(dom)):
        subprocess.run([plank, 'export', '-d', dom, '-p', src, '-l', lib, '-o', out], check=True, capture_output=True)
    return json.load(open(path)), path


def grade(model, robot, w):
    for k in range(R.GRADES):
        if f'rank{k}_{robot}' in model.labels[w]:
            return k
    raise ValueError(f'{w} has no grade for {robot}: {sorted(model.labels[w])}')


class Checker:

    def __init__(self, act, formulation):
        self.act, self.f = act, formulation
        self.team, self.tree = P.plan(act, formulation)
        self.task, self.task_path = load_task(act)
        self.agents = self.task['language']['agents']
        self.out = {'act': act, 'formulation': formulation, 'nodes': 0, 'leaves': 0, 'beliefs': 0,
                    'reports': 0, 'errors': []}

    def run(self):
        model = model_of(self.task['initial-state'])
        origin = {w: int(w[1:]) for w in model.worlds}
        M, phys = self.team.start()
        self.compare(model, origin, M, ['initial'])
        self.walk(self.tree, model, origin, M, phys, [])
        return self.out

    def compare(self, model, origin, M, path):
        if M.capped:
            self.out['errors'].append(f'{path}: a grade hit the cap')
        for r in self.team.names:
            for w in model.designated:
                cell = sorted({origin[v] for v in model.image(r, w)})
                ref = sorted(M.cell(r, origin[w]))
                if cell != ref:
                    self.out['errors'].append(f'{path}: {r} cell {cell}, the reference {ref}')
                    continue
                for v in model.image(r, w):
                    g = grade(model, r, v)
                    if g != M.ranks[r][origin[v]]:
                        self.out['errors'].append(f'{path}: {r} grades {R.name(R.worlds()[origin[v]])} {g}, '
                                                  f'the reference {M.ranks[r][origin[v]]}')
                cellw = model.image(r, w)
                m = min(grade(model, r, v) for v in cellw)
                got = sorted({origin[v] for v in cellw if grade(model, r, v) == m})
                if got != sorted(M.least(r, origin[w])):
                    self.out['errors'].append(f'{path}: {r} believes {got}, the reference {M.least(r, origin[w])}')
                self.out['beliefs'] += 1

    def applies(self, model, action):
        covered = set()
        for e in action['designated']:
            f = convert(action['preconditions'][e])
            covered |= extensions(model, f)[f]
        return model.designated <= covered

    def step(self, model, origin, name, events=None):
        action = self.task['actions'].get(name)
        if action is None:
            raise KeyError(f'no action {name}')
        if not self.applies(model, action) and events is None:
            raise ValueError(f'{name} does not apply')
        m2, pairs = update(model, action, self.agents, events or action['designated'])
        return m2, {w: origin[pairs[w][0]] for w in m2.worlds}

    def walk(self, steps, model, origin, M, phys, path):
        for k, st in enumerate(steps):
            if st[0] == 'branch':
                _, r, a, branches = st
                name = P.grounded(r, a)
                action = self.task['actions'][name]
                if not self.applies(model, action):
                    self.out['errors'].append(f'{path}: {name} does not apply')
                    return
                self.out['nodes'] += 1
                M2, phys2 = self.team.apply(M, phys, r, a)
                for value, sub in branches.items():
                    ev = next(e for e in action['events'] if e.endswith('-open' if value else '-shut'))
                    m2, o2 = self.step(model, origin, name, [ev])
                    if not m2.designated:
                        self.out['errors'].append(f'{path}: {name}/{ev} is not produced')
                        continue
                    Mb = M2.copy()
                    Mb.designated = frozenset(w for w in M2.designated if R.open_at(R.worlds()[w], a[1]) == value)
                    self.compare(m2, o2, Mb, path + [f'{name}:{ev}'])
                    self.walk(sub, m2, o2, Mb, phys2, path + [f'{name}:{ev}'])
                return
            r, a = st
            name = P.grounded(r, a)
            try:
                model, origin = self.step(model, origin, name)
            except (KeyError, ValueError) as e:
                self.out['errors'].append(f'{path}: {e}')
                return
            self.out['nodes'] += 1
            if a[0] == 'tellb':
                self.out['reports'] += 1
            M, phys = self.team.apply(M, phys, r, a)
            path = path + [name]
            self.compare(model, origin, M, path)
        goal = convert(self.task['goal'])
        if not model.designated <= extensions(model, goal)[goal]:
            self.out['errors'].append(f'leaf {path}: goal does not hold')
        self.out['leaves'] += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--acts', type=int, nargs='*', default=sorted(R.FLEETS))
    ap.add_argument('--formulations', nargs='*', default=list(R.FORMULATIONS))
    ap.add_argument('--out')
    a = ap.parse_args()
    bad = 0
    for act in a.acts:
        for f in a.formulations:
            c = Checker(act, f)
            r = c.run()
            bad += len(r['errors'])
            print(f'act {act} {f:12s}: {r["nodes"]} nodes, {r["leaves"]} leaves, {r["beliefs"]} beliefs checked, '
                  f'{r["reports"]} belief reports, {len(r["errors"])} errors')
            for e in r['errors'][:5]:
                print('   ', e)
            if a.out:
                d = os.path.join(a.out, f'act{act}-{f}')
                os.makedirs(d, exist_ok=True)
                events = {n: x['events'] for n, x in c.task['actions'].items()}
                json.dump(P.to_policy(c.tree, events), open(os.path.join(d, 'policy.json'), 'w'))
                if not os.path.exists(os.path.join(d, 'task.json')):
                    os.symlink(c.task_path, os.path.join(d, 'task.json'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
