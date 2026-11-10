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
     designated worlds, a bed-mover's guard among them;
  2. for every robot, its cell at the designated worlds is the same set of
     worlds of the model of error in both;
  3. every world's grade atom is the rank the model of error gives it, so
     every robot believes the same worlds in both, and as firmly.

Every leaf meets the goal. Over the policy as a whole:

  4. on every night, the policy has a branch for what the night's lasers read;
  5. a bed-mover drives into a shut passage only on a night whose rank in the
     fleet's own model is at least the degree of belief its formulation asks
     for: never, under knowledge.

With --out, the policies and tasks are written to DIR/act<k>-<formulation>/
for the AletheiaPlanSolver check of validate.sh.
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
from kripke import State  # noqa: E402


def epddl_dir():
    for d in (os.path.join(HERE, '..', 'epddl'), os.path.join(HERE, '../../../../epddl-workspace/faulty-sensor')):
        if os.path.exists(os.path.join(d, 'faulty-sensor.epddl')):
            return os.path.realpath(d)
    raise FileNotFoundError('faulty-sensor.epddl')


def load_task(act, kind):
    """The grounded task of an act under a model of error, exported by plank once and kept."""
    import model as MD
    epddl = epddl_dir()
    plank = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
    lib = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
    inst = f'act{act}-{kind}'
    out = os.path.join(tempfile.gettempdir(), f'faulty-sensor-tasks-{os.getuid()}', inst)
    path = os.path.join(out, f'{inst}.json')
    src = os.path.join(epddl, 'instances', f'{inst}.epddl')
    dom = os.path.join(epddl, f'{MD.DOMAIN[kind]}.epddl')
    if not os.path.exists(path) or os.path.getmtime(path) < max(os.path.getmtime(src), os.path.getmtime(dom)):
        subprocess.run([plank, 'export', '-d', dom, '-p', src, '-l', lib, '-o', out], check=True, capture_output=True)
    return json.load(open(path)), path


def grade(model, w):
    for k in range(R.GRADES):
        if f'rank{k}' in model.labels[w]:
            return k
    raise ValueError(f'{w} has no grade: {sorted(model.labels[w])}')


class Checker:

    def __init__(self, act, formulation):
        self.act, self.f = act, formulation
        self.pl, tree = P.plan(act, formulation)
        self.tree = tree
        self.steps = P.flat(tree)
        self.M = self.pl.M
        self.task, self.task_path = load_task(act, R.MODEL[formulation])
        self.agents = self.task['language']['agents']
        self.out = {'act': act, 'formulation': formulation, 'nodes': 0, 'leaves': 0, 'beliefs': 0,
                    'entries': 0, 'errors': []}

    def run(self):
        model = model_of(self.task['initial-state'])
        origin = {w: int(w[1:]) for w in model.worlds}
        S = State.initial(self.M, self.pl.names)
        self.compare(model, origin, S, ['initial'])
        self.walk(self.steps, model, origin, S, [])
        self.nights()
        return self.out

    # -- the reference update of each action ---------------------------------------------

    def is_open(self, d):
        return lambda w: d not in self.M.worlds[w][0]

    def reading(self, r, a):
        """The function of a world a reading reads: True for open."""
        d = a[1]
        if a[0] == 'check':
            return self.is_open(d)
        shut = self.M.shut_reading(r, d, a[2])
        return lambda w: not shut >> w & 1

    def apply(self, S, r, a):
        k = a[0]
        if k == 'enter':
            firm = R.FIRM[self.f]
            f = self.is_open(a[1])
            return S.restrict(lambda w: S.K(r, w, f) if firm is None else S.B(r, w, f, firm))
        if k == 'cross':
            f = self.is_open(a[1])
            return S.restrict(lambda w: f(w) and S.K(r, w, f))
        if k == 'say':
            d = a[1]
            if a[2] == 'shut':
                f = lambda w: not self.is_open(d)(w)                 # noqa: E731
            elif a[2] == 'sure':
                f = self.is_open(d)
            elif a[2] == 'open':
                f = self.reading(r, ('look', d, a[3]))
            else:
                reads_open = self.reading(r, ('look', d, a[3]))
                f = lambda w: not reads_open(w)                      # noqa: E731
            return S.restrict(lambda w: f(w) and S.K(r, w, f))
        if k in ('look', 'check'):
            return S.refine(r, self.reading(r, a))
        return S

    # -- the comparison ------------------------------------------------------------------

    def compare(self, model, origin, S, path):
        for w in model.worlds:
            if grade(model, w) != self.M.rank[origin[w]]:
                self.out['errors'].append(f'{path}: {w} carries grade {grade(model, w)}, the model of error '
                                          f'{self.M.rank[origin[w]]}')
        if sorted(origin[w] for w in model.designated) != sorted(S.designated):
            self.out['errors'].append(f'{path}: designated {sorted(origin[w] for w in model.designated)}, '
                                      f'the reference {sorted(S.designated)}')
            return
        for r in self.pl.names:
            for w in model.designated:
                cell = sorted({origin[v] for v in model.image(r, w)})
                ref = sorted(S.cell(r, origin[w]))
                if cell != ref:
                    self.out['errors'].append(f'{path}: {r} cell {cell}, the reference {ref}')
                    break
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

    def walk(self, steps, model, origin, S, path):
        for st in steps:
            if st[0] == 'branch':
                _, r, a, branches = st
                name = P.grounded(r, a, self.f)
                action = self.task['actions'].get(name)
                if action is None or not self.applies(model, action):
                    self.out['errors'].append(f'{path}: {name} does not apply')
                    return
                self.out['nodes'] += 1
                S2 = self.apply(S, r, a)
                reads_open = self.reading(r, a)
                for o, sub in branches.items():
                    ev = next(e for e in action['events'] if e.split('_')[0].endswith('-' + o))
                    m2, o2 = self.step(model, origin, name, [ev])
                    if not m2.designated:
                        self.out['errors'].append(f'{path}: {name}/{ev} is not produced')
                        continue
                    Sb = S2.select(lambda w: reads_open(w) == (o == 'open'))
                    self.compare(m2, o2, Sb, path + [f'{name}:{o}'])
                    self.walk(sub, m2, o2, Sb, path + [f'{name}:{o}'])
                return
            r, a = st
            name = P.grounded(r, a, self.f)
            try:
                model, origin = self.step(model, origin, name)
            except (KeyError, ValueError) as e:
                self.out['errors'].append(f'{path}: {e}')
                return
            self.out['nodes'] += 1
            if a[0] == 'enter':
                self.out['entries'] += 1
            S = self.apply(S, r, a)
            path = path + [name]
            self.compare(model, origin, S, path)
        goal = convert(self.task['goal'])
        if not model.designated <= extensions(model, goal)[goal]:
            self.out['errors'].append(f'leaf {path}: goal does not hold')
        self.out['leaves'] += 1

    # -- the policy on every night -------------------------------------------------------

    def nights(self):
        firm = R.FIRM[self.f]
        grade_of = {r.name: r.grade for r in R.scouts(self.act)}
        worst = None
        for night in R.nights(self.act):
            groups = P.trace(self.tree, night)
            if groups is None:
                self.out['errors'].append(f'{R.tag(night)}: the policy has no branch for what is read')
                continue
            wrong = P.timeline(groups, self.pl.fleet)[2]
            if not wrong:
                continue
            if R.MODEL[self.f] == 'sensors':
                rank = sum(grade_of[r] for r in night[1])
                worst = rank if worst is None else min(worst, rank)
                if firm is None or rank < firm:
                    self.out['errors'].append(f'{R.tag(night)}: a bed-mover drove into {wrong}, on a night of rank {rank}')
        self.out['least_rank_wrong'] = worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--acts', type=int, nargs='*', default=sorted(R.FLEETS))
    ap.add_argument('--formulations', nargs='*', default=list(R.FORMULATIONS))
    ap.add_argument('--out')
    a = ap.parse_args()
    sys.setrecursionlimit(10000)
    bad = 0
    for act in a.acts:
        for f in a.formulations:
            c = Checker(act, f)
            r = c.run()
            bad += len(r['errors'])
            lw = r['least_rank_wrong']
            print(f'act {act} {f:10s}: {r["nodes"]} nodes, {r["leaves"]} leaves, {r["beliefs"]} cells checked, '
                  f'{r["entries"]} entries, ' +
                  ('no night drives a bed into a shut passage' if lw is None and R.MODEL[f] == 'sensors' else
                   f'a shut passage entered only on nights of rank {lw} and above' if lw is not None else
                   'nights checked for branches') + f', {len(r["errors"])} errors')
            for e in r['errors'][:5]:
                print('   ', e)
            if a.out:
                d = os.path.join(a.out, f'act{act}-{f}')
                os.makedirs(d, exist_ok=True)
                events = {n: x['events'] for n, x in c.task['actions'].items()}
                json.dump(P.to_policy(c.steps, events, f), open(os.path.join(d, 'policy.json'), 'w'))
                if not os.path.lexists(os.path.join(d, 'task.json')):
                    os.symlink(c.task_path, os.path.join(d, 'task.json'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
