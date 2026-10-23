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
The planning task of a robot that has found a task, as an EPDDL problem of
epddl-workspace/gossip-coverage/gossip-coverage.epddl, and its policy from
Aletheia.

    Instance(agents, finder, k, case, knowers, may_be_busy, max_busy)

agents      the robots, in the order the planner sees them: the finder, then
            the others by the paper's own bid for them (the least time to tell
            them and bring them to the site first), so that among policies of
            the same depth the planner's first is the paper's preference.
knowers     the robots that know of the task: the finder and those in contact
            with it.
may_be_busy the robots the finder cannot rule out being away on other work.
max_busy    at most this many of them are.
case        1: every robot must come to know of the task (the paper's Case I,
            where the rest go home); 2: the task done is the goal.
"""

import itertools
import json
import os
import subprocess
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# the EPDDL: installed beside the tools in the package's share, or in the
# repository's epddl-workspace
_INSTALLED = os.path.normpath(os.path.join(HERE, '..', '..', 'epddl'))
WORKSPACE = os.environ.get('GOSSIP_EPDDL', _INSTALLED if os.path.isdir(_INSTALLED) else os.path.normpath(
    os.path.join(HERE, '..', '..', '..', '..', '..', 'epddl-workspace', 'gossip-coverage')))
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
INTERMEDIATE = os.environ.get('INTERMEDIATE', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))

PROBLEM = """\
(define (problem {name})

  ;; Written by tools/gossip/epddl.py. {about}

  (:domain gossip-coverage)

  (:requirements
    :facts :equality :list-comprehensions :finitary-S5-theories :modal-goals
    :knowing-whether :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:facts-init
    (need{k})
    {near}
  )

  (:init
    (:and
      (task)
      ([C. All] (not (done)))
      ([C. All] (c0))
      {counter}
      {needs}
      {static}
      ;; who knows of the task
      {knows}
      ;; who may be busy, and that each knows whether it is
      {busy}
    )
  )

  (:goal {goal})
)
"""


class Instance:

    def __init__(self, agents, finder, k, case, knowers=None, may_be_busy=(), max_busy=None, name=None,
                 near=None):
        self.agents = list(agents)
        # who may call whom: every pair, unless a neighbour relation is given
        self.near = sorted(set(near)) if near is not None else [
            (x, y) for x in self.agents for y in self.agents if x != y]
        self.finder = finder
        self.k = k
        self.case = case
        self.knowers = set(knowers or [finder])
        self.may_be_busy = [a for a in self.agents if a in set(may_be_busy) and a not in self.knowers]
        self.max_busy = len(self.may_be_busy) if max_busy is None else max_busy
        self.name = name or f'gossip-n{len(self.agents)}-k{k}-c{case}'

    def text(self):
        a = self.agents
        counter = ' '.join(f'([C. All] (not (c{i})))' for i in range(1, 9))
        needs = ' '.join(f'([C. All] (not (need{i})))' for i in range(1, 9) if i != self.k)
        static = []
        for x in a:
            static += [f'([C. All] (not (out {x})))', f'([C. All] (not (sited {x})))']
        for x, y in itertools.product(a, repeat=2):
            static += [f'([C. All] (not (by {x} {y})))', f'([C. All] (not (called {x} {y})))']
        knows = []
        for x in a:
            if x in self.knowers:
                knows.append(f'([C. All] ([Kw. {x}] (task)))')
            else:
                knows.append(f'([C. All] (<Kw. {x}> (task)))')
        busy = []
        for x in a:
            if x in self.may_be_busy:
                busy.append(f'([C. All] ([Kw. {x}] (busy {x})))')
            else:
                busy.append(f'([C. All] (not (busy {x})))')
        if self.may_be_busy and self.max_busy < len(self.may_be_busy):
            for group in itertools.combinations(self.may_be_busy, self.max_busy + 1):
                busy.append(f"([C. All] (not (and {' '.join(f'(busy {x})' for x in group)})))")
        if self.case == 1:
            goal = '(and (done) ' + ' '.join(f'([{x}] (task))' for x in a) + ')'
        else:
            goal = '(done)'
        about = (f'{self.finder} has found a task that needs {self.k} of {len(a)} robots; '
                 f"{', '.join(sorted(self.knowers))} know of it; "
                 + (f"{', '.join(self.may_be_busy)} may be busy, at most {self.max_busy}."
                    if self.may_be_busy else 'no robot is busy.'))
        near = ' '.join(f'(near {x} {y})' for x, y in self.near)
        return PROBLEM.format(name=self.name, about=about, agents=' '.join(a), k=self.k, counter=counter, needs=needs,
                              near=near,
                              static='\n      '.join(static), knows='\n      '.join(knows),
                              busy='\n      '.join(busy), goal=goal)


def export(inst, out_dir):
    """plank export of the instance; the grounded task's path."""
    os.makedirs(out_dir, exist_ok=True)
    prob = os.path.join(out_dir, f'{inst.name}.epddl')
    open(prob, 'w').write(inst.text())
    r = subprocess.run([PLANK, 'export', '-d', os.path.join(WORKSPACE, 'gossip-coverage.epddl'), '-p', prob,
                        '-l', INTERMEDIATE, os.path.join(WORKSPACE, 'gossip.epddl'), '-o', out_dir],
                       capture_output=True, text=True)
    task = os.path.join(out_dir, f'{inst.name}.json')
    if r.returncode != 0 or not os.path.exists(task):
        raise RuntimeError(f'plank export failed: {r.stdout[-2000:]} {r.stderr[-2000:]}')
    return task


def plan(task, out_dir, timeout=120.0, extra=()):
    """Aletheia's policy for a grounded task: (policy, seconds, log)."""
    pol = os.path.join(out_dir, 'policy.json')
    t = time.time()
    cmd = [PLANNER, '--task', task, '--plan', pol, '--conditional', '--consistent-beliefs',
           '--timeout', str(int(timeout))] + list(extra)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 30)
    dt = time.time() - t
    if not os.path.exists(pol):
        return None, dt, r.stdout + r.stderr
    return json.load(open(pol)), dt, r.stdout + r.stderr


def solve(inst, timeout=120.0, keep=None, extra=()):
    d = keep or tempfile.mkdtemp(prefix='gossip-')
    task = export(inst, d)
    pol, dt, log = plan(task, d, timeout, extra)
    return pol, dt, log, task
