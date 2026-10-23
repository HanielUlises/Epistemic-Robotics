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
What a policy of gossip-coverage.epddl does to what every robot knows, and
whether every robot can carry it out on what it knows.

    check_policy.py TASK.json POLICY.json [--json OUT] [--quiet]

For each designated world of the initial model the policy is followed along
the branch that world takes, by the product update of the repository's
tools/plan_trace.py, and the following are evaluated at the actual world with
tools/model_check.py.

The paper's update rules (Section 5.3.1), read in the language of the domain:

  R1 told       after a call i:j that is met, K_j task: the announcement
                reaches the robot in contact, eq. (18)'s B_i B_j q'_i read
                for the task.
  R2 mutual     and K_i K_j task and K_j K_i task: each knows the other knows.
  R3 unaffected for every robot k out of contact, its knowledge of the task is
                what it was, and k considers both possible: that j was told
                and that it was not (not K_k K_j task, not K_k not K_j task):
                eq. (18)'s "disconnected robots' knowledge is not impacted",
                read without a false belief.
  R4 absent     after a call i:j that finds nobody, K_i busy_j, and nobody
                else learns it: the ruleset's "j is not available".
  R5 empathy    every robot j that may be busy knows that nobody else knows
                whether it is: K_j not Kw_i busy_j, in the initial state; it
                is what makes j's particle the only place to seek it.
  R6 goal       at the end: done, and in Case I every robot knows of the task.

Implementability (the paper's joint policy, Section 5.3.2: "robots need to map
indistinguishable epistemic states to the same actions"): a robot's view at a
point of a branch is the sequence of steps it observed fully. For every robot
and every two designated worlds, wherever its views are the same, its next
action must be the same. A violation is printed with the two worlds.
"""

import argparse
import itertools
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_TOOLS = os.environ.get('ER_TOOLS', os.path.join(HERE, '..', '..', '..', '..', 'tools'))
sys.path.insert(0, REPO_TOOLS)
from model_check import extensions, model_of, parse  # noqa: E402
from plan_trace import convert, observability, update  # noqa: E402


def holds_at(model, world, text):
    f = parse(text)
    return world in extensions(model, f)[f]


def branch_event(action, branch):
    e = branch.get('event')
    return action['designated'][e] if isinstance(e, int) else e


def follow(task, policy, world, agents):
    """The steps the policy takes from one designated world: each with the
    model before it, the event, and the model and actual world after it."""
    model = model_of(task['initial-state'])
    actual = world
    steps = []
    node = policy
    while node is not None:
        action = task['actions'][node['action']]
        chosen = None
        for b in node['branches']:
            e = branch_event(action, b)
            pre = convert(action['preconditions'][e])
            if actual in extensions(model, pre)[pre]:
                chosen = (b, e)
                break
        if chosen is None:
            raise SystemExit(f"{node['action']}: no branch fits world {world}")
        b, e = chosen
        classes = {a: observability(action, a, model, actual) for a in agents}
        new, pairs = update(model, action, agents, [e])
        new_actual = next(n for n, (w, ev) in pairs.items() if w == actual and ev == e)
        steps.append({'action': node['action'], 'event': e, 'classes': classes,
                      'before': (model, actual), 'after': (new, new_actual)})
        model, actual = new, new_actual
        node = b['subtree']
    return steps


def parts(action):
    head, *args = action.split('_')
    return head, args


def check(task, policy, case=None):
    agents = task['language']['agents'] if 'language' in task else task['agents']
    designated = sorted(task['initial-state']['designated'], key=lambda w: int(w[1:]))
    init = model_of(task['initial-state'])
    if case is None:
        goal = json.dumps(task.get('goal'))
        case = 1 if '"task"' in goal and 'K' in goal else 2
    rules = {r: [0, 0] for r in ('R1 told', 'R2 mutual', 'R3 unaffected', 'R4 absent', 'R5 empathy', 'R6 goal')}
    worlds = []
    views = {a: {} for a in agents}
    violations = []

    def tally(rule, ok, where):
        rules[rule][0] += 1
        if not ok:
            rules[rule][1] += 1
            violations.append(f'{rule} fails {where}')

    for w in designated:
        # R5 empathy, in the initial state
        for j in agents:
            if f'busy_{j}' not in task['language']['atoms']:
                continue
            for i in agents:
                if i == j:
                    continue
                # only robots whose busyness is open to the others
                if holds_at(init, w, f'(Kw {i} busy_{j})'):
                    continue
                tally('R5 empathy', holds_at(init, w, f'(K {j} (not (Kw {i} busy_{j})))'), f'{j}/{i} in {w}')
        steps = follow(task, policy, w, agents)
        trail = []
        seen = {a: [] for a in agents}
        for st in steps:
            head, args = parts(st['action'])
            actor = args[0]
            view = tuple(seen[actor])
            prev = views[actor].get(view)
            if prev is not None and prev[0] != st['action']:
                violations.append(f'implementability: {actor} sees {view} and does {prev[0]} in {prev[1]} '
                                  f'but {st["action"]} in {w}')
            views[actor].setdefault(view, (st['action'], w))
            model, actual = st['after']
            if head in ('call', 'await'):
                i, j = args
                met = st['event'].startswith(('e-met', 'e-await'))
                if met:
                    tally('R1 told', holds_at(model, actual, f'(K {j} task)'), f'{st["action"]} in {w}')
                    tally('R2 mutual', holds_at(model, actual, f'(and (K {i} (K {j} task)) (K {j} (K {i} task)))'),
                          f'{st["action"]} in {w}')
                else:
                    tally('R4 absent', holds_at(model, actual, f'(K {i} busy_{j})')
                          and all(not holds_at(model, actual, f'(Kw {x} busy_{j})') for x in agents
                                  if x not in (i, j)), f'{st["action"]} in {w}')
                for k in agents:
                    if k in (i, j):
                        continue
                    before_m, before_a = st['before']
                    same = holds_at(before_m, before_a, f'(K {k} task)') == holds_at(model, actual, f'(K {k} task)')
                    open_ = (not holds_at(model, actual, f'(K {k} (K {j} task))')
                             or holds_at(before_m, before_a, f'(K {k} (K {j} task))'))
                    tally('R3 unaffected', same and open_, f'{k} at {st["action"]} in {w}')
            for a in agents:
                if st['classes'][a] == 'Fully':
                    seen[a].append((st['action'], st['event']))
            trail.append({'action': st['action'], 'event': st['event'],
                          'knows_task': [a for a in agents if holds_at(model, actual, f'(K {a} task)')],
                          'considered': {a: len(model.image(a, actual)) for a in agents}})
        model, actual = steps[-1]['after'] if steps else (init, w)
        goal = holds_at(model, actual, 'done')
        if case == 1:
            goal = goal and all(holds_at(model, actual, f'(K {a} task)') for a in agents)
        tally('R6 goal', goal, f'in {w}')
        worlds.append({'world': w, 'busy': sorted(a for a in agents if f'busy_{a}' in init.labels[w]),
                       'steps': trail})
    return {'agents': agents, 'case': case, 'rules': rules, 'violations': violations, 'worlds': worlds}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('policy')
    ap.add_argument('--case', type=int)
    ap.add_argument('--json')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    task, policy = json.load(open(a.task)), json.load(open(a.policy))
    out = check(task, policy, a.case)
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)
    if not a.quiet:
        for w in out['worlds']:
            print(f"world {w['world']}: busy {w['busy'] or 'none'}")
            for s in w['steps']:
                print(f"  {s['action']:<16} {s['event']:<18} knows the task: {', '.join(s['knows_task'])}")
    for r, (n, bad) in out['rules'].items():
        print(f'{r:<14} {n - bad}/{n} hold')
    impl = [v for v in out['violations'] if v.startswith('implementability')]
    print(f"implementability: {'every robot can follow the policy on what it observes' if not impl else impl}")
    others = [v for v in out['violations'] if not v.startswith('implementability')]
    for v in others[:20]:
        print('  ' + v)
    return 1 if out['violations'] else 0


if __name__ == '__main__':
    sys.exit(main())
