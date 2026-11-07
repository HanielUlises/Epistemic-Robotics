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

    policy_check.py OUT          OUT/act<k>-<formulation>/{task.json, policy.json}

Each policy is replayed through the product update of the grounded task
(the repository's tools/plan_trace.py, an implementation independent of
ePlanSys's and Aletheia's), along every branch:

  1. every action applies: its designated events' preconditions cover the
     designated worlds, the guards among them;
  2. every branch the policy names is produced, and every leaf meets the goal;
  3. at every node, what each courier believes, read off the model with the
     grade formula of tools/model.py, is what tools/ranks.py says the
     courier believes on the worlds it cannot rule out; and every door a
     courier believes open, or shut, by the one is so by the other.

The third is the claim that the grades revise as Spohn conditioning does:
the model never changes a grade, and the beliefs come out right anyway.
"""

import json
import os
import sys

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


def grade(model, w):
    for k in range(7):
        if f'g{k}' in model.labels[w]:
            return k
    raise ValueError(f'{w} has no grade')


def believed_doors(model, agent, w):
    """The doors agent believes open and believes shut at w, from the model alone."""
    cell = model.image(agent, w)
    m = min(grade(model, v) for v in cell)
    top = [v for v in cell if grade(model, v) == m]
    opened = {d for d in R.DOORS if all(f'open-{d}' in model.labels[v] for v in top)}
    shut = {d for d in R.DOORS if all(f'open-{d}' not in model.labels[v] for v in top)}
    return opened, shut


def walk(task, node, model, agents, origin, out, path):
    """Replay a subtree; origin maps the model's worlds to the problem's initial worlds."""
    if node is None:
        goal = convert(task['goal'])
        if not model.designated <= extensions(model, goal)[goal]:
            out['errors'].append(f'leaf {path}: goal does not hold')
        out['leaves'] += 1
        return
    action = task['actions'].get(node['action'])
    if action is None:
        out['errors'].append(f'{path}: no action {node["action"]}')
        return
    events = [action['events'][b['event']] for b in node['branches']]
    covered = set()
    for e in action['designated']:
        f = convert(action['preconditions'][e])
        covered |= extensions(model, f)[f]
    if not model.designated <= covered:
        out['errors'].append(f'{path}: {node["action"]} does not apply')
        return
    out['nodes'] += 1
    for b, e in zip(node['branches'], events):
        m2, pairs = update(model, action, agents, [e])
        if not m2.designated:
            out['errors'].append(f'{path}: {node["action"]}/{e} is not produced')
            continue
        origin2 = {w: origin[pairs[w][0]] for w in m2.worlds}
        check_beliefs(m2, origin2, out, path + [node['action'] + ('' if len(events) == 1 else ':' + e)])
        walk(task, b['subtree'], m2, agents, origin2, out, path + [node['action']])


def check_beliefs(model, origin, out, path):
    act, f = out['act'], out['formulation']
    worlds = R.worlds(act)
    for agent in P.ROBOTS[act]:
        for w in model.designated:
            cell = {worlds[origin[v]] for v in model.image(agent, w)}
            after = 'round-done' in model.labels[w]
            ref_open = {d for d in R.DOORS if R.believes(f, cell, lambda x, d=d: R.open_at(x, d, after))}
            ref_shut = {d for d in R.DOORS if R.believes(f, cell, lambda x, d=d: not R.open_at(x, d, after))}
            got = believed_doors(model, agent, w)
            out['beliefs'] += 1
            if got != (ref_open, ref_shut):
                out['errors'].append(f'{path}: {agent} believes {got}, the ranks say {(ref_open, ref_shut)}')


def check(task, policy, act, formulation):
    model = model_of(task['initial-state'])
    agents = task['language']['agents']
    origin = {w: int(w[1:]) for w in model.worlds}
    out = {'act': act, 'formulation': formulation, 'nodes': 0, 'leaves': 0, 'beliefs': 0, 'errors': []}
    check_beliefs(model, origin, out, ['initial'])
    walk(task, policy, model, agents, origin, out, [])
    return out


def main():
    root = sys.argv[1]
    bad = 0
    for act in (1, 2, 3):
        for f in R.FORMULATIONS:
            d = os.path.join(root, f'act{act}-{f}')
            task = json.load(open(os.path.join(d, 'task.json')))
            policy = json.load(open(os.path.join(d, 'policy.json')))
            r = check(task, policy, act, f)
            bad += len(r['errors'])
            print(f'act {act} {f:13s}: {r["nodes"]} nodes, {r["leaves"]} leaves, {r["beliefs"]} beliefs checked, '
                  f'{len(r["errors"])} errors')
            for e in r['errors'][:5]:
                print('   ', e)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
