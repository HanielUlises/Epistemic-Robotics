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
Does a policy keep a secret from an agent that knows the policy?

    opacity.py TASK.json PLAN.json --agent r3 --secret dest-x

The product update answers what an agent knows from the events it observes.
An agent that also knows the policy knows more: in each world the policy
produces its runs -- one, or one per choice of the events nobody controls, such
as the agent itself moving -- and the agent can ask which runs are compatible
with what it saw. This walks the policy in every designated world of the grounded task,
writes down what the agent observes at each step -- the event, if it observes
the action fully; the action, if partially; nothing, if it is oblivious --
and compares the traces of worlds the agent cannot tell apart at the start
and that differ on the secret. The policy is opaque to that agent iff every
such pair produces the same set of traces: then whatever the agent sees, and
whatever it chose to do, a world of either value of the secret explains it.

Preconditions are evaluated on their propositional part only; the policy is
taken to have been validated, so its modal conjuncts hold where it runs.
Exits non-zero when the policy is not opaque.
"""

import argparse
import json
import sys


def holds(f, world):
    """Propositional truth of a grounded formula; modalities are taken to hold."""
    if isinstance(f, str):
        return {'true': True, 'false': False}.get(f, f in world)
    if 'formula' in f and len(f) == 1:
        return holds(f['formula'], world)
    if 'modality-name' in f:
        return True
    c = f.get('connective')
    if c == 'and':
        return all(holds(g, world) for g in f['formulas'])
    if c == 'or':
        return any(holds(g, world) for g in f['formulas'])
    if c == 'not':
        return not holds(f['formula'], world)
    if c == 'imply':
        a, b = f['formulas']
        return (not holds(a, world)) or holds(b, world)
    raise ValueError(f'formula not understood: {f}')


def observed(action, event, agent, world):
    cases = action.get('observability-conditions', {}).get(agent)
    if not cases:
        return 'Fully'
    for kind, cond in cases.items():
        if holds(cond, world):
            return kind
    return 'Fully'


def runs(task, plan, world, agent):
    """Every run of the policy from one world, one per choice of the events no
    one controls -- another agent moving while the actor drives -- with the
    steps and what the agent saw of each."""
    out = []

    def go(node, world, steps, trace):
        if node is None:
            out.append((steps, [t for t in trace if t is not None]))
            return
        name = node['action']
        action = task['actions'][name]
        designated = action['designated']
        occurred = [e for e in designated if holds(action['preconditions'][e], world)]
        if not occurred:
            raise SystemExit(f'{name}: no designated event applies in {sorted(world)}')
        for event in occurred:
            kind = observed(action, event, agent, world)
            w = set(world)
            for atom, val in ((action.get('effects') or {}).get(event) or {}).items():
                (w.add if holds(val, world) else w.discard)(atom)
            branch = next((b for b in node['branches'] if designated[b['event']] == event), None)
            if branch is None and len(node['branches']) == 1:
                branch = node['branches'][0]   # one branch for events the actor cannot tell apart
            if branch is None:
                raise SystemExit(f'{name}: the policy has no branch for {event}')
            go(branch['subtree'], w, steps + [f'{name}:{event}'],
               trace + [{'Fully': f'{name}:{event}', 'Partially': name}.get(kind)])
    go(plan, set(world), [], [])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('plan')
    ap.add_argument('--agent', required=True)
    ap.add_argument('--secret', required=True)
    a = ap.parse_args()
    task, plan = json.load(open(a.task)), json.load(open(a.plan))
    init = task['initial-state']
    every = {}
    for w in init['designated']:
        every[w] = runs(task, plan, init['labels'][w], a.agent)
        label = ' '.join(sorted(x for x in init['labels'][w] if not x.startswith(('carrier', 'ally', 'watcher', 'at-'))))
        print(f'{w} [{label or "-"}]: {len(every[w])} run{"s" if len(every[w]) != 1 else ""}')
        for steps, trace in every[w]:
            print(f'   runs:  {" ".join(steps)}')
            print(f'   {a.agent} sees: {" ".join(trace) or "nothing"}')
    rel = init['relations'][a.agent]
    leaks = []
    for w in init['designated']:
        for v in rel[w]:
            if v <= w or v not in every:
                continue
            if (a.secret in init['labels'][w]) == (a.secret in init['labels'][v]):
                continue
            tw = {tuple(t) for _, t in every[w]}
            tv = {tuple(t) for _, t in every[v]}
            if tw != tv:
                leaks.append((w, v))
    if leaks:
        for w, v in leaks:
            print(f'NOT OPAQUE: {a.agent} cannot tell {w} from {v} at the start, they differ on {a.secret}, '
                  f'and the policy shows it different traces')
        return 1
    print(f'opaque: in every pair of worlds {a.agent} cannot tell apart and that differ on {a.secret}, '
          f'the policy shows it the same trace')
    return 0


if __name__ == '__main__':
    sys.exit(main())
