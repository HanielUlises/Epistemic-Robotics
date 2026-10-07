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
The muddy robots, traced update by update.

    trace.py --task out/pa-n4/pa-n4.json --plan out/pa-n4/plan.json
    trace.py --task out/pa-n4/pa-n4.json --faults r1 r2 r3 --bells 4
    trace.py --task out/silent-n4/silent-n4.json --faults r1 r2 r3 --bells 4

With --plan every branch of the policy is walked, and every leaf has to
satisfy the goal at each of its designated worlds. With --faults the actual
world is the one with exactly those lamps lit: the announcement is made if
the floor has a public address, and then the bell rings up to --bells times,
each time with the outcome that world gives it, until somebody leaves.

After every update the trace prints, for each robot, whether it knows that it
is faulty, knows that it is not, or does not know; and the depth of the
robots' mutual knowledge of "some lamp is lit": the largest k with E^k of
it, found as coordinated_attack_demo's trace finds the depth of job(s), by a
breadth-first walk from the designated worlds to the nearest world with no
lamp lit.

The product update is coordinated_attack_demo's, in its tools/trace.py.
"""

import argparse
import importlib.util
import json
import os
import sys
from collections import deque


def _trace_module():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', '..', 'coordinated_attack_demo', 'tools')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory('coordinated_attack_demo'), 'tools'))
    except Exception:   # noqa: BLE001  outside a sourced workspace
        pass
    for path in candidates:
        source = os.path.join(path, 'trace.py')
        if os.path.exists(source):
            spec = importlib.util.spec_from_file_location('ca_trace', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError('coordinated_attack_demo/tools/trace.py not found')


Model = _trace_module().Model


def lit(labels):
    return any(a.startswith('faulty_') for a in labels)


def some_depth(model):
    """(k, chain): E^k of "some lamp is lit" and not E^(k+1); k is None for C,
    and -1 when no lamp is lit at a designated world."""
    if not all(lit(model.labels[w]) for w in model.designated):
        return -1, []
    parent = {w: None for w in model.designated}
    todo = deque((w, 0) for w in model.designated)
    while todo:
        u, d = todo.popleft()
        for a in model.agents:
            for v in model.sees(a, u):
                if v in parent:
                    continue
                parent[v] = (u, a)
                if not lit(model.labels[v]):
                    chain, x = [], v
                    while parent[x] is not None:
                        x, who = parent[x]
                        chain.append(who)
                    return d, chain[::-1]
                todo.append((v, d + 1))
    return None, []


def status(model, agent):
    """What the robot knows of its own lamp: 'faulty' or 'clean' when it is
    the same at every designated world; 'knows' when the robot knows whether
    at each of them but the answer differs between them, as at a leaf of the
    policy that covers several fault patterns; '?' otherwise."""
    atom = f'faulty_{agent}'
    answers = set()
    for w in model.designated:
        seen = {atom in model.labels[v] for v in model.sees(agent, w)}
        if len(seen) > 1:
            return '?'
        answers |= seen
    if answers == {True}:
        return 'faulty'
    if answers == {False}:
        return 'clean'
    return 'knows'


def reachable(model):
    """The worlds some chain of the robots' relations reaches from the
    designated ones: the part of the model that still matters."""
    seen = set(model.designated)
    todo = list(model.designated)
    while todo:
        u = todo.pop()
        for a in model.agents:
            for v in model.sees(a, u):
                if v not in seen:
                    seen.add(v)
                    todo.append(v)
    return len(seen)


def report(model, indent):
    who = '  '.join(f'{a} {status(model, a):6s}' for a in model.agents)
    k, chain = some_depth(model)
    depth = 'C' if k is None else f'E^{k}' if k >= 0 else 'false'
    print(f'{indent}{who}   "some lamp lit": {depth}'
          + (f'  (blocked: {" > ".join(chain)})' if chain else ''))
    print(f'{indent}{reachable(model)} worlds reachable, {len(model.designated)} designated')


def walk(node, task, model, indent, problems, leaves):
    if node is None:
        goal = task['goal']['formula']
        ok = all(model.holds(w, goal) for w in model.designated)
        faults = sorted({tuple(sorted(a[7:] for a in model.labels[w] if a.startswith('faulty_')))
                         for w in model.designated})
        print(f'{indent}goal {"holds" if ok else "FAILS"} at {len(model.designated)} '
              f'designated worlds: {", ".join("+".join(f) for f in faults)}')
        leaves.append(len(model.designated))
        if not ok:
            problems.append('goal fails at a leaf')
        return
    act = task['actions'][node['action']]
    for b in node['branches']:
        outcome = act['designated'][b['event']] if len(act['designated']) > 1 else None
        if not model.applicable(act):
            problems.append(node['action'] + ' does not apply')
            print(f'{indent}{node["action"]}  NOT APPLICABLE')
            continue
        child = model.update(act, outcome).contracted()
        print(f'{indent}{node["action"]}' + (f'  [{outcome}]' if outcome else ''))
        report(child, indent + '    ')
        walk(b['subtree'], task, child, indent + '  ', problems, leaves)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--plan')
    g.add_argument('--faults', nargs='*')
    p.add_argument('--bells', type=int, default=4)
    args = p.parse_args()

    task = json.load(open(args.task))
    model = Model.of(task)
    problems = []

    if args.plan:
        plan = json.load(open(args.plan))
        if plan is None:
            print('the planner returned no plan')
            return 1
        print('initial model')
        report(model, '    ')
        leaves = []
        walk(plan, task, model, '  ', problems, leaves)
        print(f'{len(leaves)} leaves, {sum(leaves)} fault patterns covered')
    else:
        lamps = {f'faulty_{a}' for a in args.faults}
        actual = [w for w in model.designated
                  if {a for a in model.labels[w] if a.startswith('faulty_')} == lamps]
        if not actual:
            raise SystemExit(f'no designated world with exactly {sorted(lamps)} lit')
        model = Model(model.worlds, model.relations, model.labels, actual)
        print(f'lamps lit: {", ".join(args.faults)}')
        report(model, '    ')
        announce = next(a for n, a in task['actions'].items() if n.startswith('announce'))
        bell = next(a for n, a in task['actions'].items() if n.startswith('bell'))
        if model.applicable(announce):
            model = model.update(announce).contracted()
            print('  announce')
            report(model, '    ')
        else:
            print('  announce  NOT APPLICABLE: no public address')
        for k in range(1, args.bells + 1):
            outcome = next(e for e in bell['designated']
                           if model.holds(model.designated[0],
                                          (bell['preconditions'].get(e) or {}).get('formula')))
            model = model.update(bell, outcome).contracted()
            print(f'  bell {k}  [{outcome}]')
            report(model, '    ')
            if outcome == 'e-leave':
                break

    for msg in problems:
        print('PROBLEM: ' + msg)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
