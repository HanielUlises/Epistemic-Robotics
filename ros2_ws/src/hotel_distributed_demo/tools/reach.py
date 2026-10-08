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
Every model a fleet can reach, and which conjuncts of the whole goal hold in
any of them.

    reach.py --task out/siloed/siloed.json --whole out/epistemic/epistemic.json
    reach.py --task out/pa-only/pa-only.json --whole out/epistemic/epistemic.json

Aletheia does not settle these fleets. Its relaxation finds the whole goal
reachable, and its search deepens without end because the robots can move in
circles. This walks the reachable states breadth first, every applicable
action and every outcome of it, until no new state appears. If no state
reached satisfies the whole goal at its designated worlds, no policy can,
since every leaf of a policy is a reachable state. With --witness the walk
stops at the first state that does, which is how the same search is checked
against a fleet that has a policy.

A state is a bisimulation-contracted model, identified by a canonical form:
each world's signature is refined from its valuation and whether it is
designated, by the multisets of its successors' signatures under each agent,
until the number of signatures stops growing. In a contracted model that
number is the number of worlds, so the sorted signatures name the state
independently of how its worlds happen to be numbered. Signatures are SHA-256
digests; a collision would merge two states, and none is expected.

The product update is coordinated_attack_demo's, as in tools/trace.py.
"""

import argparse
import hashlib
import json
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace import Model, conjuncts  # noqa: E402


def canonical(model):
    agents = sorted(model.agents)
    designated = set(model.designated)
    sig = {w: hashlib.sha256(repr((sorted(model.labels[w]), w in designated))
                             .encode()).hexdigest() for w in model.worlds}
    count = len(set(sig.values()))
    while True:
        nxt = {w: hashlib.sha256(repr((sig[w], [sorted(sig[v] for v in model.sees(a, w))
                                                  for a in agents])).encode()).hexdigest()
               for w in model.worlds}
        n = len(set(nxt.values()))
        sig = nxt
        if n == count:
            break
        count = n
    return hashlib.sha256(repr(sorted(sig.values())).encode()).hexdigest()


def successors(model, task):
    for name, act in task['actions'].items():
        if not model.applicable(act):
            continue
        outcomes = act['designated'] if len(act['designated']) > 1 else [None]
        for e in outcomes:
            child = model.update(act, e).contracted()
            if child.designated:
                yield name, e, child


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True)
    p.add_argument('--whole', required=True, help='a grounded task whose goal is the whole goal')
    p.add_argument('--limit', type=int, default=200000, help='states before giving up')
    p.add_argument('--witness', action='store_true',
                   help='stop at the first state where the whole goal holds, and say how deep')
    args = p.parse_args()

    task = json.load(open(args.task))
    parts = conjuncts(json.load(open(args.whole)))
    start = Model.of(task).contracted()

    seen = {canonical(start)}
    todo = deque([(start, 0)])
    held = {k: 0 for k in parts}          # states where the conjunct holds at every designated world
    somewhere = {k: 0 for k in parts}     # states where it holds at some designated world
    whole = 0
    depth = 0
    worlds = 0
    while todo:
        model, d = todo.popleft()
        depth = max(depth, d)
        worlds = max(worlds, len(model.worlds))
        ok = {k: all(model.holds(w, f) for w in model.designated) for k, f in parts.items()}
        for k, f in parts.items():
            held[k] += ok[k]
            somewhere[k] += any(model.holds(w, f) for w in model.designated)
        whole += all(ok.values())
        if args.witness and all(ok.values()):
            print(f'the whole goal holds in a state {d} actions from the start, '
                  f'found after {len(seen)} states')
            print('the whole goal is reachable')
            return 0
        for _, _, child in successors(model, task):
            key = canonical(child)
            if key not in seen:
                seen.add(key)
                if len(seen) > args.limit:
                    print(f'gave up after {args.limit} states')
                    return 2
                todo.append((child, d + 1))

    print(f'{len(seen)} states reachable, the deepest {depth} actions from the start, '
          f'the largest of {worlds} worlds')
    for k in parts:
        print(f'  {k:10s} holds in {held[k]} of them, at some designated world of {somewhere[k]}')
    print(f'  the whole goal holds in {whole}')
    print('the whole goal is unreachable' if whole == 0 else 'the whole goal is reachable')
    return 0


if __name__ == '__main__':
    sys.exit(main())
