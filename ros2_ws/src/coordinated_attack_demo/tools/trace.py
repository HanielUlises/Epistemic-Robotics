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
The coordinated attack, traced update by update.

After every action this prints how deep the robots' mutual knowledge of where
the load is goes: the largest k with E^k job, where E is "both robots know".
Common knowledge is E^k for every k at once, so the depth is computed and
not probed one k at a time. From the designated worlds, a breadth-first walk over the union
of the two relations finds the nearest world where the load is elsewhere; if
that world is L steps away, E^(L-1) holds and E^L does not, and the walk
itself is printed, as the chain of "south considers ... north considers ..."
that keeps common knowledge from holding. No such world means common
knowledge.

The warehouse scenario's show_plan.py traces single-agent modalities and
takes an agent's first observability class whatever its condition. This
domain needs both C and an announcement whose audience depends on where the
robots stand, so the update here evaluates each agent's observability
condition at the world it is applied to.

  trace.py --task out/beacon/beacon.json --plan out/beacon/plan.json
  trace.py --task out/radio/radio.json --actions read-order_south_s1 \\
           tell_south_north_s1 ack_north_south_s1 ack2_south_north_s1 \\
           ack3_north_south_s1 lift_s1

With --actions each sensing action takes its first designated outcome, and
the sequence stops at the first action that does not apply.
"""

import argparse
import json
import sys
from collections import deque


class Model:

    def __init__(self, worlds, relations, labels, designated):
        self.worlds = list(worlds)
        self.relations = relations          # agent -> world -> [worlds]
        self.labels = {w: set(a) for w, a in labels.items()}
        self.designated = list(designated)

    @staticmethod
    def of(task):
        s = task['initial-state']
        return Model(s['worlds'], s['relations'], s['labels'], s['designated'])

    @property
    def agents(self):
        return list(self.relations)

    def sees(self, agent, w):
        return self.relations[agent].get(w, [])

    def reach(self, group, w):
        """Worlds reachable from @p w in one or more steps of the group's
        relations: the worlds C quantifies over."""
        seen, todo = set(), [w]
        while todo:
            u = todo.pop()
            for a in group:
                for v in self.sees(a, u):
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
        return seen

    def holds(self, w, node):
        if node is None:
            return True
        if isinstance(node, str):
            if node == 'true':
                return True
            if node == 'false':
                return False
            return node in self.labels[w]

        if 'modality-name' in node:
            name = node['modality-name']
            group = node['modality-index']
            inner = node['formula']
            if name == 'C.box':
                return all(self.holds(v, inner) for v in self.reach(group, w))
            if name == 'C.diamond':
                return any(self.holds(v, inner) for v in self.reach(group, w))
            # A box indexed by several agents is "everyone in the group".
            reach = [v for a in group for v in self.sees(a, w)]
            knows = all(self.holds(v, inner) for v in reach)
            if name == 'box':
                return knows
            if name == 'diamond':
                return any(self.holds(v, inner) for v in reach)
            settled = knows or all(not self.holds(v, inner) for v in reach)
            if name == 'Kw.box':
                return settled
            if name == 'Kw.diamond':
                return not settled
            raise ValueError('unknown modality ' + name)

        c = node.get('connective')
        if c == 'not':
            return not self.holds(w, node['formula'])
        if c == 'and':
            return all(self.holds(w, f) for f in node['formulas'])
        if c == 'or':
            return any(self.holds(w, f) for f in node['formulas'])
        if c == 'imply':
            a, b = node['formulas']
            return not self.holds(w, a) or self.holds(w, b)
        raise ValueError('unreadable formula ' + json.dumps(node))

    def update(self, action, outcome=None):
        """Product update. @p outcome restricts the designated events to one,
        which is how a branch of a policy is followed."""
        events = action['events']
        pre = action.get('preconditions') or {}
        eff = action.get('effects') or {}
        obs = action.get('observability-conditions') or {}
        rel = action['relations']

        pairs = [(w, e) for w in self.worlds for e in events
                 if self.holds(w, (pre.get(e) or {}).get('formula'))]
        name = {p: f'{p[0]}.{p[1]}' for p in pairs}

        labels = {}
        for w, e in pairs:
            value = set(self.labels[w])
            for atom, assignment in (eff.get(e) or {}).items():
                if self.holds(w, assignment.get('formula')):
                    value.add(atom)
                else:
                    value.discard(atom)
            labels[name[(w, e)]] = value

        def kind(agent, w):
            classes = obs.get(agent)
            if not classes:
                raise ValueError(f'{agent} has no observability class')
            for label, cond in classes.items():
                if self.holds(w, cond.get('formula')):
                    return label
            raise ValueError(f'no observability class of {agent} holds at {w}')

        relations = {}
        for a in self.agents:
            relations[a] = {}
            for w, e in pairs:
                table = rel[kind(a, w)]
                relations[a][name[(w, e)]] = [
                    name[(v, f)] for v, f in pairs
                    if v in self.sees(a, w) and f in table.get(e, [])]

        chosen = [outcome] if outcome else action['designated']
        designated = [name[(w, e)] for w, e in pairs
                      if w in self.designated and e in chosen]
        return Model([name[p] for p in pairs], relations, labels, designated)

    def applicable(self, action):
        """Every designated world has a designated event that can happen
        there. A sensing action applies as a whole; which outcome happens is
        then a branch."""
        pre = action.get('preconditions') or {}
        return all(any(self.holds(w, (pre.get(e) or {}).get('formula'))
                       for e in action['designated']) for w in self.designated)

    def depth(self, atom):
        """(k, chain): E^k atom holds and E^(k+1) does not, with the chain of
        agents and worlds that reaches a world where it fails; k is None and
        the chain empty when the atom is common knowledge."""
        if not all(atom in self.labels[w] for w in self.designated):
            return -1, []
        parent = {w: None for w in self.designated}
        todo = deque((w, 0) for w in self.designated)
        while todo:
            u, d = todo.popleft()
            for a in self.agents:
                for v in self.sees(a, u):
                    if v in parent:
                        continue
                    parent[v] = (u, a)
                    if atom not in self.labels[v]:
                        chain, x = [], v
                        while parent[x] is not None:
                            x, ag = parent[x]
                            chain.append(ag)
                        return d, chain[::-1]
                    todo.append((v, d + 1))
        return None, []


def ladder(model, atom, indent):
    k, chain = model.depth(atom)
    knows = {a: all(atom in model.labels[v]
                    for w in model.designated for v in model.sees(a, w))
             for a in model.agents}
    who = '  '.join(f'K_{a} {"yes" if knows[a] else "no "}' for a in model.agents)
    if k is None:
        depth = 'C  (common knowledge)'
    elif k < 0:
        depth = 'false'
    else:
        depth = f'E^{k}' + ('' if k else '  (not everyone knows)')
    print(f'{indent}{atom:8s} {who}   depth {depth}')
    if chain:
        print(f'{indent}{"":8s} broken by: ' + ' considers '.join(chain)
              + f' considers a world where not {atom}')


def stand_of(model):
    """The stand the load is on, once the branch has settled it."""
    jobs = {a for w in model.worlds for a in model.labels[w]
            if a.startswith('job_')}
    return sorted(a for a in jobs
                  if all(a in model.labels[w] for w in model.designated))


def walk(node, task, model, indent, problems):
    if node is None:
        ok = model.holds(model.designated[0], task['goal']['formula']) and \
            all(model.holds(w, task['goal']['formula']) for w in model.designated)
        print(f'{indent}goal {"holds" if ok else "FAILS"}')
        if not ok:
            problems.append('goal fails at a leaf')
        return
    act = task['actions'][node['action']]
    branches = node['branches']
    for b in branches:
        outcome = act['designated'][b['event']] if len(branches) > 1 or \
            len(act['designated']) > 1 else None
        label = f'  [{outcome}]' if outcome and len(act['designated']) > 1 else ''
        if not model.applicable(act):
            print(f'{indent}{node["action"]}{label}  NOT APPLICABLE')
            problems.append(node['action'] + ' does not apply')
            continue
        child = model.update(act, outcome)
        print(f'{indent}{node["action"]}{label}')
        for atom in stand_of(child):
            ladder(child, atom, indent + '    ')
        walk(b['subtree'], task, child, indent + '  ', problems)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--plan')
    g.add_argument('--actions', nargs='+')
    args = p.parse_args()

    task = json.load(open(args.task))
    model = Model.of(task)
    problems = []

    if args.plan:
        plan = json.load(open(args.plan))
        if plan is None:
            print('the planner returned no plan')
            return 1
        walk(plan, task, model, '  ', problems)
    else:
        for name in args.actions:
            act = task['actions'][name]
            outcome = act['designated'][0] if len(act['designated']) > 1 else None
            if not model.applicable(act):
                print(f'  {name}  NOT APPLICABLE')
                lift = (act.get('preconditions') or {})
                for e in act['designated']:
                    f = (lift.get(e) or {}).get('formula')
                    w = next(w for w in model.designated if not model.holds(w, f))
                    print(f'      precondition of {e} fails at {w}')
                break
            model = model.update(act, outcome)
            print(f'  {name}' + (f'  [{outcome}]' if outcome else ''))
            for atom in stand_of(model):
                ladder(model, atom, '      ')
            print(f'      {len(model.worlds)} worlds, '
                  f'{len(model.designated)} designated')

    for msg in problems:
        print('PROBLEM: ' + msg)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
