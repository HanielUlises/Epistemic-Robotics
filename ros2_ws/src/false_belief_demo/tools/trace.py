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
The false-belief domain, traced update by update.

After every action this prints where the crate is and where each robot
believes it is, first and second order, and whether each robot still has a
consistent belief. With W* the designated worlds:

  B_i crate(b)       crate-at(b) at every world i considers possible from W*
  B_i B_j crate(b)   the same, one relation further
  consistent(i)      every world reachable from W* has a world i considers
                     possible; where one has none, i believes everything,
                     and the trace prints `none` and names the world

The product update is coordinated_attack_demo's trace.py, observability
evaluated once per state as plank and Aletheia do. It keeps an update that
leaves a robot without a world, as plank does; the planner, run with
consistent beliefs required, refuses one. Printing such a state is the point
of --actions on the untold floor.

  trace.py --task out/told/told.json --plan out/told/plan.json
  trace.py --task out/untold/untold.json --actions go-dock_picker \\
           relocate_mover_t1_t3 look_picker_t1:e-empty

An action may name its outcome after a colon; otherwise each sensing action
takes its first designated outcome, and the sequence stops at the first
action that does not apply.
"""

import argparse
import json
import sys


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

    PER_WORLD = False

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

        def kind_at(agent, w):
            classes = obs.get(agent)
            if not classes:
                raise ValueError(f'{agent} has no observability class')
            for label, cond in classes.items():
                if self.holds(w, cond.get('formula')):
                    return label
            raise ValueError(f'no observability class of {agent} holds at {w}')

        # plank's updater: the type whose condition holds at every designated
        # world, for the whole state.
        state_kind = {}
        for agent in self.agents:
            classes = obs.get(agent) or {}
            for label, cond in classes.items():
                if all(self.holds(w, cond.get('formula')) for w in self.designated):
                    state_kind[agent] = label

        def kind(agent, w):
            if Model.PER_WORLD or agent not in state_kind:
                return kind_at(agent, w)
            return state_kind[agent]

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

    def frontier(self, chain):
        """The worlds reached from W* along the agents of @p chain, or None
        when some world on the way has no successor for the next agent."""
        worlds = set(self.designated)
        for agent in chain:
            nxt = set()
            for w in worlds:
                seen = self.sees(agent, w)
                if not seen:
                    return None
                nxt.update(seen)
            worlds = nxt
        return worlds

    def belief(self, chain, bays):
        worlds = self.frontier(chain)
        if worlds is None:
            return 'none'
        held = [b for b in bays if all(f'crate-at_{b}' in self.labels[v] for v in worlds)]
        return held[0] if len(held) == 1 else '?'

    def collapse(self):
        """(agent, world) for the first world reachable from W* at which an
        agent has no world, or None."""
        seen, todo = set(self.designated), list(self.designated)
        while todo:
            w = todo.pop()
            for a in self.agents:
                succ = self.sees(a, w)
                if not succ:
                    return a, w
                for v in succ:
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
        return None

    def reflexive(self, agent):
        return all(w in self.sees(agent, w) for w in self.designated)


def bays_of(task):
    atoms = {a for labels in task['initial-state']['labels'].values() for a in labels}
    atoms |= {a for act in task['actions'].values()
              for eff in (act.get('effects') or {}).values() if eff for a in eff}
    return sorted(a.split('_', 1)[1] for a in atoms if a.startswith('crate-at_'))


def report(model, bays, indent):
    a, b = model.agents
    truth = [x for x in bays if all(f'crate-at_{x}' in model.labels[w] for w in model.designated)]
    print(f'{indent}crate {truth[0] if truth else "out"}   '
          f'B_{a} {model.belief([a], bays)}   B_{b} {model.belief([b], bays)}   '
          f'B_{b} B_{a} {model.belief([b, a], bays)}   B_{a} B_{b} {model.belief([a, b], bays)}')
    frame = '  '.join(f'{x} {"reflexive" if model.reflexive(x) else "not reflexive"}'
                      for x in model.agents)
    lost = model.collapse()
    state = f'{lost[0]} has no world at {lost[1]}: it believes everything' if lost \
        else 'every agent has a consistent belief'
    print(f'{indent}{len(model.worlds)} worlds, {len(model.designated)} designated; {frame}; '
          f'{state}')


def walk(node, task, model, bays, indent, problems):
    if node is None:
        ok = all(model.holds(w, task['goal']['formula']) for w in model.designated)
        print(f'{indent}goal {"holds" if ok else "FAILS"}')
        if not ok:
            problems.append('goal fails at a leaf')
        return
    act = task['actions'][node['action']]
    for br in node['branches']:
        outcome = act['designated'][br['event']] if len(act['designated']) > 1 else None
        if not model.applicable(act):
            print(f'{indent}{node["action"]}  NOT APPLICABLE')
            problems.append(node['action'] + ' does not apply')
            continue
        child = model.update(act, outcome)
        print(f'{indent}{node["action"]}' + (f'  [{outcome}]' if outcome else ''))
        report(child, bays, indent + '    ')
        if child.collapse():
            problems.append(node['action'] + ' leaves an agent without a consistent belief')
        walk(br['subtree'], task, child, bays, indent + '  ', problems)


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
    bays = bays_of(task)
    problems = []
    print('  initial')
    report(model, bays, '      ')

    if args.plan:
        plan = json.load(open(args.plan))
        if plan is None:
            print('the planner returned no plan')
            return 1
        walk(plan, task, model, bays, '  ', problems)
    else:
        for step in args.actions:
            name, _, outcome = step.partition(':')
            act = task['actions'][name]
            if not outcome and len(act['designated']) > 1:
                outcome = act['designated'][0]
            if not model.applicable(act):
                print(f'  {name}  NOT APPLICABLE')
                break
            model = model.update(act, outcome or None)
            print(f'  {name}' + (f'  [{outcome}]' if outcome else ''))
            report(model, bays, '      ')

    for msg in problems:
        print('PROBLEM: ' + msg)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
