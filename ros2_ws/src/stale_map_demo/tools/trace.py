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
The stale-maps domain, traced update by update.

After every action this prints the floor and, for every robot, the map its
beliefs amount to: for each bay, `o` when it believes the bay open, `x` when
it believes it blocked, `?` when it believes neither, and `!` when it has no
world left and believes everything. A belief that is false is marked `*`,
which is what a stale map is. With W* the designated worlds:

  B_i blocked(t)      blocked(t) at every world i considers possible from W*
  B_i B_j blocked(t)  the same, one relation further

--about prints, for one bay, what every robot believes every other robot's
map shows there: the beliefs a robot sends on, and the ones that are
themselves stale.

  trace.py --task out/radio/radio.json --plan out/radio/plan.json
  trace.py --task out/radio/radio.json --actions stage_t1 clear_t3 --about t3
  trace.py --task out/radio/radio.json --actions clear_t3 stage_t1 tell:r4:t3:open

An action may also be `tell:AGENT:BAY:open|blocked`, which is not in the
domain: a private announcement to AGENT that the bay is open or blocked, the
others oblivious. It is what fusing a received map as fact would be, and it
is here to show what that does to a robot whose map says otherwise.

The product update is false_belief_demo's: observability is evaluated once
per state, as plank and Aletheia do, and an update that leaves a robot
without a world is kept and reported, as plank keeps it. The planner, run
with consistent beliefs required, refuses one.
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

        # plank's updater: the type whose condition holds at every designated
        # world, for the whole state.
        kind = {}
        for agent in self.agents:
            for label, cond in (obs.get(agent) or {}).items():
                if all(self.holds(w, cond.get('formula')) for w in self.designated):
                    kind[agent] = label
            if agent not in kind:
                raise ValueError(f'no observability class of {agent} holds')

        relations = {}
        for a in self.agents:
            table = rel[kind[a]]
            relations[a] = {}
            for w, e in pairs:
                relations[a][name[(w, e)]] = [
                    name[(v, f)] for v, f in pairs
                    if v in self.sees(a, w) and f in table.get(e, [])]

        chosen = [outcome] if outcome else action['designated']
        designated = [name[(w, e)] for w, e in pairs
                      if w in self.designated and e in chosen]
        return Model([name[p] for p in pairs], relations, labels, designated).generated()

    def generated(self):
        """The submodel generated by the designated worlds. Every formula
        evaluated here is evaluated from them, so the worlds no relation
        reaches from them change no answer; without this the model doubles
        at every message and a long policy cannot be replayed."""
        keep = self.reachable()
        return Model([w for w in self.worlds if w in keep],
                     {a: {w: [v for v in r.get(w, []) if v in keep] for w in keep}
                      for a, r in self.relations.items()},
                     {w: self.labels[w] for w in keep}, self.designated)

    def applicable(self, action):
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

    def belief(self, chain, atom):
        """'x' blocked, 'o' open, '?' neither, '!' no world."""
        worlds = self.frontier(chain)
        if worlds is None:
            return '!'
        values = {atom in self.labels[v] for v in worlds}
        if values == {True}:
            return 'x'
        if values == {False}:
            return 'o'
        return '?'

    def truth(self, atom):
        values = {atom in self.labels[w] for w in self.designated}
        return 'x' if values == {True} else 'o' if values == {False} else '?'

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

    def reachable(self):
        seen, todo = set(self.designated), list(self.designated)
        while todo:
            w = todo.pop()
            for a in self.agents:
                for v in self.sees(a, w):
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
        return seen


def announcement(agent, bay, value, agents):
    """A private announcement of the bay's state to @p agent, in the shape
    plank grounds private-announcement into."""
    atom = f'blocked_{bay}'
    pre = atom if value == 'blocked' else {'connective': 'not', 'formula': atom}
    return {
        'events': ['e-pos', 'nil'],
        'designated': ['e-pos'],
        'preconditions': {'e-pos': {'formula': pre}, 'nil': {'formula': 'true'}},
        'effects': {},
        'relations': {'Fully': {'e-pos': ['e-pos'], 'nil': ['nil']},
                      'Oblivious': {'e-pos': ['nil'], 'nil': ['nil']}},
        'observability-conditions': {
            a: {('Fully' if a == agent else 'Oblivious'): {'formula': 'true'}} for a in agents},
    }


def bays_of(task):
    """Every bay some atom of the task names. plank leaves `facts` empty and
    drops atoms false everywhere from the labels, so the actions are read too."""
    import re
    return sorted(set(re.findall(r'"blocked_([A-Za-z0-9-]+)"', json.dumps(task))))


def haulers_of(model):
    return {a for a in model.agents
            if all(f'hauls_{a}' in model.labels[w] for w in model.designated)}


def report(model, bays, indent, out=None):
    """The floor, every robot's map, and the counts. Returns the number of
    stale entries."""
    truth = ' '.join(model.truth(f'blocked_{t}') for t in bays)
    print(f'{indent}floor        {truth}')
    haulers = haulers_of(model)
    stale = 0
    rows = {}
    for a in model.agents:
        cells, wrong = [], []
        for t in bays:
            b = model.belief([a], f'blocked_{t}')
            mark = '*' if b in 'ox' and b != model.truth(f'blocked_{t}') else ' '
            if mark == '*':
                wrong.append(t)
            cells.append(b + mark)
        stale += len(wrong)
        role = 'hauler' if a in haulers else ''
        done = 'delivered' if all(f'delivered_{a}' in model.labels[w] for w in model.designated) else ''
        rows[a] = {'map': [c[0] for c in cells], 'stale': wrong}
        print(f'{indent}{a:5s} {role:6s} {"".join(c + " " for c in cells)}'
              f'{"stale: " + ", ".join(wrong) if wrong else ""}  {done}'.rstrip())
    lost = model.collapse()
    state = f'{lost[0]} has no world at {lost[1]}: it believes everything' if lost \
        else 'every robot has a consistent belief'
    print(f'{indent}{len(model.worlds)} worlds, {len(model.designated)} designated; '
          f'{stale} stale entries; {state}')
    if out is not None:
        out.append({'worlds': len(model.worlds), 'reachable': len(model.reachable()),
                    'stale': stale, 'maps': rows, 'consistent': lost is None})
    return stale


def about(model, bay):
    """What every robot believes every other robot's map shows at @p bay."""
    agents = model.agents
    atom = f'blocked_{bay}'
    print(f'      B_row B_col blocked({bay}); * where the row robot is wrong about the column '
          'robot\'s map')
    print('      ' + ' ' * 6 + ''.join(f'{b:>4s}' for b in agents))
    wrong = 0
    for a in agents:
        cells = []
        for b in agents:
            if a == b:
                cells.append('   .')
                continue
            held = model.belief([a, b], atom)
            actual = model.belief([b], atom)
            mark = '*' if held in 'ox' and actual in 'ox' and held != actual else ' '
            wrong += mark == '*'
            cells.append(f'  {held}{mark}')
        print(f'      {a:6s}' + ''.join(cells))
    print(f'      {wrong} second-order beliefs about {bay} are false')
    return wrong


def walk(node, task, model, bays, indent, problems, steps):
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
        record = []
        report(child, bays, indent + '    ', record)
        steps.append({'action': node['action'], 'outcome': outcome, **record[0]})
        if child.collapse():
            problems.append(node['action'] + ' leaves a robot without a consistent belief')
        walk(br['subtree'], task, child, bays, indent + '  ', problems, steps)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--plan')
    g.add_argument('--actions', nargs='+')
    p.add_argument('--about', nargs='*', default=[],
                   help='bays to print second-order beliefs for, after the last action')
    p.add_argument('--json', help='write the maps after every step here')
    args = p.parse_args()

    task = json.load(open(args.task))
    model = Model.of(task)
    bays = bays_of(task)
    problems, steps = [], []
    print(f'  initial        {" ".join(bays)}')
    record = []
    report(model, bays, '      ', record)
    steps.append({'action': None, 'outcome': None, **record[0]})

    if args.plan:
        plan = json.load(open(args.plan))
        if plan is None:
            print('the planner returned no plan')
            return 1
        walk(plan, task, model, bays, '  ', problems, steps)
    else:
        for step in args.actions:
            if step.startswith('tell:'):
                _, agent, bay, value = step.split(':')
                name, outcome = step, ''
                act = announcement(agent, bay, value, model.agents)
            else:
                name, _, outcome = step.partition(':')
                act = task['actions'][name]
            if not outcome and len(act['designated']) > 1:
                outcome = act['designated'][0]
            if not model.applicable(act):
                print(f'  {name}  NOT APPLICABLE')
                problems.append(name + ' does not apply')
                break
            model = model.update(act, outcome or None)
            print(f'  {name}' + (f'  [{outcome}]' if outcome else ''))
            record = []
            report(model, bays, '      ', record)
            steps.append({'action': name, 'outcome': outcome or None, **record[0]})
        for bay in args.about:
            about(model, bay)

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(steps, fh, indent=1)
    for msg in problems:
        print('PROBLEM: ' + msg)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
