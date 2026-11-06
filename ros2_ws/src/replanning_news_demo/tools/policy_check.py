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
Policies of replanning on news, checked against the domain's semantics without
a planner.

    policy_check.py TASK_DIR      the plans Aletheia wrote for act1 .. act3-news

The domain's actions are of three kinds, and on S5 models with every world
designated each has a short reading. A drive or a crossing is a public ontic
action: it moves a robot, and every agent sees it. A look by i at door d is
semi-private sensing: i's partition is refined by d's value, and the others'
is not, though they learn that i knows. A message by i about d is public
sensing: every partition is refined by d's value. A public announcement of a
formula keeps the worlds where it holds. The designated worlds of a branch
are those consistent with the outcomes observed on it.

A policy is a solution from a state when, on every branch, every action's
precondition holds at every designated world, the branches of a sensing
action cover its outcomes, and the goal holds at every leaf. This file checks
that, and the policy's height, for the policies Aletheia returned: each act's
policy on the model before the news, on the model the news leaves (the
policy kept), and the policy planned after the news.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lazy as Z  # noqa: E402
import model as M  # noqa: E402

DOORS = ('w', 'm', 'e')


class State:
    """Worlds, each agent's partition as a cell index per world, the designated
    worlds, and the robots' places, which every agent knows."""

    def __init__(self, act, restricted=False):
        self.act = act
        self.agents = M.ROBOTS[act]
        self.worlds = [w for w in M.worlds(act) if not restricted or M.holds(act, act, w)]
        n = len(self.worlds)
        self.cell = {}
        for a in self.agents:
            ids, cells = [], []
            for i in range(n):
                for k, rep in enumerate(cells):
                    if M.related(act, a, self.worlds[i], self.worlds[rep]):
                        ids.append(k)
                        break
                else:
                    cells.append(i)
                    ids.append(len(cells) - 1)
            self.cell[a] = ids
        self.designated = set(range(n))
        self.place = {r: M.START[r] for r in self.agents if r in M.START}

    def copy(self):
        s = State.__new__(State)
        s.act, s.agents, s.worlds = self.act, self.agents, self.worlds
        s.cell = {a: list(c) for a, c in self.cell.items()}
        s.designated = set(self.designated)
        s.place = dict(self.place)
        return s

    def value(self, i, d):
        return self.worlds[i][0][d]

    def knows(self, a, d, open_=True):
        """K_a (open-d) at every designated world, or K_a (not open-d) with open_=False."""
        for i in self.designated:
            for j in range(len(self.worlds)):
                if self.cell[a][j] == self.cell[a][i] and self.value(j, d) != open_:
                    return False
        return True

    def knows_whether(self, a, d, everywhere=False):
        for i in (range(len(self.worlds)) if everywhere else self.designated):
            for j in range(len(self.worlds)):
                if self.cell[a][j] == self.cell[a][i] and self.value(j, d) != self.value(i, d):
                    return False
        return True

    def refine(self, a, d):
        """Agent a's partition, split by d's value."""
        self.cell[a] = [c * 2 + int(self.value(i, d)) for i, c in enumerate(self.cell[a])]


def apply(state, action):
    """The branches of an action from a state: [(event, state)], or a reason it is not applicable."""
    name, agent = action.rsplit('_', 1)
    kind, _, arg = name.partition('-')
    s = state.copy()
    if kind == 'goto':
        if agent not in ('c', 's') or s.place.get(agent) in (arg, 'n', 'r'):
            return f'{action}: not applicable'
        s.place[agent] = arg
        return [(f'e-{name}', s)]
    if kind == 'cross':
        d = arg
        if agent != 'c' or s.place.get('c') != f'v{d}':
            return f'{action}: the courier is not at the mouth'
        if not s.knows('c', d):
            return f'{action}: the courier does not know the door open'
        s.place['c'] = 'n'
        return [(f'e-{name}', s)]
    if name == 'deliver':
        if s.place.get('c') != 'n':
            return f'{action}: the courier is not in the corridor'
        s.place['c'] = 'r'
        return [(f'e-{name}', s)]
    if kind in ('look', 'tell'):
        d = arg
        if kind == 'look':
            at = 'ks' if d == 'm' else f'v{d}'
            if s.place.get(agent) != at:
                return f'{action}: {agent} is not where door {d} is seen from'
        elif agent == 'c' or not s.knows_whether(agent, d):
            return f'{action}: {agent} does not know whether door {d} is open'
        out = []
        for v, ev in ((True, f'e-{kind}-{d}-open'), (False, f'e-{kind}-{d}-shut')):
            t = s.copy()
            t.designated = {i for i in s.designated if s.value(i, d) == v}
            if not t.designated:
                continue
            for a in ([agent] if kind == 'look' else t.agents):
                t.refine(a, d)
            out.append((ev, t))
        return out
    return f'{action}: unknown'


def tree(task, plan):
    """The plan file as (action, [(event, subtree)])."""
    events = {a: v['designated'] for a, v in task['actions'].items()}

    def node(n):
        if n is None:
            return None
        return (n['action'], [(events[n['action']][b['event']], node(b['subtree'])) for b in n['branches']])
    return node(plan)


TRANSPARENT = []


class _Item:
    def __init__(self, action, children, outcomes):
        self.action, self.children, self.outcomes = action, children, outcomes


class _Plan:
    def __init__(self, items):
        self.items = items


def committed_late(policy, place='st'):
    """The policy after tools/lazy.py, in this file's tree form."""
    items = []

    def visit(n):
        if n is None:
            return Z.DONE
        k = len(items)
        name, agent = n[0].rsplit('_', 1)
        items.append(_Item(f'({name.replace("-", "_", 1)} {agent})', [], []))
        items[k].outcomes = [ev for ev, _ in n[1]]
        items[k].children = [visit(c) for _, c in n[1]]
        return k
    visit(policy)
    plan, _ = Z.lazy(_Plan(items), 'c', place)

    def back(i):
        if i == Z.DONE:
            return None
        it = plan.items[i]
        name, agent = it.action.strip('()').split()
        return (f'{name.replace("_", "-", 1)}_{agent}', [(o, back(c)) for o, c in zip(it.outcomes, it.children)])
    return back(0)


def check(policy, state):
    """(solution, height, reason) of a policy from a state.

    Also records, in TRANSPARENT, whether each message the policy sends is sent
    where its modal precondition, the speaker knowing whether, holds at every
    world of the state and not only the designated ones: the condition under
    which the message's occurrence carries no information beyond its answer."""
    if policy is not None and policy[0].startswith('tell-'):
        name, agent = policy[0].rsplit('_', 1)
        TRANSPARENT.append(state.knows_whether(agent, name[-1], everywhere=True))
    if policy is None:
        return (state.place.get('c') == 'r', 0, '' if state.place.get('c') == 'r' else 'a leaf short of the goal')
    action, branches = policy
    out = apply(state, action)
    if isinstance(out, str):
        return False, 0, out
    by_event = dict(branches)
    height = 0
    for ev, s in out:
        if ev not in by_event:
            # a branch the policy does not plan for, reached from this state
            return False, 0, f'{action}: outcome {ev} has no continuation'
        ok, h, why = check(by_event[ev], s)
        if not ok:
            return False, 0, why
        height = max(height, h)
    return True, height + 1, ''


def leaves(policy, state, path=()):
    """The branches the policy takes from a state: (path of actions, outcomes, leaf state)."""
    if policy is None:
        return [(path, state)]
    action, branches = policy
    out = apply(state, action)
    by_event = dict(branches)
    res = []
    for ev, s in out:
        res += leaves(by_event[ev], s, path + ((action, ev),))
    return res


def counterexample():
    """Persistence fails when a message's occurrence carries information.

    Four worlds over p and q; the speaker a knows whether p where q holds and
    not where it fails; the listener b knows nothing; the designated worlds
    are the two where q holds. The policy is a's message about p, after which
    b must know q. From the start the message can be sent only where a knows,
    so its occurrence tells b that q holds. After the news p, a knows whether p
    in the world where q fails as well, the message no longer tells b anything
    about q, and the policy fails."""
    worlds = [(p, q) for q in (True, False) for p in (True, False)]

    def cells_a(ws):
        # a's partition: where q holds a tells p apart, where it fails it does not
        return [tuple(j for j, w in enumerate(ws) if (w[1] and v[1] and w[0] == v[0]) or (not w[1] and not v[1]))
                for v in ws]

    def after_message(ws, designated):
        ca = cells_a(ws)
        knows = [all(ws[j][0] == ws[i][0] for j in ca[i]) for i in range(len(ws))]
        survive = [i for i in range(len(ws)) if knows[i]]
        if not all(knows[i] for i in designated):
            return None
        # b's cell at each designated world: the survivors with the same p (the message's answer)
        return all(all(ws[j][1] for j in survive if ws[j][0] == ws[i][0]) for i in designated)
    start = after_message(worlds, [0, 1])
    news = [w for w in worlds if w[0]]
    after = after_message(news, [0])
    return {'K_b q after the message, from the start': start, 'after the news p': after}


def main():
    base = sys.argv[1]

    def load(name):
        task = json.load(open(os.path.join(base, name, f'{name}.json')))
        plan = json.load(open(os.path.join(base, name, 'plan.json')))
        return tree(task, plan)
    ok_all = True
    results = {}
    for act in (1, 2, 3):
        before, after = load(f'act{act}'), load(f'act{act}-news')
        rows = [('the policy before the news, on the model before it', before, State(act)),
                ('the same policy kept, on the model the news leaves', before, State(act, restricted=True)),
                ('the policy planned after the news, on that model', after, State(act, restricted=True)),
                ('the policy before the news, committed late', committed_late(before), State(act)),
                ('the policy planned after the news, committed late', committed_late(after),
                 State(act, restricted=True))]
        results[act] = {}
        print(f'act {act} ({", ".join(M.ROBOTS[act])}), news: {M.NEWS_TEXT[act]}')
        for label, pol, st in rows:
            TRANSPARENT.clear()
            ok, h, why = check(pol, st)
            transparent = all(TRANSPARENT)
            results[act][label] = {'solution': ok, 'height': h, 'worlds': len(st.worlds),
                                   'messages': len(TRANSPARENT), 'messages_transparent': transparent}
            print(f'  {"ok  " if ok else "FAIL"} {label}: {len(st.worlds)} worlds, '
                  f'{"height " + str(h) if ok else why}; {len(TRANSPARENT)} messages, '
                  f'{"each sent where the speaker knows at every world" if transparent else "SOME NOT TRANSPARENT"}')
            ok_all &= ok
        # the valuations the news leaves, and whether it changes them
        vals_before = {M.tag(v) for v, _ in State(act).worlds}
        vals_after = {M.tag(v) for v, _ in State(act, restricted=True).worlds}
        same = vals_before == vals_after
        results[act]['valuations'] = {'before': sorted(vals_before), 'after': sorted(vals_after), 'unchanged': same}
        print(f'  the news {"leaves the valuations as they were" if same else "removes valuations"}: '
              f'{len(vals_before)} -> {len(vals_after)}')
    results['counterexample'] = counterexample()
    json.dump(results, open(os.path.join(base, 'policy_check.json'), 'w'), indent=1)
    ce = results['counterexample']
    print(f'the counterexample: b knows q after the message from the start: {ce["K_b q after the message, from the start"]}; '
          f'after the news p: {ce["after the news p"]}')
    print('every policy a solution where it was checked' if ok_all else 'SOMETHING IS WRONG')
    sys.exit(0 if ok_all else 1)


if __name__ == '__main__':
    main()
