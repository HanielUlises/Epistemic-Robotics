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
Policies for the ward: every robot plans on its own beliefs.

    plan.py --task TASK.json --plan POLICY.json --act 2 --formulation explanation
            [--timeout S] [--consistent-beliefs]     (ePlanSys's AletheiaPlanSolver passes these)
    plan.py --act 2 --formulation explanation --show

A robot chooses from what it knows and how it ranks the worlds it cannot rule
out (tools/ranks.py), and from nothing else: its cell is the set of worlds
consistent with its own looks and with the reports it has heard. It chooses
what minimises the time it expects to take to the end of its mission when
each world w of its cell is weighted EPS ** rank(w): a rank read as an order
of magnitude, as in Spohn's and Goldszmidt and Pearl's reading of ordinal
conditional functions. The objective is additive over the branches of a look,
so a dynamic programme over the robot's own states finds the optimum. A robot
with no beliefs (knowledge) minimises its worst time instead, the sum of its
times breaking ties at each choice.

What a robot may do is the domain's (epddl-workspace/belief-revision):

  seek      a hop along the hall, or along the corridor on the way back, only
            if the robot does not believe every door ahead of it shut
  enter     a lobby, to the bend from which its door is seen, only if the
            robot does not believe the door shut
  look      at the door from the bend: the robot sees it, the others see
            that it looked
  back      out of a lobby to its mouth
  cross     along the jog through a door the robot knows to be open, and out
            on the far side
  walk      a hop on the far side, to the ward or home
  tell      a door found shut, on the radio: every robot hears it
  round     the porter's walk at two (act 3), once both couriers are in their wards

A team is two robots each planning for itself; a robot that finds a door shut
reports it at once. The team's policy is one tree, built in the order the
executor dispatches it: at each step every free robot names its next action,
actions of different robots are dispatched together, a look is last in its
group because it branches, and a report or the porter's round goes alone
because every robot's next choice may depend on it. The predicted durations
follow ePlanSys's parallel groups (plansys2_epistemic_executor/policy_parallel.hpp).

The tree is written in Aletheia's policy format with the grounded names plank
gives the domain's actions, so the AletheiaPlanSolver plugin can call this
script as its planner and validate the result as it validates Aletheia's.
"""

import argparse
import functools
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import ranks as R  # noqa: E402

EPS = R.EPS         # a rank read as an order of magnitude: P(w) proportional to EPS ** rank(w)
LOOK = 7            # turning to face the jog, and the laser's decision
TELL = 3
TURN = 6            # turning round in a lobby to back out
SEEK_BUDGET = 9

SOUTH = ['va', 'vb', 'st', 'vc', 'vd', 'sp']           # the hall, west to east
NORTH = ['nw', 'na', 'nb', 'nc', 'nd']                 # the corridor
DOOR_AT = {('S', 0): 'a', ('S', 1): 'b', ('S', 3): 'c', ('S', 4): 'd',
           ('N', 1): 'a', ('N', 2): 'b', ('N', 3): 'c', ('N', 4): 'd'}
MOUTH = {(s, d): i for (s, i), d in DOOR_AT.items()}
ROBOTS = {1: ['c'], 2: ['c', 'd'], 3: ['c', 'd']}
ROOM = {'c': ('S', 5, 'hc'), 'd': ('S', 2, 'ht')}       # where each starts: a room off the hall
WARD = {'c': ('N', 0, 'rc'), 'd': ('N', 2, 'rd')}        # the corridor place a ward opens off, and the bed


def chain(side):
    return SOUTH if side == 'S' else NORTH


def bend(side, d):
    return ('j' if side == 'S' else 'k') + d


def mouth(side, d):
    return ('v' if side == 'S' else 'n') + d


@functools.lru_cache(maxsize=None)
def hop_time(side, i, j):
    nodes, south, north, _ = L.roadmap()
    c = chain(side)
    return L.trip(L.route(south if side == 'S' else north, c[i], c[j]))


@functools.lru_cache(maxsize=None)
def enter_time(side, d):
    return L.trip([mouth(side, d), bend(side, d)])


@functools.lru_cache(maxsize=None)
def back_time(side, d):
    return L.trip([bend(side, d), mouth(side, d)]) + TURN


@functools.lru_cache(maxsize=None)
def cross_time(side, d):
    far = 'N' if side == 'S' else 'S'
    return L.trip([bend(side, d), bend(far, d), mouth(far, d)])


@functools.lru_cache(maxsize=None)
def bed_time(robot):
    nodes, south, north, _ = L.roadmap()
    side, i, bed = WARD[robot]
    return L.trip(L.route(north, NORTH[i], bed))


@functools.lru_cache(maxsize=None)
def room_time(robot):
    nodes, south, north, _ = L.roadmap()
    side, i, room = ROOM[robot]
    return L.trip(L.route(south, room, SOUTH[i]))


def ahead(side, i, j):
    """The doors at j and beyond, travelling from i to j along a chain."""
    step = 1 if j > i else -1
    out = []
    k = j
    while 0 <= k < len(chain(side)):
        if (side, k) in DOOR_AT:
            out.append(DOOR_AT[(side, k)])
        k += step
    return out


def walk_time(side, i, j):
    t, k = 0, i
    while k != j:
        n = k + (1 if j > k else -1)
        t += hop_time(side, k, n)
        k = n
    return t


# -- one robot's plan ------------------------------------------------------------------------
#
# A robot's state: its leg ('out' to its ward, 'back' home in act 3), the side it
# searches on (the start of the leg), where it is (side, index on the chain, and
# whether it stands at the bend of the lobby there), whether the porter has
# walked at two, and its cell.

class Planner:
    """One robot's planner. With weights, a map world -> probability, it minimises the
    expected time under them instead, and believes what it knows: the bound
    tools/sensitivity.py compares the formulations with, not a formulation."""

    def __init__(self, act, formulation, robot, weights=None):
        self.act, self.f, self.robot = act, formulation, robot
        self.weights = weights
        if weights is not None:
            self.f = 'knowledge'

    def value(self, times):
        """The objective of a map world -> time."""
        if self.weights is not None:
            # a night of no weight still counts a little, or a robot whose cell holds
            # only such nights is indifferent to everything, and wanders
            return sum((self.weights[w] + 1e-6) * t for w, t in times.items())
        if self.f == 'knowledge':
            return (max(times.values()), sum(times.values()))
        return sum(EPS ** R.rank(self.f, w) * t for w, t in times.items())

    def options(self, st):
        """[(action, duration, [(sub-cell, next state)])] open to the robot."""
        leg, side, pos, after, cell = st
        out = []
        s, i, at_bend = pos
        far = 'N' if side == 'S' else 'S'
        door = DOOR_AT.get((side, i))
        if at_bend:
            known_open = R.knows(cell, lambda w: R.open_at(w, door, after))
            known_shut = R.knows(cell, lambda w: not R.open_at(w, door, after))
            if not known_open and not known_shut:
                o = frozenset(w for w in cell if R.open_at(w, door, after))
                out.append((('look', side, door), LOOK,
                            [(o, (leg, side, pos, after, o)), (cell - o, (leg, side, pos, after, cell - o))]))
            if known_open:
                out.append((('cross', side, door), cross_time(side, door),
                            [(cell, (leg, side, (far, MOUTH[(far, door)], False), after, cell))]))
            out.append((('back', side, door), back_time(side, door),
                        [(cell, (leg, side, (side, i, False), after, cell))]))
            return out
        for j in (i - 1, i + 1):
            if not 0 <= j < len(chain(side)):
                continue
            doors = ahead(side, i, j)
            if not doors or not R.entertains(self.f, cell, lambda w: any(R.open_at(w, d, after) for d in doors)):
                continue
            out.append((('seek', side, i, j), hop_time(side, i, j),
                        [(cell, (leg, side, (side, j, False), after, cell))]))
        if door and R.entertains(self.f, cell, lambda w: R.open_at(w, door, after)):
            out.append((('enter', side, door), enter_time(side, door),
                        [(cell, (leg, side, (side, i, True), after, cell))]))
        return out

    def finish(self, st):
        """From the far side of the band to the end of the leg: deterministic walks."""
        leg, side, pos, after, cell = st
        if leg == 'out':
            _, i, _ = WARD[self.robot]
            return walk_time('N', pos[1], i) + bed_time(self.robot)
        _, i, _ = ROOM[self.robot]
        return walk_time('S', pos[1], i) + room_time(self.robot)

    @functools.lru_cache(maxsize=None)
    def solve(self, st, budget=SEEK_BUDGET):
        """(objective, times, first action) of the best plan from st, over st's cell."""
        leg, side, pos, after, cell = st
        if pos[0] != side:                       # across: walk on, and in act 3 come back
            t = self.finish(st)
            if leg == 'out' and self.act == 3:
                _, i, _ = WARD[self.robot]
                r = self.solve(('back', 'N', ('N', i, False), True, cell), SEEK_BUDGET)
                if r is None:
                    return None
                times = {w: t + bed_time(self.robot) + x for w, x in r[1].items()}
                return self.value(times), times, None
            times = {w: t for w in cell}
            return self.value(times), times, None
        best = None
        for action, dur, outs in self.options(st):
            b = budget - 1 if action[0] in ('seek', 'back') else budget
            if b < 0:
                continue
            times = {}
            ok = True
            for sub, nxt in outs:
                r = self.solve(nxt, b)
                if r is None:
                    ok = False
                    break
                for w, x in r[1].items():
                    times[w] = dur + x
            if not ok:
                continue
            v = self.value(times)
            if best is None or v < best[0]:
                best = (v, times, action)
        return best

    def choose(self, leg, side, pos, after, cell):
        r = self.solve((leg, side, pos, after, frozenset(cell)))
        return None if r is None else r[2]


# -- the team --------------------------------------------------------------------------------

def _repository_tools():
    """The repository's tools/, for its product update (plan_trace.py) and model checker."""
    here = os.path.dirname(os.path.abspath(__file__))
    for up in ('../../../../tools', '../../../tools', '../../../../../../tools'):
        p = os.path.realpath(os.path.join(here, up))
        if os.path.exists(os.path.join(p, 'plan_trace.py')):
            return p
    p = os.path.expanduser('~/Projects/Epistemic-Robotics-revision/tools')
    return p if os.path.exists(os.path.join(p, 'plan_trace.py')) else None


sys.path.insert(0, _repository_tools() or '.')
from model_check import Model, model_of  # noqa: E402
from plan_trace import update  # noqa: E402

_TASKS = {}


def load_task(act, formulation):
    """The grounded task of an act and formulation, exported by plank once and kept."""
    key = (act, formulation)
    if key in _TASKS:
        return _TASKS[key]
    here = os.path.dirname(os.path.abspath(__file__))
    for epddl in (os.path.join(here, '..', 'epddl'), os.path.join(here, '../../../../epddl-workspace/belief-revision')):
        if os.path.exists(os.path.join(epddl, 'belief-revision.epddl')):
            break
    plank = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
    lib = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
    out = os.path.join(tempfile.gettempdir(), f'belief-revision-tasks-{os.getuid()}', f'act{act}-{formulation}')
    path = os.path.join(out, f'act{act}-{formulation}.json')
    src = os.path.join(epddl, 'instances', f'act{act}-{formulation}.epddl')
    if not os.path.exists(path) or os.path.getmtime(path) < os.path.getmtime(src):
        subprocess.run([plank, 'export', '-d', os.path.join(epddl, 'belief-revision.epddl'), '-p', src,
                        '-l', lib, '-o', out], check=True, capture_output=True)
    _TASKS[key] = json.load(open(path))
    return _TASKS[key]


def reachable(model):
    """The part of a model the designated worlds can reach: the rest is never consulted."""
    seen, todo = set(model.designated), list(model.designated)
    agents = list(model.relations)
    while todo:
        w = todo.pop()
        for a in agents:
            for v in model.image(a, w):
                if v not in seen:
                    seen.add(v)
                    todo.append(v)
    keep = [w for w in model.worlds if w in seen]
    return Model(keep, {a: {w: [v for v in model.relations[a][w] if v in seen] for w in keep}
                        for a in agents}, {w: model.labels[w] for w in keep},
                 [w for w in model.designated if w in seen])


class Team:
    """The couriers of an act, each planning for itself on what it knows. What a
    courier knows is read off the epistemic state, the grounded task's model
    advanced by the product update of every action the team takes: its own
    looks, the reports it hears, and what the others' actions reveal, since every
    courier sees every drive and crossing happen and so learns its precondition."""

    def __init__(self, act, formulation, weights=None, task=None):
        self.act, self.f = act, formulation
        self.robots = ROBOTS[act]
        self.worlds = R.worlds(act)
        self.planners = {r: Planner(act, formulation, r, weights) for r in self.robots}
        self.task = task or load_task(act, 'knowledge' if weights is not None else formulation)
        self.agents = self.task['language']['agents']

    def tree(self):
        """The policy: a list of steps, a step being (robot, action) or
        ('branch', robot, look, {True: subtree, False: subtree})."""
        st = {r: {'leg': 'out', 'side': 'S', 'pos': ('room',), 'done': False} for r in self.robots}
        m = model_of(self.task['initial-state'])
        origin = {w: int(w[1:]) for w in m.worlds}
        return self._grow((m, origin), st, False, 0)

    def cell(self, robot, state):
        m, origin = state
        w = next(iter(m.designated))
        return frozenset(self.worlds[origin[v]] for v in m.image(robot, w))

    def designated(self, state):
        m, origin = state
        return frozenset(self.worlds[origin[w]] for w in m.designated)

    def apply(self, state, robot, action, event=None):
        m, origin = state
        act = self.task['actions'][grounded(robot, action)]
        events = [event] if event else act['designated']
        m2, pairs = update(m, act, self.agents, events)
        m2 = reachable(m2)
        return m2, {w: origin[pairs[w][0]] for w in m2.worlds}

    def _next(self, r, s, state, after):
        if s['done']:
            return None
        leg, side, pos = s['leg'], s['side'], s['pos']
        if pos == ('room',):
            return ('out', r)
        if pos[0] != side:                         # across: walk on
            if leg == 'out':
                _, i, _ = WARD[r]
                if pos[1] != i:
                    return ('walk', 'N', pos[1], pos[1] + (1 if i > pos[1] else -1))
                return ('bed', r)
            _, i, _ = ROOM[r]
            if pos[1] != i:
                return ('walk', 'S', pos[1], pos[1] + (1 if i > pos[1] else -1))
            return ('home', r)
        return self.planners[r].choose(leg, side, pos, after, self.cell(r, state))

    @staticmethod
    def _move(s, a):
        k = a[0]
        if k == 'out':
            s['pos'] = ('S', ROOM[a[1]][1], False)
        elif k in ('seek', 'walk'):
            s['pos'] = (a[1], a[3], False)
        elif k == 'enter':
            s['pos'] = (s['pos'][0], s['pos'][1], True)
        elif k == 'back':
            s['pos'] = (s['pos'][0], s['pos'][1], False)
        elif k == 'cross':
            far = 'N' if a[1] == 'S' else 'S'
            s['pos'] = (far, MOUTH[(far, a[2])], False)
        elif k in ('bed', 'home'):
            s['done'] = True

    def _grow(self, state, st, after, depth, pending=()):
        if depth > 140:
            raise RuntimeError('the team does not finish')
        if all(s['done'] for s in st.values()):
            if self.act == 3 and not after:
                # both in their wards: the porter walks at two, then they come back
                st2 = {r: {'leg': 'back', 'side': 'N', 'pos': ('N', WARD[r][1], False), 'done': False}
                       for r in st}
                state = self.apply(state, 'p', ('round',))
                steps = [('p', ('round',))]
                for r in self.robots:
                    state = self.apply(state, r, ('leave', r))
                    steps.append((r, ('leave', r)))
                return steps + self._grow(state, st2, True, depth + 1)
            return []
        # a report first, of a door found shut that a teammate still on its way does not know is shut
        for who, d in pending:
            others = [o for o in self.robots if o != who and not st[o]['done']]
            if any(not R.knows(self.cell(o, state), lambda w, d=d: not R.open_at(w, d, after)) for o in others):
                rest = tuple(x for x in pending if x != (who, d))
                return [(who, ('tell', d))] + self._grow(self.apply(state, who, ('tell', d)), st, after, depth + 1,
                                                         rest)
        acts = {}
        for r in self.robots:
            a = self._next(r, st[r], state, after)
            if a is not None:
                acts[r] = a
        # one robot in a lobby at a time: a robot about to enter one that another
        # robot stands in, or is entering or crossing now, waits a step
        for r in list(acts):
            a = acts[r]
            if a[0] != 'enter':
                continue
            busy = any(st[o]['pos'][-1] is True and DOOR_AT.get(st[o]['pos'][:2]) == a[2] for o in st if o != r)
            busy = busy or any(o != r and x[0] in ('enter', 'cross') and x[2] == a[2] for o, x in acts.items()
                               if o < r or x[0] == 'cross')
            if busy:
                del acts[r]
        if not acts:
            raise RuntimeError(f'nobody can act: {st}')
        looks = [r for r, a in acts.items() if a[0] == 'look']
        for r in looks[1:]:                        # one look per step, the first robot's
            del acts[r]
        steps = []
        st2 = {r: dict(s) for r, s in st.items()}
        look = None
        for r, a in acts.items():
            if a[0] == 'look':
                look = (r, a)
                continue
            steps.append((r, a))
            state = self.apply(state, r, a)
            self._move(st2[r], a)
        if look:
            r, a = look
            d = a[2]
            name = grounded(r, a)
            branches = {}
            for value in (True, False):
                event = next(e for e in self.task['actions'][name]['events']
                             if e.endswith('-open' if value else '-shut'))
                child = self.apply(state, r, a, event)
                if child[0].designated:
                    shut = () if value or len(self.robots) == 1 else ((r, d),)
                    branches[value] = self._grow(child, st2, after, depth + 1, pending + shut)
            return steps + [('branch', r, a, branches)]
        return steps + self._grow(state, st2, after, depth + 1, pending)


# -- durations and paths ---------------------------------------------------------------------

def duration(robot, action):
    k = action[0]
    if k in ('seek', 'walk'):
        return hop_time(action[1], action[2], action[3])
    if k == 'enter':
        return enter_time(action[1], action[2])
    if k == 'look':
        return LOOK
    if k == 'back':
        return back_time(action[1], action[2])
    if k == 'cross':
        return cross_time(action[1], action[2])
    if k == 'tell':
        return TELL
    if k in ('bed', 'leave'):
        return bed_time(robot)
    if k in ('out', 'home'):
        return room_time(robot)
    if k == 'round':
        return 0          # the porter's round lasts what his walk lasts, measured in the run
    raise ValueError(action)


def paths(tree, world, after=False):
    """The steps a world takes through the tree, as a flat list of (robot, action)."""
    out = []
    for step in tree:
        if step[0] == 'branch':
            _, r, action, branches = step
            out.append((r, action))
            return out + paths(branches[R.open_at(world, action[2], after)], world, after)
        out.append(step)
        if step[1][0] == 'round':
            after = True
    return out


ALONE = ('tell', 'round')
IN_LOBBY = ('enter', 'look', 'back', 'cross')


def lobby_of(a):
    return a[2] if a[0] in IN_LOBBY else None


def groups(steps):
    """ePlanSys's parallel groups over a path: maximal runs of steps by different
    robots, a report or the porter's round alone, a look last in its run, and no
    two robots in one lobby (the vocabulary holds each lobby as a resource)."""
    out, cur = [], []
    for r, a in steps:
        same_lobby = lobby_of(a) is not None and any(lobby_of(x[1]) == lobby_of(a) for x in cur)
        if cur and (a[0] in ALONE or same_lobby or any(x[0] == r for x in cur)
                    or any(x[1][0] in ALONE for x in cur)):
            out.append(cur)
            cur = []
        cur.append((r, a))
        if a[0] == 'look' or a[0] in ALONE:
            out.append(cur)
            cur = []
    if cur:
        out.append(cur)
    return out


def timeline(tree, world, round_time=0.0):
    """The mission's time and each robot's finishing time, for one world."""
    t = 0.0
    done = {}
    for g in groups(paths(tree, world)):
        t += max((round_time if a[0] == 'round' else duration(r, a)) for r, a in g)
        for r, a in g:
            if a[0] in ('bed', 'home'):
                done[r] = t
    return t, done


# -- Aletheia's policy format ----------------------------------------------------------------

def grounded(robot, action):
    """The grounded name plank gives the domain's action (tools/model.py writes the domain)."""
    k = action[0]
    if k in ('seek', 'walk'):
        side, i, j = action[1:]
        c = chain(side)
        return f'{k}-{c[i]}-{c[j]}_{robot}'
    if k in ('enter', 'look', 'back'):
        return f"{k}-{action[2]}-{'s' if action[1] == 'S' else 'n'}_{robot}"
    if k == 'cross':
        return f"cross-{action[2]}-{'n' if action[1] == 'S' else 's'}_{robot}"
    if k == 'tell':
        return f'tell-{action[1]}_{robot}'
    if k == 'bed':
        return f'deliver_{robot}'
    if k == 'leave':
        return f'leave_{robot}'
    if k == 'home':
        return f'arrive_{robot}'
    if k == 'out':
        return f'set-out_{robot}'
    if k == 'round':
        return 'round_p'
    raise ValueError(action)


def to_policy(tree, events):
    """The tree as Aletheia's nested {"action", "branches": [{"event", "subtree"}]}.
    events maps a grounded look to its event names, in the task's order."""
    node = None
    for step in reversed(tree):
        if step[0] == 'branch':
            _, r, action, branches = step
            name = grounded(r, action)
            names = events.get(name, [f'e-{name}-open', f'e-{name}-shut'])
            br = []
            for value, sub in branches.items():
                ev = next(i for i, e in enumerate(names) if e.endswith('-open' if value else '-shut'))
                br.append({'event': ev, 'subtree': to_policy(sub, events)})
            node = {'action': name, 'branches': br}
        else:
            r, action = step
            node = {'action': grounded(r, action), 'branches': [{'event': 0, 'subtree': node}]}
    return node


def show(tree, indent=0):
    pad = '  ' * indent
    for step in tree:
        if step[0] == 'branch':
            _, r, action, branches = step
            print(f'{pad}{grounded(r, action)}')
            for v, sub in branches.items():
                print(f"{pad}  {'open' if v else 'shut'}:")
                show(sub, indent + 2)
        else:
            print(f'{pad}{grounded(*step)}')


def looks(tree, world):
    """The looks a world's path makes: robot, door, and whether the door was open."""
    out, after = [], False
    for r, a in paths(tree, world):
        if a[0] == 'round':
            after = True
        elif a[0] == 'look':
            out.append((r, a[2], R.open_at(world, a[2], after), after))
    return out


def report(act, formulation, tree):
    rows = []
    for w in R.worlds(act):
        t, done = timeline(tree, w)
        seen = [d + ('' if o else '!') + ("'" if aft else '') for r, d, o, aft in looks(tree, w)]
        rows.append((w, t, done, seen))
    return rows


def expected(act, formulation, tree, key='makespan'):
    rows = report(act, formulation, tree)
    if key == 'makespan':
        return sum(R.frequency(w) * t for w, t, _, _ in rows)
    return sum(R.frequency(w) * sum(done.values()) for w, t, done, _ in rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task')
    ap.add_argument('--plan')
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True, choices=R.FORMULATIONS)
    ap.add_argument('--timeout', type=float)
    ap.add_argument('--consistent-beliefs', action='store_true')
    ap.add_argument('--show', action='store_true')
    a, _ = ap.parse_known_args()
    task = json.load(open(a.task)) if a.task else None
    tree = Team(a.act, a.formulation, task=task).tree()
    if a.show or not a.plan:
        show(tree)
        for w, t, done, looks in report(a.act, a.formulation, tree):
            print(f'  {R.name(w):4s} {t:6.1f} s  {done}  looked at {looks}')
        print(f'  expected {expected(a.act, a.formulation, tree):.1f} s')
    if a.plan:
        events = {n: act['events'] for n, act in task['actions'].items()} if task else {}
        json.dump(to_policy(tree, events), open(a.plan, 'w'))
        print(f'[plan] {a.formulation} act {a.act}: policy written to {a.plan}')
        print('Solution found')


if __name__ == '__main__':
    main()
