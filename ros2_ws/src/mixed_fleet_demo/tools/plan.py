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
Policies for a mixed fleet: every robot routes itself on its own beliefs, and
the hospital's courier also decides which of its beliefs to tell.

    plan.py --task TASK.json --plan POLICY.json --act 2 --formulation perspective
            [--timeout S] [--consistent-beliefs]     (ePlanSys's AletheiaPlanSolver passes these)
    plan.py --act 2 --formulation perspective --show

Routing. A robot chooses its next move from what it knows and how it ranks the
worlds it cannot rule out (tools/ranks.py), and from nothing else: it
minimises the time it expects to take to its ward with each world of its cell
weighted EPS ** rank, its own looks branching the plan, and never drives to a
door it believes shut. Every robot routes so, the courier included. A robot
that finds a door shut reports it at once.

Belief reports. At each of its steps the courier c may instead say that it
believes a door shut, or open, to a robot that takes its word. It may say so
only when it believes it, does not know it, and believes the listener does not
believe it (tools/model.py writes the guard). The formulations differ in what
c says:

  egocentric    c takes the others to rank as it does. By its own lights a
                report of evidence then carries its belief, so no belief of
                its own is ever news to anyone, and it says none.
  relevance     c says every belief the guard allows, as soon as it allows it,
                each once to each listener.
  perspective   c says a belief when saying it lowers the fleet's expected time
                to the last delivery, each world of c's cell weighted
                EPS ** (c's rank), the others predicted on the beliefs they do
                hold: their own rankings and trust, which every robot knows.
                The prediction runs every robot's routing, one step of
                lookahead with every robot then routing itself (a rollout of
                the policy in which nobody reports a belief).

The team's policy is one tree, built in the order ePlanSys dispatches it, and
written in Aletheia's policy format with the grounded names plank gives the
domain's actions (tools/model.py), so the AletheiaPlanSolver plugin can call
this script as its planner and validate what it returns.
"""

import argparse
import functools
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import ranks as R  # noqa: E402
from kripke import State  # noqa: E402

EPS = R.EPS
LOOK = 7            # turning to face the jog, and the laser's decision
TELL = 3            # a report on the radio
TURN = 6            # turning round in a lobby to back out
SEEK_BUDGET = 9
THINKER = 'c'

SOUTH = ['va', 'vb', 'st', 'vc', 'vd', 'sp']           # the hall, west to east
DOOR_AT = {0: 'a', 1: 'b', 3: 'c', 4: 'd'}
MOUTH = {d: i for i, d in DOOR_AT.items()}
ROOM_AT = {'hd': 0, 'ht': 2, 'hp': 5}
WORLDS = R.worlds()


def is_open(d):
    return lambda w: R.open_at(WORLDS[w], d)


def is_shut(d):
    return lambda w: not R.open_at(WORLDS[w], d)


# -- durations ---------------------------------------------------------------------------

@functools.lru_cache(maxsize=None)
def hop_time(i, j):
    nodes, south, north, _ = L.roadmap()
    return L.trip(L.route(south, SOUTH[i], SOUTH[j]))


@functools.lru_cache(maxsize=None)
def enter_time(d):
    return L.trip(['v' + d, 'j' + d])


@functools.lru_cache(maxsize=None)
def back_time(d):
    return L.trip(['j' + d, 'v' + d]) + TURN


@functools.lru_cache(maxsize=None)
def cross_time(d):
    return L.trip(['j' + d, 'k' + d, 'n' + d])


@functools.lru_cache(maxsize=None)
def deliver_time(d, ward):
    nodes, south, north, _ = L.roadmap()
    return L.trip(L.route(north, 'n' + d, f'bed{ward}'))


@functools.lru_cache(maxsize=None)
def room_time(room):
    nodes, south, north, _ = L.roadmap()
    return L.trip(L.route(south, room, SOUTH[ROOM_AT[room]]))


def ahead(i, j):
    """The doors at j and beyond, travelling from i to j along the hall."""
    step = 1 if j > i else -1
    out, k = [], j
    while 0 <= k < len(SOUTH):
        if k in DOOR_AT:
            out.append(DOOR_AT[k])
        k += step
    return out


def duration(robot, action, fleet):
    k = action[0]
    if k == 'seek':
        return hop_time(action[1], action[2])
    if k == 'enter':
        return enter_time(action[1])
    if k == 'look':
        return LOOK
    if k == 'back':
        return back_time(action[1])
    if k == 'cross':
        return cross_time(action[1])
    if k in ('tell', 'tellb'):
        return TELL
    if k == 'deliver':
        return deliver_time(action[1], fleet[robot].ward)
    if k == 'out':
        return room_time(fleet[robot].room)
    raise ValueError(action)


# -- one robot routing itself ------------------------------------------------------------

class Solo:
    """One robot's routing, on its own cell and its own ranks: the expected time to its
    ward with each world weighted EPS ** rank, over plans that branch on its own looks
    and never drive to a door it believes shut. Ranks are fixed within the plan: a
    robot does not anticipate what it will be told."""

    def __init__(self, robot):
        self.robot = robot

    def finish(self, d):
        return cross_time(d) + deliver_time(d, self.robot.ward)

    @functools.lru_cache(maxsize=None)
    def solve(self, i, bend, cell, ranks, budget=SEEK_BUDGET):
        rk = dict(ranks)
        m = min(rk[w] for w in cell)
        least = [w for w in cell if rk[w] == m]

        def value(times):
            return sum(EPS ** rk[w] * t for w, t in times.items())

        def entertains(f):
            return any(f(w) for w in least)

        opts = []
        door = DOOR_AT.get(i)
        if bend:
            o = frozenset(w for w in cell if R.open_at(WORLDS[w], door))
            if o and o != cell:
                opts.append((('look', door), LOOK, [o, cell - o]))
            if o == cell:
                opts.append((('cross', door), None, None))
            opts.append((('back', door), back_time(door), [cell]))
        else:
            for j in (i - 1, i + 1):
                if 0 <= j < len(SOUTH):
                    doors = ahead(i, j)
                    if doors and entertains(lambda w, ds=doors: any(R.open_at(WORLDS[w], x) for x in ds)):
                        opts.append((('seek', i, j), hop_time(i, j), [cell]))
            if door and entertains(is_open(door)):
                opts.append((('enter', door), enter_time(door), [cell]))
        best = None
        for action, dur, subs in opts:
            if action[0] == 'cross':
                times = {w: self.finish(door) for w in cell}
            else:
                b = budget - 1 if action[0] in ('seek', 'back') else budget
                if b < 0:
                    continue
                nxt = {'look': (i, True), 'back': (i, False), 'enter': (i, True)}.get(action[0])
                if action[0] == 'seek':
                    nxt = (action[2], False)
                times, ok = {}, True
                for sub in subs:
                    r = self.solve(nxt[0], nxt[1], sub, tuple(sorted((w, rk[w]) for w in sub)), b)
                    if r is None:
                        ok = False
                        break
                    for w, t in r[1].items():
                        times[w] = dur + t
                if not ok:
                    continue
            v = value(times)
            if best is None or v < best[0] - 1e-9:
                best = (v, times, action)
        return best

    def choose(self, i, bend, cell, ranks):
        m = min(ranks[w] for w in cell)
        r = self.solve(i, bend, frozenset(cell), tuple(sorted((w, ranks[w] - m) for w in cell)))
        return None if r is None else r[2]


# -- the domain's guards -----------------------------------------------------------------
#
# What the EPDDL domain requires of each action (tools/model.py writes the same
# formulas). The executor checks them before dispatching; an action every robot
# sees happen tells them its guard held, which is how the guards enter the
# epistemic state: the update keeps the worlds where the guard holds.

def searching(phys, t):
    return phys[t]['pos'][0] in ('room', 'S')


def guard_seek(M, phys, r, i, j):
    doors = ahead(i, j)

    def all_shut(v):
        return all(not R.open_at(WORLDS[v], x) for x in doors)

    return lambda w: M.E(r, w, lambda v: not all_shut(v))


def guard_enter(M, phys, r, x):
    return lambda w: M.E(r, w, is_open(x))


def guard_cross(M, r, x):
    return lambda w: R.open_at(WORLDS[w], x) and M.K(r, w, is_open(x))


def guard_tell(M, r, x):
    return lambda w: (not R.open_at(WORLDS[w], x)) and M.K(r, w, is_shut(x))


def prop(x, value):
    return is_open(x) if value else is_shut(x)


def guard_tellb(M, s, l, x, value):
    """Sincere, news to the speaker's knowledge, and relevant: s believes phi, does not
    know it, and believes l does not believe it."""
    phi = prop(x, value)
    return lambda w: (M.B(s, w, phi) and not M.K(s, w, phi)
                      and M.B(s, w, lambda v: not M.B(l, v, phi)))


# -- the fleet ---------------------------------------------------------------------------

class Team:

    def __init__(self, act, formulation):
        self.act, self.formulation = act, formulation
        self.robots = R.fleet(act, formulation)
        self.fleet = {r.name: r for r in self.robots}
        self.names = [r.name for r in self.robots]
        self.trust = R.trust(act)
        self.solo = {r.name: Solo(r) for r in self.robots}
        self.mode = formulation
        self.decisions = []          # (state key, candidates and values) for the record

    # -- the state of the robots: where each is ------------------------------------------

    def start(self):
        M = State.initial(self.robots)
        phys = {r: {'pos': ('room',)} for r in self.names}
        return M, phys

    @staticmethod
    def moved(phys, r, a):
        p = {t: dict(s) for t, s in phys.items()}
        s = p[r]
        k = a[0]
        if k == 'out':
            s['pos'] = ('S', None, False)
        elif k == 'seek':
            s['pos'] = ('S', a[2], False)
        elif k == 'enter':
            s['pos'] = ('S', s['pos'][1], True)
        elif k == 'back':
            s['pos'] = ('S', s['pos'][1], False)
        elif k == 'cross':
            s['pos'] = ('N', a[1])
        elif k == 'deliver':
            s['pos'] = ('done',)
        elif k == 'tellb':
            s['told'] = s.get('told', frozenset()) | {a[1:]}
        return p

    def place(self, phys, r):
        pos = phys[r]['pos']
        if pos[0] == 'S' and pos[1] is None:
            return ('S', ROOM_AT[self.fleet[r].room], False)
        return pos

    def apply(self, M, phys, r, a):
        """The update of the epistemic state by robot r's action a, and the robots' new
        places. A look keeps every world and splits the looker's cells; the policy's
        branch then selects the designated worlds of each outcome."""
        k = a[0]
        if k == 'out':
            phys = self.moved(phys, r, a)
            phys[r]['pos'] = ('S', ROOM_AT[self.fleet[r].room], False)
            return M, phys
        if k == 'seek':
            M = M.restrict(guard_seek(M, phys, r, a[1], a[2]))
        elif k == 'enter':
            M = M.restrict(guard_enter(M, phys, r, a[1]))
        elif k == 'cross':
            M = M.restrict(guard_cross(M, r, a[1]))
        elif k == 'tell':
            M = M.restrict(guard_tell(M, r, a[1]))
        elif k == 'tellb':
            _, l, x, v = a
            M = M.restrict(guard_tellb(M, r, l, x, v)).shift(l, prop(x, v), R.ALPHA)
        elif k == 'look':
            M = M.refine(r, is_open(a[1]))
        return M, self.moved(phys, r, a)

    # -- what each robot does next --------------------------------------------------------

    def solo_action(self, M, phys, r):
        pos = phys[r]['pos']
        w = next(iter(M.designated))
        return self.solo[r].choose(pos[1], pos[2], M.cell(r, w), M.ranks[r])

    def base_action(self, M, phys, r):
        pos = phys[r]['pos']
        if pos[0] == 'room':
            return ('out', r)
        if pos[0] == 'N':
            return ('deliver', pos[1])
        if pos[0] == 'done':
            return None
        return self.solo_action(M, phys, r)

    def candidates(self, M, phys):
        """The courier's options at this step: its own route, or a belief report to a robot
        that takes its word."""
        c = THINKER
        a0 = self.base_action(M, phys, c)
        out = [('route', a0)]
        pos = phys[c]['pos']
        if pos[0] != 'S':
            return out
        w = next(iter(M.designated))
        view = M.projected(c) if self.mode == 'egocentric' else M
        for l in self.names:
            if (l, c) not in self.trust or not searching(phys, l):
                continue
            for x in R.DOORS:
                for v in (False, True):
                    if self.mode == 'relevance' and (l, x, v) in phys[c].get('told', ()):
                        continue                 # said once
                    if M.everywhere(guard_tellb(M, c, l, x, v)) and view.everywhere(guard_tellb(view, c, l, x, v)):
                        out.append(('tellb', ('tellb', l, x, v)))
        return out

    def value(self, M, phys, pending, choice):
        """The courier's expected time to the last delivery if it takes this choice now and
        every robot then routes itself, each world of its cell weighted EPS ** its rank,
        the others predicted on the beliefs they hold (or, egocentric, on the beliefs
        they would hold ranking as c does)."""
        c = THINKER
        view = M.projected(c) if self.mode == 'egocentric' else M
        w0 = next(iter(M.designated))
        total = 0.0
        for w in M.cell(c, w0):
            Mw = view.copy()
            Mw.designated = frozenset([w])
            path = self.rollout(Mw, phys, pending, w, choice)
            total += EPS ** M.ranks[c][w] * makespan(path, self.fleet)
        return total

    def courier_action(self, M, phys, pending):
        cands = self.candidates(M, phys)
        if self.mode == 'relevance':
            tells = [ch for ch in cands if ch[0] == 'tellb']
            return (tells[0][1] if tells else cands[0][1]), None
        if len(cands) == 1:
            return cands[0][1], None
        scored = [(self.value(M, phys, pending, ch), k, ch) for k, ch in enumerate(cands)]
        scored.sort(key=lambda x: (round(x[0], 6), x[1]))
        best = scored[0][2]
        rec = [(ch, round(v, 2)) for v, _, ch in scored]
        return best[1], rec

    # -- one group of the policy ------------------------------------------------------------

    def group(self, M, phys, pending, choice=None, think=True):
        """The actions dispatched together at this point, applied: (steps, look, pending, M,
        phys), look being (robot, action) or None and still to apply. The robots choose in
        turn, each in the state the earlier ones' actions left, since the policy is a
        sequence and every action's guard must hold where it stands in it; a look goes
        last, a report alone."""
        for who, d in pending:
            others = [o for o in self.names if o != who and searching(phys, o)]
            w = next(iter(M.designated))
            if any(not M.K(o, w, is_shut(d)) for o in others):
                a = ('tell', d)
                M, phys = self.apply(M, phys, who, a)
                return [(who, a)], None, [x for x in pending if x != (who, d)], M, phys
        pending = []
        steps, look = [], None
        for r in self.names:
            if r == THINKER and self.mode != 'alone' and phys[r]['pos'][0] == 'S':
                if choice is not None:
                    a = choice[1]
                    choice = None
                elif think:
                    a, rec = self.courier_action(M, phys, pending)
                    if rec:
                        self.decisions.append((self.place(phys, r), sorted(M.designated), rec))
                else:
                    a = self.base_action(M, phys, r)
            else:
                a = self.base_action(M, phys, r)
            if a is None:
                continue
            if a[0] == 'tellb':
                if steps or look:
                    break                      # alone: next group
                M, phys = self.apply(M, phys, r, a)
                return [(r, a)], None, pending, M, phys
            if a[0] == 'look':
                if look is None:
                    look = (r, a)
                continue
            if a[0] == 'enter':
                busy = any(phys[o]['pos'][0] == 'S' and phys[o]['pos'][2] and DOOR_AT.get(phys[o]['pos'][1]) == a[1]
                           for o in self.names if o != r)
                busy = busy or any(x[0] in ('enter', 'cross') and x[1] == a[1] for _, x in steps)
                if busy:
                    continue
            M, phys = self.apply(M, phys, r, a)
            steps.append((r, a))
        if not steps and not look:
            raise RuntimeError(f'nobody can act: {phys}')
        return steps, look, pending, M, phys

    def rollout(self, M, phys, pending, world, choice):
        """The flat path from here in one world: the courier takes `choice` now and then
        routes itself, as every robot does."""
        phys = {t: dict(s) for t, s in phys.items()}
        path = []
        first = True
        for _ in range(200):
            if all(s['pos'][0] == 'done' for s in phys.values()):
                return path
            steps, look, pending, M, phys = self.group(M, phys, list(pending), choice if first else None,
                                                       think=False)
            first = False
            path += steps
            if look:
                r, a = look
                M, phys = self.apply(M, phys, r, a)
                path.append((r, a))
                if not R.open_at(WORLDS[world], a[1]) and len(self.names) > 1:
                    pending = pending + [(r, a[1])]
        raise RuntimeError('a rollout does not finish')

    def tree(self):
        M, phys = self.start()
        return self._grow(M, phys, [], 0)

    def _grow(self, M, phys, pending, depth):
        if depth > 160:
            raise RuntimeError('the fleet does not finish')
        if all(s['pos'][0] == 'done' for s in phys.values()):
            return []
        phys = {t: dict(s) for t, s in phys.items()}
        steps, look, pending, M, phys = self.group(M, phys, list(pending))
        out = list(steps)
        if look:
            r, a = look
            d = a[1]
            M2, phys2 = self.apply(M, phys, r, a)
            branches = {}
            for value in (True, False):
                # the look keeps every world; the branch selects the designated ones
                child = M2.copy()
                child.designated = frozenset(w for w in M2.designated if R.open_at(WORLDS[w], d) == value)
                if child.designated:
                    shut = [] if value or len(self.names) == 1 else [(r, d)]
                    branches[value] = self._grow(child, phys2, pending + shut, depth + 1)
            return out + [('branch', r, a, branches)]
        return out + self._grow(M, phys, pending, depth + 1)


# -- paths and times ---------------------------------------------------------------------

def paths(tree, world):
    """The steps a world takes through the tree, as a flat list of (robot, action)."""
    out = []
    for step in tree:
        if step[0] == 'branch':
            _, r, action, branches = step
            out.append((r, action))
            return out + paths(branches[R.open_at(WORLDS[world], action[1])], world)
        out.append(step)
    return out


ALONE = ('tell', 'tellb')
IN_LOBBY = ('enter', 'look', 'back', 'cross')


def lobby_of(a):
    return a[1] if a[0] in IN_LOBBY else None


def groups(steps):
    """ePlanSys's parallel groups over a path: maximal runs of steps by different
    robots, a report alone, a look last in its run, and no two robots in one lobby
    (the vocabulary holds each lobby as a resource)."""
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


def timeline(steps, fleet):
    t, done = 0.0, {}
    for g in groups(steps):
        t += max(duration(r, a, fleet) for r, a in g)
        for r, a in g:
            if a[0] == 'deliver':
                done[r] = t
    return t, done


def makespan(steps, fleet):
    return timeline(steps, fleet)[0]


# -- Aletheia's policy format ------------------------------------------------------------

def grounded(robot, action):
    """The grounded name plank gives the domain's action (tools/model.py writes the domain)."""
    k = action[0]
    if k == 'seek':
        return f'seek-{SOUTH[action[1]]}-{SOUTH[action[2]]}_{robot}'
    if k in ('enter', 'look', 'back'):
        return f'{k}-{action[1]}-s_{robot}'
    if k == 'cross':
        return f'cross-{action[1]}-n_{robot}'
    if k == 'deliver':
        return f'deliver-{action[1]}_{robot}'
    if k == 'out':
        return f'set-out_{robot}'
    if k == 'tell':
        return f'tell-{action[1]}_{robot}'
    if k == 'tellb':
        _, l, x, v = action
        return f"believe-{x}-{'open' if v else 'shut'}_{robot}_{l}"
    raise ValueError(action)


def to_policy(tree, events):
    node = None
    for step in reversed(tree):
        if step[0] == 'branch':
            _, r, action, branches = step
            name = grounded(r, action)
            names = events.get(name, [f'e-look-{action[1]}-s-open', f'e-look-{action[1]}-s-shut'])
            br = []
            for value, sub in branches.items():
                ev = next(i for i, e in enumerate(names) if e.split('_')[0].endswith('-open' if value else '-shut'))
                br.append({'event': ev, 'subtree': to_policy(sub, events)})
            node = {'action': name, 'branches': br}
        else:
            r, action = step
            node = {'action': grounded(r, action), 'branches': [{'event': 0, 'subtree': node}]}
    return node


def say(robot, action):
    k = action[0]
    if k == 'tellb':
        _, l, x, v = action
        return f"{robot} to {l}: I believe {x.upper()} {'open' if v else 'shut'}"
    if k == 'tell':
        return f'{robot}: {action[1].upper()} is shut'
    return grounded(robot, action)


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
            print(f'{pad}{say(*step)}')


def looks(tree, world):
    return [(r, a[1], R.open_at(WORLDS[world], a[1])) for r, a in paths(tree, world) if a[0] == 'look']


def report(team, tree):
    rows = []
    for k, w in enumerate(WORLDS):
        steps = paths(tree, k)
        t, done = timeline(steps, team.fleet)
        said = [say(r, a) for r, a in steps if a[0] in ('tell', 'tellb')]
        rows.append((w, t, done, looks(tree, k), said))
    return rows


def expected(team, tree):
    return sum(R.frequency(w) * t for w, t, _, _, _ in report(team, tree))


def plan(act, formulation):
    team = Team(act, formulation)
    return team, team.tree()


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
    team, tree = plan(a.act, a.formulation)
    if a.show or not a.plan:
        show(tree)
        for w, t, done, lk, said in report(team, tree):
            seen = ' '.join(f"{r}:{d.upper()}{'' if o else '!'}" for r, d, o in lk)
            print(f'  {R.name(w)} {t:6.1f} s  {done}  looked {seen}  said {said}')
        print(f'  expected {expected(team, tree):.1f} s')
    if a.plan:
        task = json.load(open(a.task)) if a.task else None
        events = {n: act['events'] for n, act in task['actions'].items()} if task else {}
        json.dump(to_policy(tree, events), open(a.plan, 'w'))
        print(f'[plan] {a.formulation} act {a.act}: policy written to {a.plan}')
        print('Solution found')


if __name__ == '__main__':
    main()
