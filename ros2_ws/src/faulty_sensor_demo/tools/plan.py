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
The fleet's policy: where the scouts read, and when a bed-mover drives in.

    plan.py --task TASK.json --plan POLICY.json --act 2 --formulation sensors
            [--timeout S] [--consistent-beliefs]     (ePlanSys's AletheiaPlanSolver passes these)
    plan.py --act 2 --formulation sensors --show [--night AB-s]

The state the fleet plans on is public: every reading is said on the radio as
soon as it is taken, so what the fleet knows is a set of worlds of its model
of error (tools/ranks.py), and what it believes is the least-ranked of them.

What a robot may do. A scout hops along the hall on its own line, looks down
a passage from its mouth (a reading that may be wrong) and says what it read,
or drives in to the mark before the door, reads it there (never wrong), says
so and drives out. A bed-mover hops along the hall; it drives into a passage
only when it is sure enough that the door is open, as its formulation has it
(belief, firm belief of degree 2, or knowledge); at the mark it reads the
door, and crosses, or says the door is shut and backs out at a crawl.

The search. The fleet decides when something has been said: every scout is
given its next reading (a door, and whether from the mouth or the mark, or
nothing) and every bed-mover the mouth it waits at. The robots then act in
the groups ePlanSys dispatches, each group lasting as long as its longest
action, until the next reading; a bed-mover that becomes sure of a door goes
for the one that takes it soonest to its ward. The policy minimises the
expected time to the last delivery, a world of rank k weighing EPS ** k, by
dynamic programming over what is known and where the robots stand: it is the
best policy of this form, for the model and the degree of belief of the
formulation.

The policy is one tree, written in Aletheia's policy format with the grounded
names plank gives the domain's actions (tools/model.py), so the
AletheiaPlanSolver plugin can call this script as its planner and validate
what it returns.
"""

import argparse
import functools
import hashlib
import itertools
import json
import os
import pickle
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import ranks as R  # noqa: E402

LOOK = 7            # settling at a mouth, and the laser's decision
CHECK = 4           # the same at the mark
SAY = 3             # a reading said on the radio
TURN_IN = 3         # turning from the hall's line into a passage
TURN_ROUND = 6      # a scout turning round at the mark

HALL = ['va', 'vb', 'sl', 'st', 'vc', 'vd', 'sp']          # the hall's places, west to east
DOOR_IDX = {'a': 0, 'b': 1, 'c': 4, 'd': 5}
ROOM_AT = {'hd': 0, 'ld': 2, 'ht': 3, 'hp': 6}
INF = float('inf')
SEARCH = 1          # the search's version: raise it when the search changes, and kept policies are planned anew
MOVERS = {r.name for act in R.FLEETS for r in R.FLEETS[act] if r.role == 'mover'}
STAGE = 2           # how many mouths the search considers for a waiting bed-mover
IDLE = 0.02         # a bed is not wheeled about for nothing: what a second of a bed-mover's hop weighs
DETOUR = 60         # what a shut door costs a bed-mover waiting at its mouth, roughly


# -- durations ---------------------------------------------------------------------------

@functools.lru_cache(maxsize=None)
def hop_time(r, i, j):
    south = L.roadmap(r)[1]
    return L.trip(L.route(south, HALL[i], HALL[j], r), robot=r)


@functools.lru_cache(maxsize=None)
def out_time(r, room):
    south = L.roadmap(r)[1]
    return L.trip(L.route(south, room, HALL[ROOM_AT[room]], r), robot=r)


@functools.lru_cache(maxsize=None)
def in_path(r, d):
    """From where a robot stands at a mouth to the mark: a scout straight, a bed-mover by
    the mouth's middle."""
    return ['v' + d, 'e' + d, 'm' + d] if r in MOVERS else ['v' + d, 'm' + d]


@functools.lru_cache(maxsize=None)
def in_time(r, d):
    return L.trip(in_path(r, d), robot=r) + TURN_IN


@functools.lru_cache(maxsize=None)
def retire_time(r, d):
    return L.trip(['m' + d, 'v' + d], robot=r) + TURN_ROUND


@functools.lru_cache(maxsize=None)
def reverse_time(r, d):
    return int(round(L.length(['m' + d, 'e' + d], r) / L.CRAWL))


@functools.lru_cache(maxsize=None)
def cross_time(d):
    return L.trip(['m' + d, 'n' + d])


@functools.lru_cache(maxsize=None)
def deliver_time(d, ward):
    north = L.roadmap()[2]
    return L.trip(L.route(north, 'n' + d, f'bed{ward}')) + TURN_IN


def duration(r, a, fleet):
    k = a[0]
    if k == 'hop':
        return hop_time(r, a[1], a[2])
    if k == 'out':
        return out_time(r, fleet[r].room)
    if k in ('advance', 'enter'):
        return in_time(r, a[1])
    if k == 'look':
        return LOOK
    if k == 'check':
        return CHECK
    if k == 'say':
        return SAY
    if k == 'retire':
        return retire_time(r, a[1])
    if k == 'reverse':
        return reverse_time(r, a[1])
    if k == 'cross':
        return cross_time(a[1])
    if k == 'deliver':
        return deliver_time(a[1], fleet[r].ward)
    raise ValueError(a)


# -- the planner -------------------------------------------------------------------------

class Planner:

    def __init__(self, act, formulation):
        self.act, self.formulation = act, formulation
        self.robots = R.fleet(act)
        self.fleet = {r.name: r for r in self.robots}
        self.names = [r.name for r in self.robots]
        self.scouts = [r.name for r in self.robots if r.role == 'scout']
        self.movers = [r.name for r in self.robots if r.role == 'mover']
        self.M = R.model(act, formulation)
        self.firm = R.FIRM[formulation]
        self.eps = R.PLAN_EPS[formulation]
        self.ranks = sorted(set(self.M.rank))
        self.by_rank = {k: self.M.mask(lambda w, k=k: self.M.rank[self.M.worlds.index(w)] == k) for k in self.ranks}
        self.memo = {}

    # -- what is known and believed -------------------------------------------------------

    @functools.lru_cache(maxsize=None)
    def weight(self, W):
        return sum(self.eps ** k * bin(W & m).count('1') for k, m in self.by_rank.items())

    @functools.lru_cache(maxsize=None)
    def least(self, W):
        for k in self.ranks:
            if W & self.by_rank[k]:
                return k
        raise ValueError('no world')

    def known_open(self, W, d):
        return not W & ~self.M.open[d]

    def known_shut(self, W, d):
        return not W & self.M.open[d]

    def decided(self, W, d):
        return self.known_open(W, d) or self.known_shut(W, d)

    @functools.lru_cache(maxsize=None)
    def firmness(self, W, d):
        """How firmly the door is believed open: the rank of the most plausible world in
        which it is shut, over the least rank; INF when it is known open."""
        shut = W & ~self.M.open[d]
        if not shut:
            return INF
        return self.least(shut) - self.least(W)

    def sure(self, W, d):
        f = self.firmness(W, d)
        return f == INF if self.firm is None else f >= self.firm

    def p_open(self, W, d):
        return self.weight(W & self.M.open[d]) / self.weight(W)

    @functools.lru_cache(maxsize=None)
    def shut_reading(self, r, d, k):
        return self.M.shut_reading(r, d, k)

    def look_index(self, looks, d):
        return looks[R.DOORS.index(d)] + 1 if self.M.kind == 'readings' else 1

    def look_splits(self, W, looks, r, d):
        if self.formulation == 'contact':
            return False
        k = self.look_index(looks, d)
        if k > R.LOOKS:
            return False
        m = self.shut_reading(r, d, k)
        return bool(W & m) and bool(W & ~m)

    # -- the start --------------------------------------------------------------------------

    def start(self):
        return (self.M.all, tuple(('R',) for _ in self.names), (0, 0, 0, 0))

    def done(self, pos):
        return all(pos[self.names.index(m)][0] == 'D' for m in self.movers)

    def through(self, r, i, d):
        """A bed-mover's time from hall place i through door d to its ward."""
        j = DOOR_IDX[d]
        t = sum(hop_time(r, a, a + (1 if j > i else -1)) for a in range(i, j, 1 if j > i else -1))
        return t + in_time(r, d) + cross_time(d) + deliver_time(d, self.fleet[r].ward)

    # -- one robot's next action -------------------------------------------------------------

    def next_action(self, r, W, pos, looks, tasks, busy):
        p = pos[self.names.index(r)]
        rb = self.fleet[r]
        if p[0] == 'R':
            return ('out',)
        if p[0] == 'D':
            return None
        if p[0] == 'N':
            return ('deliver', p[1])
        if p[0] == 'M':
            d = p[1]
            if rb.role == 'scout':
                return ('retire', d) if self.decided(W, d) else ('check', d)
            if self.known_open(W, d):
                return ('cross', d)
            if self.known_shut(W, d):
                return ('reverse', d)
            return ('check', d)
        i = p[1]
        if rb.role == 'mover':
            sure = [d for d in R.DOORS if self.sure(W, d)]
            if sure:
                d = min(sure, key=lambda x: self.through(r, i, x))
                if DOOR_IDX[d] == i:
                    return None if d in busy else ('enter', d)
                return ('hop', i, i + (1 if DOOR_IDX[d] > i else -1))
            d = tasks.get(r)
            if d is None or DOOR_IDX[d] == i:
                return None
            return ('hop', i, i + (1 if DOOR_IDX[d] > i else -1))
        task = tasks.get(r)
        if task is None:
            return None
        kind, d = task
        if DOOR_IDX[d] != i:
            return ('hop', i, i + (1 if DOOR_IDX[d] > i else -1))
        if kind == 'look':
            return ('look', d, self.look_index(looks, d))
        return None if d in busy else ('advance', d)

    @staticmethod
    def moved(p, a, room_at=None):
        k = a[0]
        if k == 'out':
            return ('S', room_at)
        if k == 'hop':
            return ('S', a[2])
        if k in ('advance', 'enter'):
            return ('M', a[1])
        if k in ('retire', 'reverse'):
            return ('S', DOOR_IDX[a[1]])
        if k == 'cross':
            return ('N', a[1])
        if k == 'deliver':
            return ('D',)
        return p

    # -- the groups up to the next reading ---------------------------------------------------

    def run(self, W, pos, looks, tasks):
        """The groups dispatched from here until a reading is taken, or until every bed is
        delivered: (time, groups, reading, pos). A reading is (robot, action), taken last
        in its group. None when nobody can act."""
        t, groups = 0, []
        for _ in range(60):
            if self.done(pos):
                return t, groups, None, pos
            busy = {p[1] for p in pos if p[0] == 'M'}
            group, sense, new = [], None, list(pos)
            for k, r in enumerate(self.names):
                a = self.next_action(r, W, pos, looks, tasks, busy)
                if a is None:
                    continue
                if a[0] in ('look', 'check'):
                    if sense is None:
                        sense = (r, a)
                    continue
                if a[0] in ('advance', 'enter'):
                    busy = busy | {a[1]}
                group.append((r, a))
                new[k] = self.moved(pos[k], a, ROOM_AT.get(self.fleet[r].room))
            if not group and sense is None:
                return None
            if sense is not None:
                group.append(sense)
            t += max(duration(r, a, self.fleet) for r, a in group)
            t += IDLE * sum(duration(r, a, self.fleet) for r, a in group if a[0] == 'hop' and r in self.movers)
            groups.append(group)
            pos = tuple(new)
            if sense is not None:
                return t, groups, sense, pos
        return None

    def outcomes(self, W, pos, looks, sense):
        """What a reading may be: [(outcome, what is said, W, pos, looks)]."""
        r, a = sense
        d = a[1]
        if a[0] == 'look':
            m = self.shut_reading(r, d, a[2])
            lk = list(looks)
            if self.M.kind == 'readings':
                lk[R.DOORS.index(d)] += 1
            out = [('shut', ('say', d, 'read-shut', a[2]), W & m, pos, tuple(lk)),
                   ('open', ('say', d, 'open', a[2]), W & ~m, pos, tuple(lk))]
        else:
            out = [('shut', ('say', d, 'shut'), W & ~self.M.open[d], pos, looks),
                   ('open', ('say', d, 'sure'), W & self.M.open[d], pos, looks)]
        return [o for o in out if o[2]]

    # -- the decisions -------------------------------------------------------------------------

    def bound(self, W, pos):
        """No policy delivers sooner: every bed-mover by its quickest door not known shut."""
        worst = 0.0
        for r in self.movers:
            p = pos[self.names.index(r)]
            ward = self.fleet[r].ward
            if p[0] == 'D':
                continue
            if p[0] == 'N':
                t = deliver_time(p[1], ward)
            else:
                i = ROOM_AT[self.fleet[r].room] if p[0] == 'R' else DOOR_IDX[p[1]] if p[0] == 'M' else p[1]
                t = min((self.through(r, i, d) for d in R.DOORS if not self.known_shut(W, d)), default=INF)
                if p[0] == 'R':
                    t += out_time(r, self.fleet[r].room)
                if p[0] == 'M' and self.known_open(W, p[1]):
                    t = cross_time(p[1]) + deliver_time(p[1], ward)
            worst = max(worst, t)
        return worst

    def staging(self, W, pos, r):
        """The mouths a bed-mover may be sent to wait at: the STAGE doors that would take it
        soonest to its ward, a door likelier shut counting as that much further."""
        p = pos[self.names.index(r)]
        if p[0] not in ('R', 'S') or any(self.sure(W, d) for d in R.DOORS):
            return [None]
        i = ROOM_AT[self.fleet[r].room] if p[0] == 'R' else p[1]
        doors = [d for d in R.DOORS if not self.known_shut(W, d)]
        doors.sort(key=lambda d: self.through(r, i, d) + (1 - self.p_open(W, d)) * DETOUR)
        return doors[:STAGE]

    def options(self, W, pos, looks):
        per = []
        for r in self.names:
            if self.fleet[r].role == 'scout':
                o = [None]
                for d in R.DOORS:
                    if self.look_splits(W, looks, r, d):
                        o.append(('look', d))
                    if not self.decided(W, d):
                        o.append(('probe', d))
            else:
                o = self.staging(W, pos, r)
            per.append(o)
        for combo in itertools.product(*per):
            yield {r: t for r, t in zip(self.names, combo) if t is not None}

    def solve(self, state):
        """The least expected time to the last delivery from a state, and the tasks that
        give it."""
        if state in self.memo:
            return self.memo[state]
        W, pos, looks = state
        if self.done(pos):
            self.memo[state] = (0.0, None)
            return self.memo[state]
        cands, seen = [], set()
        total = self.weight(W)
        for tasks in self.options(W, pos, looks):
            got = self.run(W, pos, looks, tasks)
            if got is None:
                continue
            t, groups, sense, pos2 = got
            key = (repr(groups), pos2)
            if key in seen:
                continue
            seen.add(key)
            outs = self.outcomes(W, pos2, looks, sense) if sense is not None else []
            lb = t + sum(self.weight(o[2]) / total * (SAY + self.bound(o[2], o[3])) for o in outs)
            cands.append((lb, t, tasks, outs))
        cands.sort(key=lambda c: c[0])
        best = (INF, None)
        for lb, t, tasks, outs in cands:
            if lb >= best[0] - 1e-9:
                break
            v, rest = t, lb - t
            for _, _, Wo, po, lk in outs:
                share = self.weight(Wo) / total
                rest -= share * (SAY + self.bound(Wo, po))
                v += share * (SAY + self.solve((Wo, po, lk))[0])
                if v + rest >= best[0] - 1e-9:
                    v = INF
                    break
            if v < best[0] - 1e-9:
                best = (v, tasks)
        self.memo[state] = best
        return best

    def tree(self, state=None):
        """The policy from a state: a list of groups, the last one, if it ends in a reading,
        followed by its branches."""
        state = state or self.start()
        W, pos, looks = state
        if self.done(pos):
            return []
        v, tasks = self.solve(state)
        if tasks is None and v == INF:
            raise RuntimeError(f'no policy from {self.describe(state)}')
        t, groups, sense, pos2 = self.run(W, pos, looks, tasks)
        out = [('group', g) for g in groups]
        if sense is None:
            return out
        branches = {}
        for name, said, Wo, po, lk in self.outcomes(W, pos2, looks, sense):
            branches[name] = [('group', [(sense[0], said)])] + self.tree((Wo, po, lk))
        out[-1] = ('branch', groups[-1], branches)
        return out

    def describe(self, state):
        W, pos, looks = state
        ws = [self.M.name(i) for i in range(self.M.n) if W >> i & 1]
        return f'{dict(zip(self.names, pos))} knowing {ws if len(ws) < 13 else len(ws)}'


# -- a policy on a night -----------------------------------------------------------------

def outcome_on(night, r, a):
    """What a reading reads on a night."""
    if a[0] == 'look':
        return 'shut' if R.reads_shut(night, r, a[1]) else 'open'
    return 'shut' if a[1] in night[0] else 'open'


def trace(tree, night):
    """The groups a night takes through the policy; None if the policy has no branch for
    a reading the night gives (the model held it impossible)."""
    out = []
    for node in tree:
        if node[0] == 'group':
            out.append(node[1])
            continue
        _, group, branches = node
        out.append(group)
        o = outcome_on(night, *group[-1])
        if o not in branches:
            return None
        rest = trace(branches[o], night)
        return None if rest is None else out + rest
    return out


def timeline(groups, fleet):
    """(the time of the last delivery, each bed-mover's delivery time, the passages a
    bed-mover drove into and found shut)."""
    t, done, wrong = 0.0, {}, []
    for g in groups:
        t += max(duration(r, a, fleet) for r, a in g)
        for r, a in g:
            if a[0] == 'deliver':
                done[r] = t
            if a[0] == 'reverse':
                wrong.append((r, a[1]))
    return (max(done.values()) if done else t), done, wrong


def said(groups):
    return [(r, a) for g in groups for r, a in g if a[0] == 'say']


# -- Aletheia's policy format ------------------------------------------------------------

def grounded(robot, action, formulation):
    """The grounded name plank gives the domain's action (tools/model.py writes the domain)."""
    k = action[0]
    readings = R.MODEL[formulation] == 'readings'
    if k == 'out':
        return f'set-out_{robot}'
    if k == 'hop':
        return f'hop-{HALL[action[1]]}-{HALL[action[2]]}_{robot}'
    if k == 'look':
        return f'look{action[2]}-{action[1]}_{robot}' if readings else f'look-{action[1]}_{robot}'
    if k == 'say':
        if action[2] in ('open', 'read-shut'):          # what a look from the mouth read
            return f'say-{action[2]}{action[3] if readings else ""}-{action[1]}_{robot}'
        return f'say-{action[2]}-{action[1]}_{robot}'
    if k == 'enter':
        return f'enter-{GUARD[formulation]}-{action[1]}_{robot}'
    if k in ('advance', 'check', 'retire', 'reverse', 'cross', 'deliver'):
        return f'{k}-{action[1]}_{robot}'
    raise ValueError(action)


GUARD = {'credulous': 'believing', 'readings': 'firmly', 'sensors': 'firmly', 'contact': 'knowing'}


def flat(tree):
    """The policy as steps in order: (robot, action), or ('branch', robot, action, {outcome: steps})."""
    out = []
    for node in tree:
        if node[0] == 'group':
            out += node[1]
        else:
            _, group, branches = node
            out += group[:-1]
            out.append(('branch', group[-1][0], group[-1][1], {o: flat(sub) for o, sub in branches.items()}))
    return out


def to_policy(steps, events, formulation):
    node = None
    for step in reversed(steps):
        if step[0] == 'branch':
            _, r, action, branches = step
            name = grounded(r, action, formulation)
            names = events.get(name, [f'e-{name.split("_")[0]}-open', f'e-{name.split("_")[0]}-shut'])
            br = []
            for o, sub in branches.items():
                ev = next(i for i, e in enumerate(names) if e.split('_')[0].endswith('-' + o))
                br.append({'event': ev, 'subtree': to_policy(sub, events, formulation)})
            node = {'action': name, 'branches': br}
        else:
            r, action = step
            node = {'action': grounded(r, action, formulation), 'branches': [{'event': 0, 'subtree': node}]}
    return node


def words(robot, action):
    k = action[0]
    if k == 'say':
        d = action[1].upper()
        return {'shut': f'{robot}: {d} is shut, read at the mark', 'sure': f'{robot}: {d} is open, read at the mark',
                'open': f'{robot}: I read {d} open from the mouth',
                'read-shut': f'{robot}: I read {d} shut from the mouth'}[action[2]]
    if k == 'hop':
        return f'{robot} {HALL[action[1]]}>{HALL[action[2]]}'
    return f'{robot} {k}' + (f' {action[1].upper()}' if len(action) > 1 else '')


def show(tree, indent=0):
    pad = '  ' * indent
    for node in tree:
        if node[0] == 'group':
            print(pad + ' | '.join(words(r, a) for r, a in node[1]))
        else:
            _, group, branches = node
            print(pad + ' | '.join(words(r, a) for r, a in group) + ' ?')
            for o, sub in branches.items():
                print(f'{pad}  {o}:')
                show(sub, indent + 2)


def report(pl, tree):
    """Per night: (night, frequency, time, deliveries, passages found shut, what was said)."""
    rows = []
    for night in R.nights(pl.act):
        groups = trace(tree, night)
        if groups is None:
            rows.append((night, R.frequency(pl.act, night), None, {}, [], []))
            continue
        t, done, wrong = timeline(groups, pl.fleet)
        rows.append((night, R.frequency(pl.act, night), t, done, wrong, said(groups)))
    return rows


def summary(rows):
    """Expected time, the probability that a bed-mover drives into a shut passage, and
    the same on the nights with no short-sighted laser and on those with exactly one."""
    def over(sel):
        z = sum(f for n, f, t, *_ in rows if sel(n))
        if not z:
            return None, None
        return (sum(f * t for n, f, t, *_ in rows if sel(n)) / z,
                sum(f for n, f, t, d, wrong, s in rows if sel(n) and wrong) / z)
    return {'all': over(lambda n: True), 'sound': over(lambda n: not n[1]),
            'one': over(lambda n: len(n[1]) == 1), 'more': over(lambda n: len(n[1]) > 1)}


_PLANS = {}


def _cache_path(act, formulation):
    """Where a policy is kept: the search takes minutes in the larger acts. A policy is a
    function of the search (SEARCH, to be raised when it changes), the act's fleet,
    the formulation and every duration."""
    fleet = {r.name: r for r in R.fleet(act)}
    times = []
    for r in fleet.values():
        times += [out_time(r.name, r.room)] + [hop_time(r.name, i, i + 1) for i in range(len(HALL) - 1)]
        times += [(in_time(r.name, d), retire_time(r.name, d), reverse_time(r.name, d), cross_time(d),
                   deliver_time(d, r.ward) if r.ward else 0) for d in R.DOORS]
    key = repr((SEARCH, act, formulation, [(r.name, r.role, r.room, r.ward, r.grade) for r in fleet.values()],
                R.MODEL[formulation], R.FIRM[formulation], R.PLAN_EPS[formulation], R.EPS, R.LOOKS,
                LOOK, CHECK, SAY, STAGE, DETOUR, IDLE, times))
    d = os.path.join(tempfile.gettempdir(), f'faulty-sensor-policies-{os.getuid()}')
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, f'act{act}-{formulation}-{hashlib.sha1(key.encode()).hexdigest()[:12]}.pickle')


def plan(act, formulation):
    key = (act, formulation)
    if key not in _PLANS:
        sys.setrecursionlimit(10000)
        pl = Planner(act, formulation)
        path = _cache_path(act, formulation)
        if os.path.exists(path):
            tree, pl.value, pl.states = pickle.load(open(path, 'rb'))
        else:
            tree = pl.tree()
            pl.value, pl.states = pl.solve(pl.start())[0], len(pl.memo)
            pickle.dump((tree, pl.value, pl.states), open(path + '.tmp', 'wb'))
            os.replace(path + '.tmp', path)
        _PLANS[key] = (pl, tree)
    return _PLANS[key]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task')
    ap.add_argument('--plan')
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True, choices=R.FORMULATIONS)
    ap.add_argument('--timeout', type=float)
    ap.add_argument('--consistent-beliefs', action='store_true')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--night')
    a, _ = ap.parse_known_args()
    sys.setrecursionlimit(10000)
    pl, tree = plan(a.act, a.formulation)
    if a.show or not a.plan:
        if a.night:
            night = R.night_of(a.act, a.night)
            t = 0
            for g in trace(tree, night):
                t += max(duration(r, x, pl.fleet) for r, x in g)
                print(f'{t:5.0f}  ' + ' | '.join(words(r, x) for r, x in g))
        else:
            show(tree)
        rows = report(pl, tree)
        for night, f, t, done, wrong, sd in rows:
            if a.night and R.tag(night) != a.night:
                continue
            print(f'  {R.tag(night):7s} {f:.4f} {t:6.0f} s  {done}  ' +
                  (f'DROVE INTO {[d.upper() for _, d in wrong]} SHUT  ' if wrong else '') +
                  '; '.join(words(r, x) for r, x in sd))
        s = summary(rows)
        print('  ' + '  '.join(f'{k}: {v[0]:.1f} s, wrong {v[1]:.3f}' for k, v in s.items() if v[0] is not None))
        print(f'  {pl.states} states; the fleet expects {pl.value:.1f} s')
    if a.plan:
        task = json.load(open(a.task)) if a.task else None
        events = {n: act['events'] for n, act in task['actions'].items()} if task else {}
        json.dump(to_policy(flat(tree), events, a.formulation), open(a.plan, 'w'))
        print(f'[plan] {a.formulation} act {a.act}: policy written to {a.plan}')
        print('Solution found')


if __name__ == '__main__':
    main()
