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
The task allocation of Section 5.4: task bundling (Algorithm 1), makespan
minimisation (Algorithm 2) and the gossip auction (Algorithm 3).

A robot allocates with what it knows. Every robot has a state for the
allocation, Start(pos, t): where it can set out from and when. A robot in
contact sets out from where it is, now. A robot out of contact is bid for by
the allocating robot, from its belief about it, and sets out from its believed
place at the time the allocating robot could reach it there: the arrival time
t_i,tau of eq. (21) counts the time to tell it.

Times are travel times at the top speed over the allocating robot's map, along
paths through the cells it does not know to be occupied.
"""

import collections
import math

from . import nav
from .env import OCC

Start = collections.namedtuple('Start', 'pos t')


class Travel:
    """Travel times over one map, with the distance fields from task cells
    kept."""

    def __init__(self, area, known, speed):
        self.area, self.speed = area, speed
        self.blocked = known == OCC
        self.fields = {}

    def field(self, q):
        c = self.area.cell_of(q)
        if c not in self.fields:
            self.fields[c] = nav.distance_field(self.blocked, c)
        return self.fields[c]

    def time(self, p, q):
        """From p to the task at q, along the map."""
        f = self.field(q)
        d = f[self.area.cell_of(p)] * self.area.spec.cell
        if not math.isfinite(d):
            d = math.dist(p, q) * 1.5
        return d / self.speed

    def straight(self, p, q):
        return math.dist(p, q) / self.speed


def intercept(travel, p, t0, believed, horizon=180.0, reach=0.0):
    """The earliest time at which a robot leaving p at t0 can come within
    `reach` of a point that moves as believed(t), and where that point is
    then. Straight-line travel."""
    v = travel.speed
    t = t0
    step = 0.5
    while t <= t0 + horizon:
        q = believed(t)
        if math.dist(p, q) <= v * (t - t0) + reach:
            return t, q
        t += step
    q = believed(t0 + horizon)
    return t0 + horizon + math.dist(p, q) / v, q


def bundle_cost(travel, start, tasks, bundle):
    """Time to travel the bundle in order from the start: S_j of eq. (22)."""
    t, p = start.t, start.pos
    for tid in bundle:
        q = tasks[tid]['pos']
        t += travel.time(p, q)
        p = q
    return t


def bundling(travel, starts, tasks, order, robots):
    """Algorithm 1. starts: {robot: Start}; tasks: {tid: {'pos', 'k'}};
    order: the tasks to allocate, in the order they were found; robots: the
    robots that may be assigned. Each task goes to the k bidders of highest
    bid, h = lambda - S(B + tau), that is the least marginal cost."""
    bundles = {j: [] for j in robots}
    for tid in order:
        k = tasks[tid]['k']
        bids = []
        for j in robots:
            before = bundle_cost(travel, starts[j], tasks, bundles[j])
            after = bundle_cost(travel, starts[j], tasks, bundles[j] + [tid])
            bids.append((after - before + 1e-6 * len(bundles[j]), j))
        bids.sort()
        for _, j in bids[:k]:
            bundles[j].append(tid)
    return bundles


def schedule(travel, starts, tasks, bundles, work):
    """When every task starts, given the bundles: a task starts when the last
    of the robots assigned to it arrives, eq. (21). Returns ({tid: start},
    {robot: finish}) or None when the orders wait on each other in a cycle."""
    nxt = {j: 0 for j in bundles}
    pos = {j: starts[j].pos for j in bundles}
    free = {j: starts[j].t for j in bundles}
    start = {}
    assigned = collections.defaultdict(list)
    for j, b in bundles.items():
        for tid in b:
            assigned[tid].append(j)
    left = sum(len(b) for b in bundles.values())
    while left:
        progress = False
        for tid, who in assigned.items():
            if tid in start:
                continue
            if all(nxt[j] < len(bundles[j]) and bundles[j][nxt[j]] == tid for j in who):
                q = tasks[tid]['pos']
                arrive = max(free[j] + travel.time(pos[j], q) for j in who)
                start[tid] = arrive
                for j in who:
                    nxt[j] += 1
                    pos[j] = q
                    free[j] = arrive + work
                left -= len(who)
                progress = True
        if not progress:
            return None
    return start, free


def makespan(travel, starts, tasks, bundles, work):
    s = schedule(travel, starts, tasks, bundles, work)
    if s is None:
        return math.inf
    return max(s[1].values()) if s[1] else 0.0


def minimise(travel, starts, tasks, bundles, work, fixed=()):
    """Algorithm 2: move each task of a bundle to every other place in it and
    keep the order of least makespan. Tasks in `fixed` keep their place: their
    order has been told to robots now out of contact."""
    best = makespan(travel, starts, tasks, bundles, work)
    for j in bundles:
        for tid in list(bundles[j]):
            if tid in fixed:
                continue
            rest = [x for x in bundles[j] if x != tid]
            for n in range(len(rest) + 1):
                trial = dict(bundles)
                trial[j] = rest[:n] + [tid] + rest[n:]
                m = makespan(travel, starts, tasks, trial, work)
                if m < best - 1e-9:
                    best, bundles = m, trial
    return bundles


def gossip(travel, knowing, need, believed, now, radio):
    """Algorithm 3. knowing: {robot: Start}, the robots that know (in contact
    with the allocating robot), where and when each is free to go; need: the
    robots that must be told; believed(g, t): where g is believed to be at t.
    Every robot of `knowing` bids for every robot not yet told, by the time it
    would reach it; each takes its best, and a robot told joins the bidders
    from where it was met. Returns {robot: [robots to tell, in order]} and
    {robot: (time, place) it is expected to be told}."""
    avail = dict(knowing)
    told = set(knowing)
    bundles = {v: [] for v in knowing}
    when = {}
    pending = [g for g in need if g not in told]
    while pending:
        bids = {}
        for g in pending:
            for v, st in avail.items():
                t, q = intercept(travel, st.pos, max(st.t, now), lambda s, g=g: believed(g, s), reach=0.8 * radio)
                bids[(v, g)] = (t, q)
        progress = False
        for v in sorted(avail, key=lambda v: avail[v].t):
            options = [(bids[(v, g)][0], g) for g in pending if (v, g) in bids]
            if not options:
                continue
            t, g = min(options)
            if g in told:
                continue
            q = bids[(v, g)][1]
            bundles[v].append(g)
            bundles.setdefault(g, [])
            told.add(g)
            when[g] = (t, q)
            avail[v] = Start(q, t)
            pending.remove(g)
            progress = True
        for g in list(when):
            if g not in avail:
                avail[g] = Start(when[g][1], when[g][0])
        if not progress:
            break
    return bundles, when


def allocate(travel, starts, tasks, order, robots, work, knowing, need_all, believed, now, radio, fixed=()):
    """The three algorithms in turn. robots: who may be assigned; knowing:
    {robot: Start} of the robots in contact; need_all: every robot must be
    told (Case I), and not only those assigned. Returns (bundles, gossip
    bundles, expected telling times)."""
    bundles = bundling(travel, starts, tasks, order, robots)
    bundles = minimise(travel, starts, tasks, bundles, work, fixed)
    assigned = [j for j, b in bundles.items() if b]
    need = list(starts) if need_all else assigned
    gb, when = gossip(travel, knowing, need, believed, now, radio)
    return bundles, gb, when
