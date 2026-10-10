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
Instances of the communication study: a fleet, a set of bays, the changes a
forklift makes to them, who sees each change, and the regime's extras.

An instance fixes everything the strategies are compared on, so that every
strategy runs on the same instance and a comparison between two of them is
paired. The four regimes differ in what they add:

  budget     a cap on the number of maps of a bay sent, and optionally a
             radio graph: two robots exchange maps only along an edge
  secrecy    some changes are secret from a set of contractor robots, which
             must end the shift not believing them
  joint      haulers come in pairs that must cross the same bay together,
             each believing it open and believing the other does
  skew       a bay changes twice, seen by different robots, and each robot's
             clock is off by a random offset
  elimination  exactly one bay is open; robots see some bays at the start,
             and the open bay may be seen by nobody (an S5 instance)
"""

import dataclasses
import random
from typing import Dict, List, Optional, Set, Tuple


@dataclasses.dataclass
class Change:
    bay: str
    kind: str                      # 'stage' or 'clear'
    observers: Set[str]
    time: float                    # when it happens, seconds into the shift


@dataclasses.dataclass
class Instance:
    regime: str
    seed: int
    agents: List[str]
    bays: List[str]
    shift_blocked: Set[str]
    changes: List[Change]
    haulers: List[str]
    links: Optional[Set[Tuple[str, str]]] = None      # undirected; None is every pair
    contractors: Set[str] = dataclasses.field(default_factory=set)
    secret_bays: Set[str] = dataclasses.field(default_factory=set)
    pairs: List[Tuple[str, str]] = dataclasses.field(default_factory=list)
    clock_offset: Dict[str, float] = dataclasses.field(default_factory=dict)
    # elimination only: which bay is open, and which bays each robot sees
    open_bay: Optional[str] = None
    sees_at_start: Dict[str, Set[str]] = dataclasses.field(default_factory=dict)

    # ── the floor after the forklift ────────────────────────────────────────

    def actual_blocked(self):
        if self.regime == 'elimination':
            return set(self.bays) - {self.open_bay}
        out = set(self.shift_blocked)
        for c in self.changes:
            (out.add if c.kind == 'stage' else out.discard)(c.bay)
        return out

    def linked(self, i, j):
        if self.links is None:
            return i != j
        return (i, j) in self.links or (j, i) in self.links

    def solo_haulers(self):
        paired = {h for p in self.pairs for h in p}
        return [h for h in self.haulers if h not in paired]

    def to_row(self):
        """The instance's parameters, for a CSV row."""
        return {
            'regime': self.regime, 'seed': self.seed, 'robots': len(self.agents),
            'bays': len(self.bays), 'haulers': len(self.haulers),
            'changes': len(self.changes),
            'links': len(self.links) if self.links is not None else -1,
            'contractors': len(self.contractors), 'pairs': len(self.pairs),
        }


def _agents(n):
    return [f'r{k + 1}' for k in range(n)]


def _bays(b):
    return [f't{k + 1}' for k in range(b)]


def _observers(rng, agents, p):
    return {a for a in agents if rng.random() < p}


def stale(n, b, seed, p_see=0.3, haulers=None, regime='budget'):
    """The stale-maps setting for n robots and b bays: at the shift's start
    the first bay is open and the rest blocked; the forklift stages a load in
    the first bay and clears the last. Each robot sees each change with
    probability p_see. At least one robot sees the clearing, or no robot can
    ever know the open bay and no strategy has anything to send."""
    rng = random.Random(seed)
    agents, bays = _agents(n), _bays(b)
    while True:
        staged = _observers(rng, agents, p_see)
        cleared = _observers(rng, agents, p_see)
        if cleared:
            break
    changes = [Change(bays[-1], 'clear', cleared, 10.0), Change(bays[0], 'stage', staged, 20.0)]
    h = haulers if haulers is not None else max(1, n // 3)
    return Instance(regime, seed, agents, bays, set(bays[1:]), changes, rng.sample(agents, h))


def budget(n, b, seed, p_see=0.3, p_link=None):
    """The stale setting with, when p_link is given, a radio graph in which
    each pair of robots is linked with probability p_link; the graph is
    redrawn until it is connected, since a robot nobody can reach is a
    different question."""
    inst = stale(n, b, seed, p_see, regime='budget')
    if p_link is not None:
        rng = random.Random(seed * 7919 + 17)
        while True:
            links = {(i, j) for k, i in enumerate(inst.agents) for j in inst.agents[k + 1:]
                     if rng.random() < p_link}
            if _connected(inst.agents, links):
                break
        inst.links = links
    return inst


def _connected(agents, links):
    seen, todo = {agents[0]}, [agents[0]]
    while todo:
        a = todo.pop()
        for i, j in links:
            for x, y in ((i, j), (j, i)):
                if x == a and y not in seen:
                    seen.add(y)
                    todo.append(y)
    return len(seen) == len(agents)


def secrecy(n, b, seed, p_see=0.3, contractors=None, hauling=False):
    """The stale setting in which the staging of the first bay is secret from
    a set of contractor robots, robots that did not see it. They must end the
    shift not believing the bay blocked. With `hauling`, one of them is a
    hauler, which needs a route and must not learn the secret on the way to
    it; otherwise no contractor hauls."""
    inst = stale(n, b, seed, p_see, regime='secrecy')
    rng = random.Random(seed * 104729 + 3)
    staged = inst.changes[1]
    k = contractors if contractors is not None else max(1, n // 4)
    if hauling:
        hp = [h for h in inst.haulers if h not in staged.observers]
        if not hp:
            # no hauler missed the staging: make one that did a hauler
            pool0 = [a for a in inst.agents if a not in staged.observers]
            inst.haulers[0] = rng.choice(pool0)
            hp = [inst.haulers[0]]
        first = {rng.choice(hp)}
        pool = [a for a in inst.agents if a not in staged.observers and a not in inst.haulers]
        inst.contractors = first | set(rng.sample(pool, min(k - 1, len(pool))))
        inst.secret_bays = {staged.bay}
        return inst
    pool = [a for a in inst.agents if a not in staged.observers and a not in inst.haulers]
    inst.contractors = set(rng.sample(pool, min(k, len(pool))))
    inst.secret_bays = {staged.bay}
    return inst


def joint(n, b, seed, p_see=0.3, pairs=1):
    """The stale setting in which haulers come in pairs that must take one
    load through one bay together."""
    rng = random.Random(seed * 15485863 + 11)
    inst = stale(n, b, seed, p_see, haulers=2 * pairs, regime='joint')
    hs = list(inst.haulers)
    rng.shuffle(hs)
    inst.pairs = [(hs[2 * k], hs[2 * k + 1]) for k in range(pairs)]
    return inst


def skew(n, b, seed, p_see=0.3, sigma=0.0, gap=10.0):
    """Every bay starts blocked. The forklift clears bay t2 and leaves it
    open; it clears the last bay and stages a load in it again `gap` seconds
    later, so that two robots can hold contradicting readings of it, each
    right when it was taken. Each change is seen by its own random set of
    robots, and each robot's clock is off by a normal offset of standard
    deviation sigma."""
    rng = random.Random(seed)
    agents, bays = _agents(n), _bays(b)
    while True:
        first = _observers(rng, agents, p_see)
        second = _observers(rng, agents, p_see)
        other = _observers(rng, agents, p_see)
        if first and second and other:
            break
    t_twice, t_open = bays[-1], bays[1]
    changes = [Change(t_open, 'clear', other, 15.0),
               Change(t_twice, 'clear', first, 10.0),
               Change(t_twice, 'stage', second, 10.0 + gap)]
    changes.sort(key=lambda c: c.time)
    rng2 = random.Random(seed * 2654435761 % (2 ** 31))
    offsets = {a: rng2.gauss(0.0, sigma) for a in agents}
    h = max(1, n // 3)
    return Instance('skew', seed, agents, bays, set(bays), changes,
                    rng.sample(agents, h), clock_offset=offsets)


def elimination(n, b, seed, p_see=0.3):
    """Exactly one of b bays is open, which is common knowledge, and which one
    is not. Each robot sees each bay with probability p_see at the start of
    the shift; the open bay is drawn so that it may be seen by nobody. The
    instance is redrawn until the robots together see every bay but at most
    one, so that the open bay is distributed knowledge of the fleet."""
    rng = random.Random(seed)
    agents, bays = _agents(n), _bays(b)
    while True:
        sees = {a: {t for t in bays if rng.random() < p_see} for a in agents}
        seen = set().union(*sees.values())
        open_bay = rng.choice(bays)
        if len(set(bays) - seen) <= 1 and (set(bays) - seen) <= {open_bay}:
            break
    h = max(1, n // 3)
    return Instance('elimination', seed, agents, bays, set(), [], rng.sample(agents, h),
                    open_bay=open_bay, sees_at_start=sees)
