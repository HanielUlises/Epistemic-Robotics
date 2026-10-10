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
The map-sharing protocols the epistemic strategy is compared with, simulated
at the level of each robot's reading of each bay.

A reading is a value, 'x' blocked or 'o' open, the time it was taken on the
observer's own clock, the version of the bay it reflects (the forklift slot
it saw, 0 for the shift map), and who took it. Observation is transient: a
robot sees a change as it happens, at its station, and has moved on by the
time maps are sent; nothing re-reads a bay afterwards. A message is one
robot's reading of one bay sent to one robot, the unit a policy's map report
is counted in.

Protocols:

  flood     every robot sends every linked robot its reading of every bay,
            then re-sends a bay to a neighbour only when its reading has
            changed since it last sent it there, round after round until
            nothing changes; relays reach robots no link joins directly
  pull      each hauler asks every neighbour about every bay, one request
            and one reply per neighbour and bay, and then chooses as the
            others do; no relays, so a hauler hears only its neighbours
  gossip    random linked pairs exchange their whole maps

Merge rules, for a bay both robots have read:

  recency     the reading taken later, by its observer's clock
  version     the reading of the later forklift slot: the schedule, which
              every robot knows, used as a logical clock
  overwrite   the sender's
  confidence  the receiver's: epistemic_slam::fuse on readings of 0 and 100

A budget caps the messages a protocol may send; it stops when it is spent.
Then every hauler acts on its map: a solo hauler crosses the lowest-numbered
bay its map shows open, a pair crosses together only if both choose the same
bay, and the shift succeeds when every crossing is through an open bay and,
under secrecy, no contractor's map holds the secret change.
"""

import random

RULES = ('recency', 'version', 'overwrite', 'confidence')


class Reading:
    __slots__ = ('value', 'stamp', 'version', 'origin')

    def __init__(self, value, stamp, version, origin):
        self.value, self.stamp, self.version, self.origin = value, stamp, version, origin


def initial_maps(inst):
    """Every robot's reading of every bay after the forklift, before any
    message. Elimination: readings of the bays a robot saw, and none of the
    rest."""
    maps = {}
    if inst.regime == 'elimination':
        for a in inst.agents:
            maps[a] = {t: Reading('o' if t == inst.open_bay else 'x', 0.0, 1, a)
                       for t in inst.sees_at_start[a]}
        return maps
    for a in inst.agents:
        off = inst.clock_offset.get(a, 0.0)
        maps[a] = {t: Reading('x' if t in inst.shift_blocked else 'o', off, 0, a) for t in inst.bays}
    for k, c in enumerate(sorted(inst.changes, key=lambda c: c.time), start=1):
        for a in c.observers:
            off = inst.clock_offset.get(a, 0.0)
            maps[a][c.bay] = Reading('x' if c.kind == 'stage' else 'o', c.time + off, k, a)
    return maps


def merge(mine, theirs, rule):
    """The receiver's reading after it is sent `theirs`."""
    if mine is None:
        return theirs
    if rule == 'recency':
        return theirs if theirs.stamp > mine.stamp else mine
    if rule == 'version':
        return theirs if theirs.version > mine.version else mine
    if rule == 'overwrite':
        return theirs
    return mine


class Run:
    """One protocol on one instance, sending until done or out of budget."""

    def __init__(self, inst, rule, budget, seed):
        self.inst, self.rule, self.budget = inst, rule, budget
        self.rng = random.Random(seed)
        self.maps = initial_maps(inst)
        self.sent = 0
        self.requests = 0
        self.log = []          # every reading sent, (sender, receiver, bay), in order
        self.overwrote_fresh = 0
        self.truth = {t: ('x' if t in inst.actual_blocked() else 'o') for t in inst.bays}

    def spent(self):
        return self.budget is not None and self.sent >= self.budget

    def send(self, i, j, t):
        """i sends j its reading of t. False when the budget is spent or i has
        no reading of t."""
        if self.spent():
            return False
        r = self.maps[i].get(t)
        if r is None:
            return False
        self.sent += 1
        self.log.append((i, j, t))
        before = self.maps[j].get(t)
        after = merge(before, r, self.rule)
        if before is not None and before.value == self.truth[t] and after.value != self.truth[t]:
            self.overwrote_fresh += 1
        self.maps[j][t] = after
        return True

    def neighbours(self, i):
        return [j for j in self.inst.agents if self.inst.linked(i, j)]


def flood(inst, rule, budget=None, seed=0):
    run = Run(inst, rule, budget, seed)
    last_sent = {}   # (i, j, t) -> (value, stamp, version) last sent
    while not run.spent():
        changed = False
        senders = list(inst.agents)
        run.rng.shuffle(senders)
        for i in senders:
            for j in run.neighbours(i):
                for t in inst.bays:
                    r = run.maps[i].get(t)
                    if r is None:
                        continue
                    key = (r.value, r.stamp, r.version)
                    if last_sent.get((i, j, t)) == key:
                        continue
                    if not run.send(i, j, t):
                        return run
                    last_sent[(i, j, t)] = key
                    changed = True
        if not changed:
            break
    return run


def pull(inst, rule, budget=None, seed=0):
    """Each hauler asks every neighbour about every bay, the ones it already
    believes open included, since its own reading may be stale. A request and
    its reply are two messages; a neighbour with no reading still replies.
    Stopping at the first bay a hauler then believes open saves messages and
    loses shifts: a change nobody saw leaves that bay open on every map, and
    only a fresher reading of another bay would steer the hauler away."""
    run = Run(inst, rule, budget, seed)
    for h in inst.haulers:
        for t in inst.bays:
            peers = run.neighbours(h)
            run.rng.shuffle(peers)
            for j in peers:
                if run.budget is not None and run.sent + 2 > run.budget:
                    return run
                run.sent += 1          # the request
                run.requests += 1
                if run.maps[j].get(t) is None:
                    run.sent += 1      # the reply that it has none
                    run.requests += 1
                    continue
                run.send(j, h, t)
    return run


def gossip(inst, rule, budget=None, seed=0, quiet=20, cap=5000):
    """Random linked pairs exchange whole maps; stops after `quiet`
    exchanges in a row change nothing, or at `cap` messages, which overwrite
    can otherwise churn past without settling."""
    run = Run(inst, rule, budget if budget is not None else cap, seed)
    edges = [(i, j) for k, i in enumerate(inst.agents) for j in inst.agents[k + 1:] if inst.linked(i, j)]
    still = 0
    while not run.spent() and still < quiet and edges:
        i, j = run.rng.choice(edges)
        before = snapshot(run.maps)
        for t in inst.bays:
            if not run.send(i, j, t) and run.spent():
                break
            if not run.send(j, i, t) and run.spent():
                break
        still = still + 1 if snapshot(run.maps) == before else 0
    return run


def snapshot(maps):
    return tuple((a, t, r.value, r.stamp) for a in sorted(maps) for t, r in sorted(maps[a].items()))


PROTOCOLS = {'flood': flood, 'pull': pull, 'gossip': gossip}


# ─── What the haulers then do ───────────────────────────────────────────────

def believes_open(m, t, inst, infer=True):
    r = m.get(t)
    if r is not None:
        return r.value == 'o'
    if inst.regime == 'elimination' and infer:
        # exactly one bay is open: the robot that knows every other shut
        # knows this one open
        others = [u for u in inst.bays if u != t]
        return all(m.get(u) is not None and m[u].value == 'x' for u in others)
    return False


def freshness(r, rule):
    """How fresh a reading is, as the hauler's merge rule judges it."""
    if r is None:
        return (-1, float('-inf'))
    if rule == 'version':
        return (r.version, 0.0)
    return (0, r.stamp)


def choice(m, inst, infer=True, rule='recency'):
    """The bay a hauler takes: among those its map shows open, the one whose
    reading is freshest, as its merge rule judges freshness, and the
    lowest-numbered of equals. A bay known open only by elimination ranks
    after every bay read open."""
    best, key = None, None
    for k, t in enumerate(inst.bays):
        if not believes_open(m, t, inst, infer):
            continue
        r = m.get(t)
        f = freshness(r, rule)
        cand = (f, -k)
        if key is None or cand > key:
            best, key = t, cand
    return best


def outcome(inst, maps, infer=True, rule='recency', crossed=None):
    """Whether the shift succeeds on these maps, and why not. With `crossed`,
    a hauler crosses the bay given there, as a plan has it, and must hold
    that bay open on its map; without, it chooses by `choice`."""
    truth_open = set(inst.bays) - inst.actual_blocked()

    def pick(h):
        if crossed is not None:
            t = crossed.get(h)
            return t if t is not None and believes_open(maps[h], t, inst, infer) else None
        return choice(maps[h], inst, infer, rule)

    for h in inst.solo_haulers():
        t = pick(h)
        if t is None:
            return False, 'a hauler has no bay it believes open'
        if t not in truth_open:
            return False, 'a hauler drives into a blocked bay'
    for h, k in inst.pairs:
        a, b = pick(h), pick(k)
        if a is None or b is None:
            return False, 'a pair: one has no bay it believes open'
        if a != b:
            return False, 'a pair goes to two bays'
        if a not in truth_open:
            return False, 'a pair drives into a blocked bay'
    for c in inst.contractors:
        for t in inst.secret_bays:
            r = maps[c].get(t)
            if r is not None and r.value == 'x':
                return False, 'a contractor holds the secret'
    return True, ''


def stale_entries(inst, maps):
    truth = {t: ('x' if t in inst.actual_blocked() else 'o') for t in inst.bays}
    return sum(1 for a in inst.agents for t in inst.bays
               if maps[a].get(t) is not None and maps[a][t].value != truth[t])


def needs_communication(inst):
    """True when the shift fails with no message at all."""
    ok, _ = outcome(inst, initial_maps(inst))
    return not ok
