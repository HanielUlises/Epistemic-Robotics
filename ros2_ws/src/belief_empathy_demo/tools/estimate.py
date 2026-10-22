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
Times and decisions for the latent-tool-fault variant, without a simulator.

A fault pattern is (d1, t1, d2, t2): the drive faults and whether the tool has
failed, of g1 and of g2. A drive fault puts a vehicle on its next particle and
slows it; a tool fault is latent, and only an attempt at the task shows it.

The kinematic model is the layout's: particles move along the known routes at
their speeds, the aerial vehicle flies straight at layout.V_UAV and meets a
particle by the shortest intercept, a ground vehicle drives the roadmap at
layout.DRIVE of its drive faults, a hail answered takes HAIL_MET seconds and
one unanswered layout.HAIL_WINDOW, and an attempt layout.DELTA seconds with
both at the site. A ground vehicle that is sent drives on its own; the aerial
vehicle goes on with what it does next.

Three things are built on it:

  Procedure      the paper's procedure, in two strengths. `published`: each
                 vehicle's particles in order of likelihood, no joint fault
                 budget, the vehicle of least estimated makespan sought and
                 sent, and a vehicle whose attempt fails dropped. `strong`:
                 the set of fault patterns consistent with every hail answer,
                 every attempt, the budget and the assumption that some tool
                 works; the plans considered are every assignment of the task
                 (one vehicle, or both) and every order of visits, valued by
                 their mean makespan over that set; attempts in order of
                 arrival; everything estimated again after each observation.
                 `ideal` is `strong` knowing where every vehicle is.
  run_policy     the time an epistemic policy takes in a pattern.
  run_procedure  the time a procedure takes in a pattern, deciding as it does.

Used by tools/precheck.py before any run, and by scripts/mission.py, which
asks Procedure for its decisions during a run.
"""

import itertools
import math

import layout as L

HAIL_MET = 1.0
PATTERNS = [p for p in itertools.product((0, 1, 2), (0, 1), (0, 1, 2), (0, 1))
            if sum(p) <= 2 and not (p[1] and p[3])]
VEHICLES = ('g1', 'g2')


def faults(p, g):
    return p[0] if g == 'g1' else p[2]


def tool_fault(p, g):
    return bool(p[1] if g == 'g1' else p[3])


def drive_to_site(pos, site, d):
    return L.length(L.ground_path(pos, site)) / L.DRIVE[min(d, 2)]


def fly(a, b):
    return math.dist(a, b) / L.V_UAV


# ─────────────────────────────────────────────────────────────── a world ──

class World:
    """What actually happens in one pattern, from a state of the run."""

    def __init__(self, pattern, site):
        self.p, self.site = pattern, tuple(site)

    def at(self, g, t):
        return L.particle(g, faults(self.p, g) + 1, t)

    def seek(self, u, t, g, order):
        """The aerial vehicle at u at time t seeks g at the particles of
        `order` in turn; where and when it meets g, and the particles it found
        empty."""
        empty = []
        for b in order:
            q, t = L.intercept(u, t, g, b)
            u = q
            if b == faults(self.p, g) + 1:
                return u, t + HAIL_MET, empty
            t += L.HAIL_WINDOW
            empty.append(b)
        raise ValueError(f'{g} not at any of {order}')

    def send(self, g, t):
        """When g, sent from its particle at time t, reaches the site."""
        return t + drive_to_site(self.at(g, t), self.site, faults(self.p, g))


# ──────────────────────────────────────────────────────────── the policy ──

def run_policy(task, policy, pattern, site, u, t, arrival_order=False, spare_slow=False, drive_blocks=False):
    """Follow an epistemic policy in a pattern from the aerial vehicle at u at
    time t; the time the task is done, and the steps taken.

    Two counterfactuals, for taking apart how its time differs from a
    procedure's: with arrival_order, the attempts go to the vehicles it has
    sent in the order they reach the site, and not in the policy's; with
    spare_slow, where the policy would fly to a vehicle's third particle to
    send it, the aerial vehicle fetches the other vehicle, which the fault
    budget then says is sound.

    With drive_blocks, a drive to the site answers on arrival, as the
    replica's ground vehicles do, and ePlanSys's executor, which runs
    consecutive actions on disjoint agents as one group and waits for the
    group, holds the aerial vehicle's second action after the drive until the
    vehicle arrives."""
    w = World(pattern, site)
    hold, first = None, False
    labels = task['initial-state']['labels']
    arrive, steps, node = {}, [], policy
    events = {a: v['designated'] for a, v in task['actions'].items()}
    while node is not None:
        name = node['action']
        act, *args = name.split('_')
        pick = node['branches'][0]
        if len(node['branches']) > 1:
            for b in node['branches']:
                e = events[name][b['event']]
                if e.startswith('e-met-') or e.startswith('e-absent-'):
                    g, k = args[1], int(e[-1])
                    here = faults(pattern, g) + 1 == k
                    if here == e.startswith('e-met-'):
                        pick = b
                        break
                elif e in ('e-done', 'e-fail'):
                    if tool_fault(pattern, args[1]) == (e == 'e-fail'):
                        pick = b
                        break
        if spare_slow and act == 'back-p3':
            other = 'g1' if args[1] == 'g2' else 'g2'
            u, t = L.intercept(u, t, other, faults(pattern, other) + 1)
            t += HAIL_MET
            arrive[other] = w.send(other, t)
            t = max(t + fly(u, site), arrive[other]) + L.DELTA
            steps.append(f'fetch {other} instead')
            return t, steps
        if arrival_order and act == 'perform':
            t, u = t + fly(u, site), site
            for g in sorted(arrive, key=arrive.get):
                t = max(t, arrive[g]) + L.DELTA
                if not tool_fault(pattern, g):
                    steps.append(f'attempt {g} first: done')
                    return t, steps
                steps.append(f'attempt {g} first: failed')
            return None, steps
        if hold is not None and act != 'go-site':
            if first:
                first = False           # in the drive's group: runs beside it
            else:
                t, hold = max(t, hold), None
        if act in ('fly-p1', 'back-p2', 'back-p3'):
            u, t = L.intercept(u, t, args[1], int(act[-1]))
        elif act.startswith('hail-p'):
            g, k = args[1], int(act[-1])
            t += HAIL_MET if faults(pattern, g) + 1 == k else L.HAIL_WINDOW
        elif act == 'go-site':
            arrive[args[0]] = w.send(args[0], t)
            if drive_blocks:
                hold, first = arrive[args[0]], True
        elif act == 'fly-site':
            t, u = t + fly(u, site), site
        elif act == 'perform':
            g = args[1]
            t = max(t, arrive[g]) + L.DELTA
            if not tool_fault(pattern, g):
                steps.append(f'{name}: done')
                return t, steps
        if act.startswith('hail-p'):
            k = int(act[-1])
            steps.append(f"{name}: {'e-met' if faults(pattern, args[1]) + 1 == k else 'e-absent'}-{k}")
        else:
            steps.append(name if len(node['branches']) == 1 else f'{name}: {events[name][pick["event"]]}')
        node = pick['subtree']
    return None, steps


# ───────────────────────────────────────────────────────── the procedure ──

class Procedure:
    """The paper's procedure, `published`, `strong` or `ideal`, as a decision
    maker: it holds what the aerial vehicle has learned and, asked, says what
    to do next."""

    def __init__(self, strength, site):
        self.strength, self.site = strength, tuple(site)
        self.patterns = list(PATTERNS)
        self.empty = {g: set() for g in VEHICLES}     # particles hailed and found empty
        self.found = {}                               # g: drive faults, learned at a meeting
        self.sent = {}                                # g: time it reaches the site
        self.failed = set()

    # what is known
    def observe_hail(self, g, b, met):
        if met:
            self.found[g] = b - 1
        else:
            self.empty[g].add(b)
        self._prune()

    def observe_attempt(self, g, done):
        if not done:
            self.failed.add(g)
        self._prune()

    def observe_drives(self, d):
        """The ideal method: every vehicle's drive faults, known at once."""
        self.found.update(d)
        self._prune()

    def _prune(self):
        def fits(p):
            for g in VEHICLES:
                if g in self.found and faults(p, g) != self.found[g]:
                    return False
                if faults(p, g) + 1 in self.empty[g]:
                    return False
                if g in self.failed and not tool_fault(p, g):
                    return False
            return True
        if self.strength != 'published':
            self.patterns = [p for p in PATTERNS if fits(p)]

    def candidates(self, g):
        """The particles g may still hold, most likely first: by the patterns
        left, or for the published procedure in the paper's fixed order."""
        if g in self.found:
            return [self.found[g] + 1]
        left = [b for b in (1, 2, 3) if b not in self.empty[g]]
        if self.strength == 'published':
            return left
        count = {b: sum(1 for p in self.patterns if faults(p, g) + 1 == b) for b in left}
        return sorted((b for b in left if count[b]), key=lambda b: (-count[b], b))

    # what to do
    def value(self, plan, u, t, pattern):
        """The makespan of a plan in a pattern: visit the vehicles of `plan`
        not yet sent, in order, sending each when met, fly to the site and
        attempt in order of arrival; if every one fails, seek and send the
        other and attempt with it."""
        w = World(pattern, self.site)
        arrive = dict(self.sent)
        for g in plan:
            if g in arrive:
                continue
            order = [b for b in self.candidates(g)]
            if faults(pattern, g) + 1 not in order:
                order.append(faults(pattern, g) + 1)
            u, t, _ = w.seek(u, t, g, order)
            arrive[g] = w.send(g, t)
        t, u = t + fly(u, self.site), self.site
        for g in sorted((g for g in arrive if g not in self.failed), key=lambda g: arrive[g]):
            t = max(t, arrive[g]) + L.DELTA
            if not tool_fault(pattern, g):
                return t
        rest = [g for g in VEHICLES if g not in arrive and g not in self.failed]
        if not rest:
            return math.inf
        g = rest[0]
        order = self.candidates(g)
        if faults(pattern, g) + 1 not in order:
            order = order + [faults(pattern, g) + 1]
        u, t, _ = w.seek(u, t, g, order)
        arr = w.send(g, t)
        t = max(t + fly(u, self.site), arr) + L.DELTA
        return t

    def estimate_published(self, g, u, t):
        """The paper's estimate for g: met and sent, or sought at its most
        likely particle, at that particle's speed."""
        if g in self.sent:
            return max(t + fly(u, self.site), self.sent[g])
        b = self.candidates(g)[0]
        q, when = L.intercept(u, t, g, b)
        return when + HAIL_MET + max(fly(q, self.site), drive_to_site(q, self.site, b - 1))

    def plans(self):
        live = [g for g in VEHICLES if g not in self.failed]
        out = []
        for k in (1, 2):
            for combo in itertools.permutations(live, k):
                out.append(combo)
        return out

    def decide(self, u, t):
        """('seek', g, b): fly to g's particle b and hail; ('site',): every
        vehicle needed is sent, fly to the site and attempt; with the values
        it compared."""
        if self.strength == 'published':
            live = [g for g in VEHICLES if g not in self.failed]
            if any(g in self.sent for g in live):
                return ('site',), {}
            options = {g: self.estimate_published(g, u, t) for g in live if self.candidates(g)}
            g = min(options, key=options.get)
            return ('seek', g, self.candidates(g)[0]), options
        options = {}
        for plan in self.plans():
            vals = [self.value(plan, u, t, p) for p in self.patterns]
            options[plan] = sum(vals) / len(vals)
        plan = min(options, key=lambda k: (options[k], len(k)))
        todo = [g for g in plan if g not in self.sent]
        if not todo:
            return ('site',), options
        g = todo[0]
        return ('seek', g, self.candidates(g)[0]), options


def run_procedure(strength, pattern, site, u, t):
    """Run a procedure in a pattern from the aerial vehicle at u at time t;
    the time the task is done, and the steps."""
    proc = Procedure(strength, site)
    w = World(pattern, site)
    steps = []
    if strength == 'ideal':
        proc.observe_drives({g: faults(pattern, g) for g in VEHICLES})
    for _ in range(20):
        what, _ = proc.decide(u, t)
        if what[0] == 'seek':
            g, b = what[1], what[2]
            if strength == 'ideal':
                proc.sent[g] = w.send(g, t)
                steps.append(f'tell {g}')
                continue
            u, t = L.intercept(u, t, g, b)
            met = faults(pattern, g) + 1 == b
            t += HAIL_MET if met else L.HAIL_WINDOW
            proc.observe_hail(g, b, met)
            steps.append(f'{g}@{b}{"+" if met else "-"}')
            if met:
                if strength == 'published':
                    # the paper's procedure, met, estimates again with what it learned
                    others = [h for h in VEHICLES if h != g and h not in proc.failed and proc.candidates(h)]
                    here = max(t + fly(u, site), w.send(g, t))
                    alt = {h: proc.estimate_published(h, u, t) for h in others}
                    if alt and min(alt.values()) < here:
                        continue
                proc.sent[g] = w.send(g, t)
        else:
            t, u = t + fly(u, site), site
            for g in sorted((g for g in proc.sent if g not in proc.failed), key=lambda g: proc.sent[g]):
                t = max(t, proc.sent[g]) + L.DELTA
                done = not tool_fault(pattern, g)
                proc.observe_attempt(g, done)
                steps.append(f'attempt {g}: {"done" if done else "failed"}')
                if done:
                    return t, steps
            proc.sent = {g: a for g, a in proc.sent.items() if g not in proc.failed}
    return None, steps
