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
The kinematic simulator of the paper's Section 6.

    run(area, n, case, method, seed) -> result

Robots are single integrators with Gaussian process noise, eq. (1), a range
sensor and a communication disk. The methods:

proposed   the paper's framework. Out of contact a robot tracks its particle
           of the common belief (belief.py). A robot that finds a task
           allocates by Algorithms 1 to 3 (alloc.py) with what it knows,
           bidding for robots out of contact from its belief about them, and
           tells the robots it must by going to where it believes them to be.
           A robot found absent there is excluded, eq. (20)'s first ruleset,
           and the robot that sought it allocates again. A task that the
           robot believes a teammate found first, because the teammate's
           particle saw it earlier, is left to that teammate (the second
           ruleset) until they are in contact. Robots that meet share maps,
           tasks and the newest allocation; when all are in contact the
           common belief is reset, eq. (14).
ideal      the same framework with a communication disk as large as the area.
flock      the robots never leave each other's range: a formation whose
           centre explores the frontier of the shared map, and goes, all
           together, to every task found.
epistemic  the proposed framework with the decision at a task found made by
           a policy of the EPDDL model instead of the auctions
           (epistemic.py).

Case I: one task, which every robot knows to exist; once it is found every
robot must learn so, and those the task does not need go home. The mission
ends when the task is done and every robot is home. Case II: an unknown number
of tasks; robots go back to their particles after a task, and the mission ends
when every task is done, the whole area has been seen and every robot is home.
"""

import collections
import math

import numpy as np

from . import alloc, nav
from .belief import CommonBelief
from .env import FREE, OCC, UNKNOWN

WAIT_GRACE = 30.0      # s a robot waits at a task past the expected start before it allocates again
EXCLUDE_FOR = 60.0     # s a robot found absent stays excluded, unless met before
HORIZON = 900.0


class Knowledge:
    """What a robot knows that it can share: the tasks it knows of, the
    newest allocation it has heard of, whom it has excluded, and when it last
    heard from each robot (the timestamps q of Section 5.3.1)."""

    def __init__(self, ids):
        self.tasks = {}          # tid -> {'pos', 'k', 'found_t', 'found_by', 'done'}
        self.plan = None         # {'version', 'bundles', 'gb', 'told', 'fixed', 'expect'}
        self.excluded = {}       # robot -> time
        self.heard = {r: 0.0 for r in ids}
        self.assumed = {}        # tid -> robot believed to have found it first

    def merge(self, other, now):
        for tid, t in other.tasks.items():
            mine = self.tasks.get(tid)
            if mine is None:
                self.tasks[tid] = dict(t)
            else:
                mine['done'] = mine['done'] or t['done']
                mine['handled'] = mine.get('handled', False) or t.get('handled', False)
                if t['found_t'] < mine['found_t']:
                    mine['found_t'], mine['found_by'] = t['found_t'], t['found_by']
        if other.plan is not None and (self.plan is None or other.plan['version'] > self.plan['version']):
            self.plan = other.plan
        elif other.plan is not None and self.plan is not None and other.plan['version'] == self.plan['version']:
            self.plan['told'] |= other.plan['told']
        for r, t in other.excluded.items():
            self.excluded[r] = max(t, self.excluded.get(r, -math.inf))
        for r, t in other.heard.items():
            self.heard[r] = max(t, self.heard.get(r, 0.0))
        for tid in list(self.assumed):
            if tid in other.tasks and other.tasks[tid]['found_by'] == self.assumed[tid]:
                del self.assumed[tid]


class Robot:

    def __init__(self, rid, pos, area):
        self.id = rid
        self.pos = tuple(pos)
        self.known = area.walls_map()
        self.seen = np.zeros(area.occ.shape, dtype=bool)
        self.version = 0
        self.mover = nav.Mover(area.spec)
        self.k = None
        self.jobs = []
        self.job = None
        self.status = 'explore'
        self.plan_seen = None
        self.wait_since = None
        self.travelled = 0.0
        self.cell = None
        self.pjob = None


class Sim:

    def __init__(self, area, n, case, method, seed, record=False):
        self.area, self.spec = area, area.spec
        self.case, self.method = case, method
        self.rng = np.random.default_rng(seed + 7919)
        self.ids = [f'r{i + 1}' for i in range(n)]
        h = self.spec.home
        span = min(3.0, 0.8 * self.spec.home_radius)
        self.robots = {}
        for i, r in enumerate(self.ids):
            off = -span / 2 + span * i / max(1, n - 1)
            self.robots[r] = Robot(r, (h[0] + off, h[1]), area)
            self.robots[r].k = Knowledge(self.ids)
        self.tasks = {t['id']: dict(t, done=False, done_t=None, found_t=None, found_by=None) for t in area.tasks}
        self.t = 0.0
        self.radio = 1e9 if method == 'ideal' else self.spec.radio
        self.cb = None
        self.last_sync = -math.inf
        self.sync_occ = 0
        self.sync_due = True
        self.events = []
        self.record = record
        self.frames = []
        self.counts = collections.Counter()
        self.flock = None
        self.planner = None
        if method == 'epistemic':
            from .epistemic import Planner
            self.planner = Planner(self)
        for r in self.robots.values():
            self.area.sense(r.pos, r.known, r.seen)

    # ───────────────────────────────────────────────────────────── helpers ──

    def log(self, kind, **kw):
        self.events.append(dict(kw, t=round(self.t, 2), kind=kind))

    def blocked(self, r):
        return r.known == OCC

    def components(self):
        ids = self.ids
        adj = {r: set() for r in ids}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                if math.dist(self.robots[a].pos, self.robots[b].pos) <= self.radio:
                    adj[a].add(b)
                    adj[b].add(a)
        seen, comps = set(), []
        for r in ids:
            if r in seen:
                continue
            stack, comp = [r], set()
            while stack:
                x = stack.pop()
                if x in comp:
                    continue
                comp.add(x)
                stack.extend(adj[x] - comp)
            seen |= comp
            comps.append(comp)
        return comps

    def site_point(self, r, tid):
        """Where robot r stands at a task: the task itself, or, with
        spec.slot, its own place on a circle around it."""
        q = self.tasks[tid]['pos']
        return self.slot_of(r, q)

    def home_point(self, r):
        return self.slot_of(r, self.spec.home)

    def slot_of(self, r, q):
        if not self.spec.slot:
            return tuple(q)
        i = self.ids.index(r.id if hasattr(r, 'id') else r)
        a = 2 * math.pi * i / len(self.ids) + math.pi / 2
        return (q[0] + self.spec.slot * math.cos(a), q[1] + self.spec.slot * math.sin(a))

    def particle(self, r, t=None):
        return self.cb.pos(r, self.t if t is None else t)

    def believed(self, me, g, t):
        """Where robot `me` believes g to be at t: with g's own work, if the
        allocation it holds gave g work and g has been told of it, then the
        particle: the belief with the most recent timestamp."""
        plan = me.k.plan
        if plan is not None and g in plan['told'] and plan['expect'].get(g):
            for (t0, t1, q) in plan['expect'][g]:
                if t0 <= t <= t1:
                    return q
        return self.particle(g, t)

    def undone(self, k):
        return [tid for tid, x in sorted(k.tasks.items(), key=lambda kv: kv[1]['found_t']) if not x['done']]

    def task_view(self, k):
        return {tid: {'pos': x['pos'], 'k': x['k']} for tid, x in k.tasks.items()}

    # ─────────────────────────────────────────────────────── communication ──

    def communicate(self):
        comps = self.components()
        if len(comps) == 1:
            changed = self.share(comps[0])
            # the common belief is reset when all are in contact and new
            # knowledge is shared, eq. (14): tasks, allocations, or obstacles
            # nobody knew; coverage the belief did not predict is shared at
            # the latest every spec.resync seconds
            occ = int((self.robots[self.ids[0]].known == OCC).sum())
            if changed or self.sync_due or occ != self.sync_occ or self.t - self.last_sync >= self.spec.resync:
                self.sync_occ = occ
                self.global_sync()
            return
        for comp in comps:
            if len(comp) > 1:
                self.share(comp)

    def share(self, comp):
        """Robots in contact share maps, tasks and the newest allocation; a
        seeker whose quarry is here has met it."""
        rs = [self.robots[r] for r in sorted(comp)]
        before = {r.id: (r.k.plan['version'] if r.k.plan else None, len(r.k.tasks),
                         sum(x['done'] for x in r.k.tasks.values())) for r in rs}
        known = rs[0].known.copy()
        seen = rs[0].seen.copy()
        for r in rs[1:]:
            known = np.where(known == UNKNOWN, r.known, known)
            seen |= r.seen
        merged = Knowledge(self.ids)
        for r in rs:
            merged.merge(r.k, self.t)
        for r in comp:
            merged.heard[r] = self.t
            merged.excluded.pop(r, None)
        changed = False
        for r in rs:
            if not np.array_equal(r.known, known):
                r.known = known.copy()
                r.version += 1
            r.seen = seen.copy()
            tasks_before = before[r.id]
            k = Knowledge(self.ids)
            k.merge(merged, self.t)
            k.assumed = {tid: who for tid, who in r.k.assumed.items() if tid not in merged.tasks
                         or merged.tasks[tid]['found_by'] == who}
            if merged.plan is not None:
                k.plan = dict(merged.plan, told=set(merged.plan['told']) | set(comp))
            r.k = k
            after = (k.plan['version'] if k.plan else None, len(k.tasks), sum(x['done'] for x in k.tasks.values()))
            if after != tasks_before:
                changed = True
        # a seeker whose quarry is in contact has met it
        for r in rs:
            if r.job and r.job[0] == 'seek' and r.job[1] in comp:
                self.log('met', robot=r.id, other=r.job[1])
                self.counts['met'] += 1
                if self.planner:
                    self.planner.met(r, r.job[1])
                self.next_job(r)
        if self.planner:
            self.planner.contact(comp)
            return changed
        if len(comp) < len(self.ids):
            self.reallocate_if_needed(rs)
        for r in rs:
            self.adopt(r)
        return changed

    def reallocate_if_needed(self, rs):
        """In contact, robots that know of undone tasks the allocation they
        hold does not cover allocate again (the lowest-numbered of them)."""
        k = rs[0].k
        undone = self.undone(k)
        if not undone:
            return
        covered = set()
        if k.plan:
            for b in k.plan['bundles'].values():
                covered |= set(b)
        if all(tid in covered for tid in undone):
            return
        if any(tid in k.assumed for tid in undone) and all(tid in k.assumed or tid in covered for tid in undone):
            return
        self.allocate(rs[0], [r.id for r in rs])

    def global_sync(self):
        """Every robot in contact: one shared state, from which the common
        belief is propagated anew, eq. (14)."""
        if self.planner:
            self.planner.cancel_all()
        self.last_sync = self.t
        self.sync_due = False
        rs = [self.robots[r] for r in self.ids]
        k = rs[0].k
        undone = self.undone(k)
        if undone:
            covered = set()
            if k.plan:
                for b in k.plan['bundles'].values():
                    covered |= set(b)
            if not all(tid in covered for tid in undone) or k.plan['gb'] and any(k.plan['gb'].values()):
                self.allocate(rs[0], self.ids, quiet=True)
        k = rs[0].k
        if k.plan is not None:
            # in contact with all, everyone knows the allocation: nobody is left to tell
            k.plan = dict(k.plan, gb={g: [] for g in self.ids}, told=set(self.ids))
        k.excluded = {}
        for r in rs:
            r.k = k_copy(k, self.ids)
        prefix, after = {}, 'explore'
        found = any(True for _ in k.tasks)
        if self.case == 1 and found:
            after = 'home'
        plan = k.plan
        for r in rs:
            steps = []
            if plan is not None:
                for tid in plan['bundles'].get(r.id, []):
                    if not k.tasks[tid]['done']:
                        cohort = [j for j, b in plan['bundles'].items() if tid in b]
                        steps.append(('task', tid, k.tasks[tid]['pos'], cohort))
            if r.status in ('home', 'parked') and not steps and (self.case == 1 and found):
                steps = [('home',)]
            prefix[r.id] = steps
        starts = {r.id: r.pos for r in rs}
        self.cb = CommonBelief(self.area, self.t, starts, rs[0].known, rs[0].seen, prefix, after,
                               work={tid: x.get('work', self.spec.work) for tid, x in self.tasks.items()},
                               previous=self.cb)
        nothing_left = not len(frontier_points(rs[0].seen, rs[0].known, self.spec.cell))
        for r in rs:
            parked = r.status == 'parked'
            r.jobs, r.job = [], None
            r.status = 'explore'
            r.plan_seen = None
            self.adopt(r)
            if parked and r.status in ('explore', 'home') and (r.job is None or r.job[0] == 'home') and (
                    nothing_left or (self.case == 1 and found)):
                r.status, r.job, r.jobs = 'parked', None, []
            r.mover.reset()
        self.counts['sync'] += 1

    # ─────────────────────────────────────────────────────────── allocation ──

    def starts_for(self, me, comp):
        """The allocation state of every robot as `me` knows it."""
        travel = alloc.Travel(self.area, me.known, self.spec.v_max)
        starts = {}
        for g in self.ids:
            if g in comp:
                starts[g] = alloc.Start(self.robots[g].pos, self.t)
            else:
                t, q = alloc.intercept(travel, me.pos, self.t, lambda s, g=g: self.believed(me, g, s),
                                       reach=0.8 * self.radio)
                starts[g] = alloc.Start(q, t)
        return travel, starts

    def allocate(self, me, comp, quiet=False, reason='found'):
        """Algorithms 1 to 3 with what `me` knows, for the robots in contact
        with it (comp) and the rest by belief."""
        k = me.k
        undone = [tid for tid in self.undone(k) if tid not in k.assumed]
        if not undone:
            return
        avail = [g for g in self.ids if g in comp or not (g in k.excluded and self.t - k.excluded[g] < EXCLUDE_FOR)]
        travel, starts = self.starts_for(me, comp)
        tasks = self.task_view(k)
        fixed = set()
        if k.plan:
            for g, b in k.plan['bundles'].items():
                if g not in comp and g in k.plan['told']:
                    fixed |= set(b)
        need_k = sum(tasks[t]['k'] for t in undone)
        order = undone
        if len(avail) < max(tasks[t]['k'] for t in undone):
            # not enough robots believed free for some task: allocate those
            # that can be done, and keep the rest for later
            order = [t for t in undone if tasks[t]['k'] <= len(avail)]
            self.counts['short'] += 1
            self.log('short', robot=me.id, avail=avail, need=need_k)
            if not order:
                return
        knowing = {g: alloc.Start(self.robots[g].pos, self.t) for g in comp}
        bundles = alloc.bundling(travel, starts, tasks, order, avail)
        bundles = alloc.minimise(travel, starts, tasks, bundles, self.spec.work, fixed)
        need = self.to_tell(me, comp, bundles)
        gb, when = alloc.gossip(travel, knowing, need, lambda g, s: self.believed(me, g, s), self.t, self.radio)
        for g in self.ids:
            bundles.setdefault(g, [])
            gb.setdefault(g, [])
        sched = alloc.schedule(travel, {g: alloc.Start(when[g][1], when[g][0]) if g in when else starts[g]
                                        for g in self.ids}, tasks, bundles, self.spec.work)
        expect = {}
        if sched:
            tstart = sched[0]
            for g, b in bundles.items():
                spans = []
                for tid in b:
                    s0 = tstart[tid]
                    spans.append((s0 - 5.0, s0 + self.spec.work + 5.0, tasks[tid]['pos']))
                expect[g] = spans
        plan = {'version': (round(self.t, 3), me.id), 'bundles': bundles, 'gb': gb,
                'told': set(comp), 'fixed': fixed, 'expect': expect, 'tasks': list(order)}
        for g in comp:
            other = self.robots[g].k
            for tid, x in k.tasks.items():
                other.tasks.setdefault(tid, dict(x))
            other.plan = plan
        self.counts['allocations'] += 1
        if not quiet:
            self.log('allocate', robot=me.id, reason=reason, bundles={g: b for g, b in bundles.items() if b},
                     gossip={g: b for g, b in gb.items() if b})

    def to_tell(self, me, comp, bundles):
        """Who must be told of an allocation: in Case I every robot not yet
        told of the task, and in both cases every robot out of contact whose
        work is not what it was last told; never a robot excluded."""
        old = me.k.plan
        out = []
        for g in self.ids:
            if g in comp or (g in me.k.excluded and self.t - me.k.excluded[g] < EXCLUDE_FOR):
                continue
            told_before = old is not None and g in old['told']
            before = [tid for tid in (old['bundles'].get(g, []) if old else []) if not me.k.tasks[tid]['done']]
            now = [tid for tid in bundles.get(g, []) if not me.k.tasks[tid]['done']]
            if told_before and before == now:
                continue
            if now or (self.case == 1 and not told_before) or (told_before and before):
                out.append(g)
        return out

    def retry(self, r):
        """A robot that knows of a task no allocation it holds covers tries
        again every few seconds, as exclusions lapse."""
        k = r.k
        undone = [tid for tid in self.undone(k) if tid not in k.assumed]
        if not undone:
            return
        covered = set()
        if k.plan:
            for b in k.plan['bundles'].values():
                covered |= set(b)
        if all(tid in covered for tid in undone):
            return
        comp = next(c for c in self.components() if r.id in c)
        self.allocate(r, comp, reason='retry')
        for x in comp:
            self.adopt(self.robots[x])

    def adopt(self, r):
        """Take up the jobs of the allocation r holds, if it is new to r."""
        plan = r.k.plan
        v = plan['version'] if plan else None
        if v == r.plan_seen and v is not None:
            return
        r.plan_seen = v
        if plan is None:
            if self.case == 1 and r.k.tasks:
                r.jobs, r.job = [('home',)], None
                self.next_job(r)
            return
        jobs = []
        for g in plan['gb'].get(r.id, []):
            jobs.append(('seek', g))
        for tid in plan['bundles'].get(r.id, []):
            if not r.k.tasks[tid]['done']:
                jobs.append(('task', tid))
        if self.case == 1 and r.k.tasks:
            jobs.append(('home',))
        elif jobs:
            jobs.append(('resume',))
        if not jobs and r.status in ('seek', 'task'):
            jobs = [('resume',)]
        r.jobs = jobs
        r.job = None
        self.next_job(r)

    def next_job(self, r):
        r.mover.reset()
        r.wait_since = None
        if not r.jobs:
            r.job = None
            if r.status not in ('home', 'parked'):
                r.status = 'home' if (self.case == 1 and r.k.tasks) else 'explore'
            return
        r.job = r.jobs.pop(0)
        r.status = {'seek': 'seek', 'task': 'task', 'home': 'home', 'resume': 'explore'}[r.job[0]]
        if r.job[0] == 'resume':
            r.job = None
            if r.jobs:
                self.next_job(r)

    # ──────────────────────────────────────────────────────────── discovery ──

    def discover(self, r):
        for tid, x in self.tasks.items():
            if tid in r.k.tasks or x['done']:
                continue
            if not self.area.sees(r.pos, x['pos']):
                continue
            first = x['found_t'] is None
            if first:
                x['found_t'], x['found_by'] = self.t, r.id
            r.k.tasks[tid] = {'pos': x['pos'], 'k': x['k'], 'found_t': self.t, 'found_by': r.id, 'done': False}
            self.log('found', robot=r.id, task=tid, k=x['k'], first=first)
            self.sync_due = True
            if self.method in ('ideal', 'flock'):
                continue
            # the second ruleset: a teammate whose particle saw it earlier
            # is believed to have found it, and to be handling it
            cell = self.area.cell_of(x['pos'])
            comp = next(c for c in self.components() if r.id in c)
            earlier = [(self.cb.first_seen(g, cell), g) for g in self.ids
                       if g not in comp and self.cb.first_seen(g, cell) < self.t - self.spec.dt]
            # the epistemic method acts on what a robot knows: that a
            # teammate's particle saw the task is not knowledge that the
            # teammate did, so it does not apply the second ruleset
            if earlier and not self.planner:
                who = min(earlier)[1]
                r.k.assumed[tid] = who
                r.k.tasks[tid]['found_by'] = who
                self.log('assumed', robot=r.id, task=tid, by=who)
                self.counts['assumed'] += 1
                continue
            if x['k'] == 1 and not r.job:
                # a task for one robot is done by the robot that finds it
                pass
            if self.planner and len(comp) == len(self.ids):
                self.sync_due = True
            elif self.planner:
                self.planner.found(r, tid, comp)
            else:
                self.allocate(r, comp)
                for g in comp:
                    self.adopt(self.robots[g])

    # ──────────────────────────────────────────────────────────── behaviour ──

    def drive(self, r, goal, speed):
        near = nav.occupied_points(r.known, self.spec.cell, r.pos, self.spec.rho_o + 1.0)
        vx, vy = r.mover.step(r.pos, goal, self.blocked(r), r.version, speed, near)
        vx += self.rng.normal(0.0, self.spec.noise)
        vy += self.rng.normal(0.0, self.spec.noise)
        sp = math.hypot(vx, vy)
        if sp > self.spec.v_max:
            vx, vy = vx * self.spec.v_max / sp, vy * self.spec.v_max / sp
        nxt = (r.pos[0] + vx * self.spec.dt, r.pos[1] + vy * self.spec.dt)
        if self.area.blocked(nxt):
            # slide along the obstacle, or stay
            for alt in ((nxt[0], r.pos[1]), (r.pos[0], nxt[1])):
                if not self.area.blocked(alt):
                    nxt = alt
                    break
            else:
                nxt = r.pos
        r.travelled += math.dist(r.pos, nxt)
        r.pos = nxt

    def behave(self, r):
        s = self.spec
        if self.planner and r.id in self.planner.member:
            # a robot working for a policy: its actions are driven by the
            # policy; between them it tracks its particle
            if r.status == 'member':
                p = self.particle(r.id)
                d = math.dist(r.pos, p)
                self.drive(r, p, s.v_max if d > 0.5 else min(s.v_max, s.v_particle))
            elif r.status == 'task' and r.job:
                q = self.site_point(r, r.job[1])
                if math.dist(r.pos, q) > 0.25 * s.r_task:
                    self.drive(r, q, s.v_max)
            return
        if r.status == 'explore':
            st = self.cb.status(r.id, self.t)
            if st in ('home', 'parked'):
                r.status = 'home'
            else:
                p = self.particle(r.id)
                d = math.dist(r.pos, p)
                self.drive(r, p, s.v_max if d > 0.5 else min(s.v_max, s.v_particle))
                return
        if r.status == 'home':
            q = self.home_point(r)
            if math.dist(r.pos, q) <= 0.6 * s.home_radius and math.dist(r.pos, q) <= max(0.3, 0.6 * s.home_radius
                                                                                             - s.slot):
                r.status = 'parked'
                self.log('home', robot=r.id)
                return
            self.drive(r, q, s.v_max)
            return
        if r.status == 'parked':
            return
        if r.status == 'seek':
            g = r.job[1]
            q = self.believed(r, g, self.t)
            if math.dist(r.pos, q) <= s.meet:
                # nobody in range at the place it was believed: absent
                self.log('absent', robot=r.id, other=g)
                self.counts['absent'] += 1
                r.k.excluded[g] = self.t
                if self.planner:
                    self.planner.absent(r, g)
                else:
                    comp = next(c for c in self.components() if r.id in c)
                    self.allocate(r, comp, reason='absent')
                    for x in comp:
                        self.adopt(self.robots[x])
                    if r.job and r.job[0] == 'seek' and r.job[1] == g:
                        self.next_job(r)
                return
            self.drive(r, q, s.v_max)
            return
        if r.status == 'task':
            tid = r.job[1] if r.job else None
            if tid is None or r.k.tasks[tid]['done']:
                self.next_job(r)
                return
            q = self.site_point(r, tid)
            if math.dist(r.pos, q) > 0.25 * s.r_task:
                self.drive(r, q, s.v_max)
                return
            r.wait_since = r.wait_since or self.t
            if self.planner is None and self.t - r.wait_since > WAIT_GRACE + self.expected_wait(r, tid):
                # the others have not come: allocate again, as if they were absent
                comp = next(c for c in self.components() if r.id in c)
                missing = [g for g, b in (r.k.plan or {}).get('bundles', {}).items()
                           if tid in b and g not in comp]
                for g in missing:
                    r.k.excluded[g] = self.t
                self.log('timeout', robot=r.id, task=tid, missing=missing)
                self.counts['timeout'] += 1
                r.wait_since = self.t
                r.plan_seen = None
                self.allocate(r, comp, reason='timeout')
                for x in comp:
                    self.adopt(self.robots[x])

    def expected_wait(self, r, tid):
        plan = r.k.plan
        if not plan:
            return 0.0
        spans = [sp for g in plan['expect'] for sp in plan['expect'][g]
                 if math.dist(sp[2], r.k.tasks[tid]['pos']) < 1e-6]
        return max(0.0, max((sp[0] + 5.0 for sp in spans), default=self.t) - self.t)

    def work_tasks(self):
        s = self.spec
        for tid, x in self.tasks.items():
            if x['done'] or x['found_t'] is None:
                continue
            here = [r for r in self.robots.values() if math.dist(r.pos, x['pos']) <= s.r_task
                    and tid in r.k.tasks and r.status in ('task', 'flock')]
            if len(here) >= x['k']:
                x.setdefault('since', self.t)
                if self.t - x['since'] >= x.get('work', s.work):
                    x['done'], x['done_t'] = True, self.t
                    self.log('done', task=tid, by=[r.id for r in here])
                    self.sync_due = True
                    for r in here:
                        r.k.tasks[tid]['done'] = True
                        if self.planner and r.id in self.planner.member:
                            continue
                        if r.job and r.job[0] == 'task' and r.job[1] == tid:
                            self.next_job(r)
            else:
                x.pop('since', None)

    # ─────────────────────────────────────────────────────────────── flock ──

    def flock_step(self):
        """The flock method: a formation of all the robots whose centre goes
        to the nearest point of the shared frontier, or to the nearest task
        known and not done, and home when the area has all been seen and no
        task is left."""
        s = self.spec
        if self.flock is None:
            self.flock = {'c': s.home, 'heading': math.pi / 2, 'goal': None, 'goal_t': -1.0,
                          'mover': nav.Mover(s), 'mode': 'explore'}
        f = self.flock
        rs = [self.robots[r] for r in self.ids]
        known = rs[0].known
        seen = rs[0].seen
        for r in rs[1:]:
            known = np.where(known == UNKNOWN, r.known, known)
            seen |= r.seen
        version = int(seen.sum())
        for r in rs:
            r.known = known.copy()
            r.seen = seen.copy()
            r.version = version
        undone = [tid for tid, x in self.tasks.items() if x['found_t'] is not None and not x['done']]
        blocked = known == OCC
        n = len(rs)
        spacing = 0.0 if n == 1 else min(0.9 * self.radio / (n - 1), 1.8 * s.sense)
        target_task = None
        if undone:
            target_task = min(undone, key=lambda tid: math.dist(f['c'], self.tasks[tid]['pos']))
            f['goal'] = self.tasks[target_task]['pos']
            f['mode'] = 'task'
        elif self.case == 1 and any(x['done'] for x in self.tasks.values()):
            f['goal'], f['mode'] = s.home, 'home'
        else:
            pts = frontier_points(seen, known, s.cell)
            # a frontier point the centre has stood on for a while that no
            # robot has seen from its slot is passed over
            if f['goal'] is not None and f['mode'] == 'explore' and math.dist(f['c'], f['goal']) < 1.0:
                f['held'] = f.get('held', 0.0) + s.dt
                if f['held'] > 3.0:
                    f.setdefault('skip', set()).add(self.area.cell_of(f['goal']))
                    f['goal'], f['held'] = None, 0.0
            else:
                f['held'] = 0.0
            if f.get('skip') and len(pts):
                keep = [i for i, q in enumerate(pts) if self.area.cell_of(q) not in f['skip']]
                pts = pts[keep]
            if not len(pts):
                f['goal'], f['mode'] = s.home, 'home'
            elif f['goal'] is None or f['mode'] != 'explore' or self.t - f['goal_t'] >= s.reselect \
                    or math.dist(f['c'], f['goal']) < 1.0:
                d = np.hypot(pts[:, 0] - f['c'][0], pts[:, 1] - f['c'][1])
                ang = np.arctan2(pts[:, 1] - f['c'][1], pts[:, 0] - f['c'][0])
                turn = np.abs(np.angle(np.exp(1j * (ang - f['heading']))))
                f['goal'] = tuple(pts[int(np.argmin(d + s.heading * turn))])
                f['goal_t'] = self.t
                f['mode'] = 'explore'
        # the centre moves on while the formation keeps up
        slots = self.slots(f, spacing, n)
        lag = max(math.dist(r.pos, q) for r, q in zip(rs, slots))
        speed = s.v_particle if lag < 2.0 else s.v_particle * 0.3
        if math.dist(f['c'], f['goal']) > 0.3:
            vx, vy = f['mover'].step(f['c'], f['goal'], blocked, version, speed)
            if abs(vx) + abs(vy) > 1e-9:
                f['heading'] = math.atan2(vy, vx)
            # the centre is a point of reference, not a robot: obstacles stop
            # the robots, which keep their slots around it as they can
            f['c'] = (f['c'][0] + vx * s.dt, f['c'][1] + vy * s.dt)
        slots = self.slots(f, spacing, n)
        if target_task is not None and math.dist(f['c'], f['goal']) < 3.0:
            x = self.tasks[target_task]
            near = sorted(rs, key=lambda r: math.dist(r.pos, x['pos']))[:x['k']]
            for r in near:
                slots[rs.index(r)] = self.site_point(r, target_task)
                r.status = 'flock'
                if target_task not in r.k.tasks:
                    r.k.tasks[target_task] = {'pos': x['pos'], 'k': x['k'], 'found_t': self.t,
                                              'found_by': r.id, 'done': False}
        for r, q in zip(rs, slots):
            if f['mode'] == 'home' and math.dist(r.pos, self.home_point(r)) <= max(0.3, 0.6 * s.home_radius - s.slot):
                if r.status != 'parked':
                    r.status = 'parked'
                    self.log('home', robot=r.id)
                continue
            r.status = 'flock' if f['mode'] != 'home' else 'home'
            self.drive(r, q, s.v_max)
        for tid in undone:
            for r in rs:
                r.k.tasks.setdefault(tid, {'pos': self.tasks[tid]['pos'], 'k': self.tasks[tid]['k'],
                                           'found_t': self.t, 'found_by': r.id, 'done': False})

    def slots(self, f, spacing, n):
        if f['mode'] == 'home':
            return [self.home_point(self.robots[r]) for r in self.ids]
        px, py = -math.sin(f['heading']), math.cos(f['heading'])
        out = []
        for i in range(n):
            o = (i - (n - 1) / 2) * spacing
            q = (f['c'][0] + o * px, f['c'][1] + o * py)
            q = (min(self.spec.width - 0.5, max(0.5, q[0])), min(self.spec.height - 0.5, max(0.5, q[1])))
            out.append(q)
        return out

    # ────────────────────────────────────────────────────────────── the end ──

    def over(self):
        if not all(r.status == 'parked' for r in self.robots.values()):
            return False
        if not all(x['done'] for x in self.tasks.values()):
            return False
        if self.case != 1:
            seen = np.zeros_like(self.robots[self.ids[0]].seen)
            known = self.robots[self.ids[0]].known
            for r in self.robots.values():
                seen |= r.seen
                known = np.where(known == UNKNOWN, r.known, known)
            pts = frontier_points(seen, known, self.spec.cell)
            if self.flock and self.flock.get('skip'):
                pts = [q for q in pts if self.area.cell_of(q) not in self.flock['skip']]
            if len(pts):
                return False
        return True

    def begin(self):
        if self.method != 'flock':
            self.share(set(self.ids))
            self.global_sync()
        self.steps = 0

    def step(self, t=None):
        """One tick: to time t, or one dt on. Returns whether the mission is
        over."""
        s = self.spec
        self.t = round(self.t + s.dt, 6) if t is None else t
        self.steps += 1
        step = self.steps
        frame_every = int(round(0.5 / s.dt))
        if True:
            if self.method == 'flock':
                self.flock_step()
            else:
                self.communicate()
                if self.planner:
                    self.planner.step()
                if step % int(round(5.0 / s.dt)) == 0:
                    for r in self.robots.values():
                        if self.planner is None:
                            self.retry(r)
                for r in self.robots.values():
                    self.behave(r)
            for r in self.robots.values():
                c = self.area.cell_of(r.pos)
                if c != r.cell:
                    r.cell = c
                    self.area.sense(r.pos, r.known, r.seen)
                    r.version += 1 if (r.known[c] == OCC) else 0
                self.discover(r)
            # the map a robot plans on changes when it sees an obstacle
            for r in self.robots.values():
                r.version = int((r.known == OCC).sum())
            self.work_tasks()
            if self.record and step % frame_every == 0:
                self.frames.append(self.frame())
        return self.over()

    def run(self):
        self.begin()
        while self.t < self.spec.limit:
            if self.step():
                break
        return self.result()

    def frame(self):
        return {'t': round(self.t, 2),
                'robots': {r.id: [round(r.pos[0], 2), round(r.pos[1], 2), r.status,
                                  r.job[1] if r.job and r.job[0] == 'seek' else None]
                           for r in self.robots.values()},
                'particles': ({r: [round(v, 2) for v in self.cb.pos(r, self.t)] for r in self.ids}
                              if self.cb else None),
                'flock': list(self.flock['c']) if self.flock else None}

    def result(self):
        ended = self.over()
        return {'method': self.method, 'case': self.case, 'n': len(self.ids),
                'mission': round(self.t, 2) if ended else None, 'ended': ended,
                'tasks': {tid: {'k': x['k'], 'pos': x['pos'], 'found_t': x['found_t'], 'found_by': x['found_by'],
                                'done_t': x['done_t']} for tid, x in self.tasks.items()},
                'counts': dict(self.counts),
                'travelled': {r.id: round(r.travelled, 1) for r in self.robots.values()},
                'planning': self.planner.stats if self.planner else None,
                'events': self.events}


def k_copy(k, ids):
    c = Knowledge(ids)
    c.merge(k, 0.0)
    c.assumed = dict(k.assumed)
    return c


def frontier_points(seen, known, cell):
    s = seen & (known != OCC)
    unseen = ~seen
    nb = np.zeros_like(s)
    nb[1:, :] |= unseen[:-1, :]
    nb[:-1, :] |= unseen[1:, :]
    nb[:, 1:] |= unseen[:, :-1]
    nb[:, :-1] |= unseen[:, 1:]
    return (np.argwhere(s & nb) + 0.5) * cell


def run(area, n, case, method, seed, record=False):
    sim = Sim(area, n, case, method, seed, record=record)
    out = sim.run()
    if record:
        out['frames'] = sim.frames
    return out
