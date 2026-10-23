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
The epistemic method of the kinematic simulator: the decision a robot makes
when it finds a task is Aletheia's policy for the EPDDL task of epddl.py, run
as ePlanSys runs it (schedule.py).

A robot that finds a task writes the planning task from what it knows: the
robots in contact with it know of the task and are free; every other robot is
where the common belief puts it, or, in Case II, may be busy on other work;
who may call whom is the neighbour relation of the common belief's partition,
eq. (9), over where the finder believes each robot to be. Aletheia plans with
its replanning strategy (AO* does not reach the depths of the larger teams),
and the policy is scheduled and dispatched as ePlanSys's executor does: a
group of independent actions starts together and the next waits for all of it.

What each action does here:

  call i j     i goes to where it believes j to be. When j is within radio
               range the call is heard: met if j is free (not working for
               another policy and not on a task), absent otherwise; when i
               reaches the place with nobody in range, absent.
  await i j    i holds at j's particle until j is in range and free.
  leave i j    nothing to do.
  go-site r    r sets out for the task's site; the action is answered then,
               and r goes on to the site and waits there.
  do-task r    r goes to the site and waits there until the task is done.

A robot that has heard of the task belongs to the policy until the policy
has no more actions for it; meanwhile, between actions, it tracks its
particle, so that others still find it where they believe it to be. When it is
released it goes home in Case I and back to its particle in Case II. A task
another robot finds while it belongs to a policy waits until it is released.
If the planner returns no policy the robot allocates by the paper's
algorithms instead, which is counted.
"""

import math

import numpy as np

from . import alloc
from . import epddl as P
from . import schedule as SCH

# declared durations, as in the executor's mapping: an arrival is answered when the robot sets out
DURATION = {'call': 10.0, 'await': 20.0, 'leave': 0.5, 'go-site': 0.5, 'do-task': 15.0}
PATIENCE = 45.0      # s a robot waits at a busy teammate's particle before it falls back to the paper's rule


def kind(action):
    return action.split('_')[0].rsplit('-', 1)[0] if action.startswith(('go-site', 'do-task')) else \
        action.split('_')[0]


def args(action):
    return action.split('_')[1:]


def requires(action):
    """The knowledge requirements of a grounded action of the domain, as
    ePlanSys reads them off the preconditions."""
    k, a = kind(action), args(action)
    if k == 'call':
        return [f'(K {a[0]} task)']
    if k == 'await':
        return [f'(K {a[0]} busy_{a[1]})']
    if k in ('go-site', 'do-task'):
        return [f'(K {a[0]} task)']
    return []


def near_relation(pos):
    """Pairs whose cells in the Voronoi partition of the given points share
    a border: the Delaunay edges (all pairs for three points or fewer)."""
    ids = list(pos)
    if len(ids) <= 3:
        return [(a, b) for a in ids for b in ids if a != b]
    from scipy.spatial import Delaunay
    pts = np.array([pos[i] for i in ids], dtype=float)
    pts = pts + 1e-6 * np.arange(len(ids))[:, None]
    try:
        tri = Delaunay(pts)
    except Exception:
        return [(a, b) for a in ids for b in ids if a != b]
    edges = set()
    for s in tri.simplices:
        for x in s:
            for y in s:
                if x != y:
                    edges.add((ids[x], ids[y]))
    return sorted(edges)


class Run:
    """One policy being carried out."""

    def __init__(self, planner, finder, tid, inst, root, items, task):
        self.p, self.sim = planner, planner.sim
        self.finder, self.tid, self.inst = finder, tid, inst
        self.cur = root
        self.items = items
        self.task = task
        self.members = set(inst.knowers)
        self.agents = set(inst.agents)
        self.active = None          # [(node, item, state)] of the group being run
        self.ended = False
        self.started = self.sim.t

    def outcome_index(self, item, event):
        """The branch of `item` that the event chose."""
        node = self.task['actions'][item.action]
        names = node['designated']
        for i, b in enumerate(self.p.branches[(id(self), item.index)]):
            e = b
            name = names[e] if isinstance(e, int) else e
            if name.startswith(event):
                return i
        return 0

    def remaining(self, node, seen=None):
        """The agents named by any action at or below node."""
        out = set()
        stack = [node]
        seen = set()
        while stack:
            n = stack.pop()
            if n is None or id(n) in seen:
                continue
            seen.add(id(n))
            out |= self.items[n.item].agents
            stack.extend(n.next)
        return out

    def step(self):
        if self.ended:
            return
        if self.active is None:
            if self.cur is None:
                self.end()
                return
            group, n = [], self.cur
            for _ in range(self.cur.group):
                group.append(n)
                n = n.next[0] if n.next else None
            self.active = [[node, self.items[node.item], {'done': False, 'event': None, 't0': self.sim.t}]
                           for node in group]
            for node, it, st in self.active:
                self.dispatch(it, st)
            self.release_idle()
        for node, it, st in self.active:
            if self.ended:
                return
            if not st['done']:
                self.advance(it, st)
        if self.ended:
            return
        if all(st['done'] for _, _, st in self.active):
            at = self.cur
            for node, it, st in self.active:
                if not at.next:
                    at = None
                    break
                i = self.outcome_index(it, st['event']) if len(at.next) > 1 else 0
                at = at.next[i]
            self.cur = at
            self.active = None

    # ─────────────────────────────────────────────────────────── actions ──

    def dispatch(self, it, st):
        k, a = kind(it.action), args(it.action)
        s = self.sim
        self.p.log_action(self, it.action)
        if k == 'call' or k == 'await':
            i = s.robots[a[0]]
            i.pjob = (k, a[1], st)
            i.status = 'policy'
        elif k == 'leave':
            st['done'], st['event'] = True, 'e-leave'
        elif k == 'go-site':
            # answered when the robot sets out: it goes on to the site by
            # itself, and the arrival that does the task waits for it there
            r = s.robots[a[0]]
            r.status, r.job = 'task', ('task', self.tid)
            st['done'], st['event'] = True, f"e-site-{it.action.split('_')[0].rsplit('-', 1)[1]}"
        elif k == 'do-task':
            r = s.robots[a[0]]
            r.pjob = (k, self.tid, st)
            r.status = 'policy'

    def advance(self, it, st):
        k, a = kind(it.action), args(it.action)
        s = self.sim
        spec = s.spec
        if k in ('call', 'await'):
            i, j = s.robots[a[0]], s.robots[a[1]]
            q = s.believed(i, j.id, s.t)
            in_range = math.dist(i.pos, j.pos) <= s.radio
            free = self.p.free(j) or self.p.member.get(j.id) is self
            if k == 'call':
                other = self.p.member.get(j.id)
                if in_range and not free and other is not None and other is not self:
                    # two policies in contact: the robots share what they know,
                    # and the task found first keeps its robots
                    mine = (s.tasks[self.tid]['found_t'], self.started)
                    theirs = (s.tasks[other.tid]['found_t'], other.started)
                    s.counts['merged'] += 1
                    if (mine, self.tid) <= (theirs, other.tid):
                        self.p.suspend(other, f'met {self.tid}, found earlier')
                        self.finish(i, st, 'e-met')
                        self.join(j, i)
                    else:
                        self.p.suspend(self, f'met {other.tid}, found earlier')
                        other.join(i, j)
                    return
                if in_range:
                    self.finish(i, st, 'e-met' if free else 'e-absent')
                    if free:
                        self.join(j, i)
                    else:
                        s.log('absent', robot=i.id, other=j.id, policy=True)
                        s.counts['absent'] += 1
                    return
                if math.dist(i.pos, q) <= spec.meet:
                    self.finish(i, st, 'e-absent')
                    s.log('absent', robot=i.id, other=j.id, policy=True)
                    s.counts['absent'] += 1
                    return
                s.drive(i, q, spec.v_max)
            else:
                if in_range and free:
                    self.finish(i, st, 'e-await')
                    self.join(j, i)
                    s.counts['awaited'] += 1
                    return
                if s.t - st['t0'] > PATIENCE:
                    # the teammate has not come back: its work may itself be
                    # waiting on this robot (a cycle the model's assumption
                    # rules out); fall back to the paper's exclusion
                    self.p.fail(self, 'patience')
                    return
                if math.dist(i.pos, q) > 0.5:
                    s.drive(i, q, spec.v_max)
        elif k == 'do-task':
            r = s.robots[a[0]]
            x = s.tasks[self.tid]
            q = s.site_point(r, self.tid)
            if math.dist(r.pos, q) > 0.25 * spec.r_task:
                s.drive(r, q, spec.v_max)
                return
            r.status = 'task'
            r.job = ('task', self.tid)
            if x['done']:
                self.finish(r, st, f"e-last-{int(it.action.split('_')[0].rsplit('-', 1)[1]) - 1}")

    def finish(self, r, st, event):
        st['done'], st['event'] = True, event
        if r.pjob is not None and r.pjob[2] is st:
            r.pjob = None
            if r.status == 'policy':
                r.status = 'member'

    def join(self, j, by):
        s = self.sim
        if self.p.member.get(j.id) is self:
            return
        self.members.add(j.id)
        self.p.member[j.id] = self
        x = s.tasks[self.tid]
        j.k.tasks.setdefault(self.tid, {'pos': x['pos'], 'k': x['k'], 'found_t': x['found_t'],
                                        'found_by': x['found_by'], 'done': x['done']})
        for g in (by, j):
            if self.tid in g.k.tasks:
                g.k.tasks[self.tid]['knowers'] = set(g.k.tasks[self.tid].get('knowers', ())) | {by.id, j.id}
        # told of the task, a robot is told of the policy too: it knows the
        # task is handled, by whom, and what is left for it to do
        j.k.tasks[self.tid]['handled'] = True
        if j.status in ('explore', 'home', 'parked'):
            j.status = 'member'
        s.log('met', robot=by.id, other=j.id, policy=True)
        s.counts['met'] += 1

    def release_idle(self):
        """Members the rest of the policy has no action for are released."""
        if self.cur is None:
            return
        left = self.remaining(self.cur)
        done = self.sim.tasks[self.tid]['done']
        for m in list(self.members):
            if m not in left and self.p.member.get(m) is self:
                # a robot that has gone to the site stays there until the task is done
                if self.sim.robots[m].status == 'task' and not done:
                    continue
                self.p.release(m, self)

    def end(self):
        self.ended = True
        for m in list(self.members):
            if self.p.member.get(m) is self:
                self.p.release(m, self)
        self.p.runs.remove(self)
        self.sim.log('policy-end', task=self.tid, finder=self.finder, seconds=round(self.sim.t - self.started, 1))


class Planner:

    def __init__(self, sim):
        self.sim = sim
        self.runs = []
        self.member = {}          # robot -> Run
        self.pending = {}         # robot -> [tid]
        self.branches = {}
        self.stats = []
        self.given_up = set()
        self.retry_at = {}        # task -> (time, robot): a policy failed; plan again then
        self.count = 0

    # ─────────────────────────────────────────────────────────── queries ──

    def free(self, j):
        """j works for no policy and on no task: it is at its particle, on its
        way home, or home."""
        if j.id in self.member:
            return False
        return j.status in ('explore', 'home', 'parked') and (j.job is None or j.job[0] == 'home')

    def handling(self, tid, comp=None):
        """A policy for the task that a robot in `comp` belongs to, which the
        robots of comp therefore know of; any policy for it when comp is None."""
        return any(r.tid == tid and (comp is None or any(self.member.get(g) is r for g in comp))
                   for r in self.runs)

    # ──────────────────────────────────────────────────────────── events ──

    def found(self, r, tid, comp):
        if self.handling(tid, comp) or self.sim.tasks[tid]['done'] or tid in self.retry_at:
            return
        if r.id in self.member or r.status in ('task', 'policy', 'member'):
            self.pending.setdefault(r.id, []).append(tid)
            return
        self.start(r, tid, comp)

    def contact(self, comp):
        """Robots in contact share what they know (the paper's local update):
        a free robot in contact with a member of a policy learns its task and
        joins it; a task known to the group that no policy handles is planned
        for by the first free robot that knows it."""
        s = self.sim
        if len(comp) == len(s.ids):
            return          # all in contact: the global sync allocates
        for run in list(self.runs):
            if any(self.member.get(g) is run for g in comp):
                for g in comp:
                    if self.free(s.robots[g]) and run.tid not in s.robots[g].k.tasks:
                        run.join(s.robots[g], s.robots[next(m for m in comp if self.member.get(m) is run)])
        for g in sorted(comp):
            r = s.robots[g]
            if not self.free(r):
                continue
            planned = set()
            if r.k.plan:
                for b in r.k.plan['bundles'].values():
                    planned |= set(b)
            for tid, x in r.k.tasks.items():
                if x['done'] or s.tasks[tid]['done'] or self.handling(tid, comp) or tid in r.k.assumed \
                        or tid in planned or tid in self.retry_at or x.get('handled'):
                    continue
                self.start(r, tid, comp)
                break

    def met(self, r, g):
        pass

    def absent(self, r, g):
        pass

    def done(self, r, tid):
        pass

    def instance(self, r, tid, comp):
        """The planning task of robot r, which has found task tid, from what it
        knows; comp is who is in contact with it."""
        s = self.sim
        x = s.tasks[tid]
        comp = [g for g in comp if not (g in self.member) and s.robots[g].status in ('explore', 'home', 'parked')
                or g == r.id]
        k = x['k']
        # in Case I a robot told of the task with nothing to do goes home: the
        # finder leaves out those it knows to have been told and that are out
        # of contact, unless the task needs them
        skip = set()
        if s.case == 1:
            told = set(r.k.tasks[tid].get('knowers', ())) - set(comp)
            if len(s.ids) - len(told) >= k:
                skip = told
        travel = alloc.Travel(s.area, r.known, s.spec.v_max)
        rank = {}
        for g in s.ids:
            if g == r.id or g in skip:
                continue
            if g in comp:
                rank[g] = travel.time(s.robots[g].pos, x['pos'])
            else:
                t, q = alloc.intercept(travel, r.pos, s.t, lambda u, g=g: s.believed(r, g, u),
                                       reach=0.8 * s.radio)
                rank[g] = (t - s.t) + travel.time(q, x['pos'])
        others = sorted(rank, key=rank.get)
        if s.case == 1:
            chosen = others
        else:
            # the robots in contact, then the best candidates, two spare
            want = max(0, k - 1 - (len(comp) - 1)) + 2
            out = [g for g in others if g not in comp]
            chosen = [g for g in others if g in comp] + out[:want]
        agents = [r.id] + chosen
        knowers = [g for g in agents if g in comp]
        may_be_busy = [] if s.case == 1 else [g for g in agents if g not in comp]
        max_busy = min(len(may_be_busy), 2)
        pos = {}
        for g in agents:
            pos[g] = s.robots[g].pos if g in comp else s.believed(r, g, s.t)
        near = near_relation(pos)
        self.count += 1
        return P.Instance(agents, r.id, k, 1 if s.case == 1 else 2, knowers=knowers, may_be_busy=may_be_busy,
                          max_busy=max_busy, near=near, name=f'gossip-{tid}-{r.id}-{self.count}')

    def enlist(self, run):
        """The robots that know of the task when the policy starts belong to it."""
        s = self.sim
        x = s.tasks[run.tid]
        self.runs.append(run)
        for g in run.inst.knowers:
            self.member[g] = run
            rg = s.robots[g]
            rg.k.tasks.setdefault(run.tid, {'pos': x['pos'], 'k': x['k'], 'found_t': x['found_t'],
                                            'found_by': x['found_by'], 'done': False})
            rg.k.tasks[run.tid]['handled'] = True
            rg.jobs, rg.job = [], None
            rg.status = 'member'

    def start(self, r, tid, comp):
        s = self.sim
        inst = self.instance(r, tid, comp)
        agents, knowers, may_be_busy = inst.agents, sorted(inst.knowers), inst.may_be_busy
        try:
            policy, seconds, log, task_path = P.solve(inst, timeout=30.0,
                                                      extra=['--strategy', 'replan', '--subjective', 'auto'])
        except Exception as e:     # noqa: BLE001
            policy, seconds, log, task_path = None, 0.0, str(e), None
        self.stats.append({'t': round(s.t, 2), 'robot': r.id, 'task': tid, 'agents': len(agents),
                           'seconds': round(seconds, 3), 'solved': policy is not None})
        if policy is None:
            s.log('no-policy', robot=r.id, task=tid, seconds=round(seconds, 2))
            s.counts['fallback'] += 1
            s.allocate(r, list(inst.knowers))
            for g in inst.knowers:
                s.adopt(s.robots[g])
            return
        import json
        task = json.load(open(task_path))
        root, items = SCH.schedule(policy, task, lambda a: DURATION.get(kind(a), 10.0), requires)
        run = Run(self, r.id, tid, inst, root, items, task)
        # the event each branch of each item is chosen by, in preorder
        idx = [0]

        def walk(node):
            if node is None:
                return
            i = idx[0]
            idx[0] += 1
            self.branches[(id(run), i)] = [b['event'] for b in node['branches']]
            for b in node['branches']:
                walk(b['subtree'])
        walk(policy)
        self.enlist(run)
        s.log('policy', robot=r.id, task=tid, agents=agents, knowers=knowers, may_be_busy=may_be_busy,
              seconds=round(seconds, 2))
        s.counts['policies'] += 1

    def log_action(self, run, action):
        self.sim.log('action', task=run.tid, action=action)

    def release(self, g, run):
        s = self.sim
        r = s.robots[g]
        self.member.pop(g, None)
        r.pjob = None
        r.job, r.jobs = None, []
        r.mover.reset()
        if s.case == 1:
            r.status = 'home' if r.status != 'parked' else 'parked'
        else:
            r.status = 'explore'
        for tid in self.pending.pop(g, []):
            if not s.tasks[tid]['done'] and not self.handling(tid):
                comp = next(c for c in s.components() if g in c)
                self.found(r, tid, comp)

    def suspend(self, run, why):
        """A policy whose task gives way to an older one: its members go back
        to their particles, and its finder plans again later."""
        self.sim.counts['suspended'] += 1
        self.fail(run, why)
        self.sim.counts['failed'] -= 1

    def fail(self, run, why, fallback=None):
        s = self.sim
        s.log('policy-failed', task=run.tid, why=why)
        s.counts['failed'] += 1
        run.ended = True
        for m in list(run.members):
            if self.member.get(m) is run:
                self.release(m, run)
        if run in self.runs:
            self.runs.remove(run)
        # every member is back at its particle, where the others believe it to
        # be; the finder plans again after a back-off of its own
        self.retry_at[run.tid] = (s.t + 5.0 + 20.0 * s.rng.random(), run.finder)

    def cancel_all(self):
        for run in list(self.runs):
            run.ended = True
            for m in list(run.members):
                if self.member.get(m) is run:
                    self.member.pop(m, None)
                    self.sim.robots[m].pjob = None
            self.runs.remove(run)
        self.pending.clear()

    def step(self):
        s = self.sim
        for tid, (t, g) in list(self.retry_at.items()):
            if s.tasks[tid]['done']:
                del self.retry_at[tid]
            elif s.t >= t and self.free(s.robots[g]):
                del self.retry_at[tid]
                comp = next(c for c in s.components() if g in c)
                if len(comp) < len(s.ids):
                    self.start(s.robots[g], tid, comp)
        for run in list(self.runs):
            run.step()
            if not run.ended and self.sim.tasks[run.tid]['done']:
                for m in list(run.members):
                    r = self.sim.robots[m]
                    if self.member.get(m) is run and r.status == 'task' and m not in run.remaining(run.cur) \
                            and not (run.active and any(m in it.agents for _, it, st in run.active
                                                        if not st['done'])):
                        self.release(m, run)
