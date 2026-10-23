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
The common belief set C of Sections 5.1 and 5.2.

When every robot is in contact they share their states, maps and tasks, and
from that moment each of them propagates, alike, one particle per robot: the
common belief. A particle covers by the epistemic frontier method of Section
5.2. The frontier is that of the shared map together with what every particle
has seen since; each frontier point belongs to the Voronoi cell of the nearest
particle, eq. (9); a particle's utility for a point is its distance, a term for
the turn, and the penalty Delta when the point is in another particle's cell,
eq. (10); it goes to the point of least utility, eqs. (11) and (12). A particle
sees with its robot's sensor over the shared map, through the obstacles nobody
has seen yet, and moves by the planner and field of nav.py, at the particle
speed. When the frontier is empty the particles go home.

A particle with work in the shared plan, a task it was assigned when the plan
was shared, does it first: it goes to the task, waits there for the particles
of the other robots assigned to it, and goes on to explore, or home in Case I.

Every robot computes the same particles from the same shared state, so one
CommonBelief object stands for every robot's copy. A robot out of contact
tracks its own particle (its empathy state): it stays where the others believe
it to be. Nothing a robot learns out of contact changes the common belief; it
is reset at the next moment all are in contact again (eq. (14)).
"""

import math

import numpy as np

from . import nav
from .env import FREE, OCC, UNKNOWN


class Particle:

    def __init__(self, rid, pos, prefix, after, spec):
        self.id = rid
        self.pos = tuple(pos)
        self.prefix = list(prefix)      # [('task', tid, pos, cohort)] or [('home',)]
        self.after = after              # 'explore' or 'home'
        self.status = 'prefix' if prefix else after
        self.goal = None
        self.goal_t = -math.inf
        self.heading = math.pi / 2
        self.arrived = None             # when it reached the task it waits at
        self.mover = nav.Mover(spec)
        self.cell = None
        self.stuck = 0


class CommonBelief:

    def __init__(self, area, t0, starts, known, seen, prefix=None, after='explore', tasks_done=(), work=None,
                 previous=None):
        """starts: {robot: (x, y)}; known, seen: the shared map and coverage;
        prefix: {robot: [steps]} the work of the shared plan; after: what a
        particle does once its prefix is done ('explore' or 'home')."""
        self.area, self.spec = area, area.spec
        self.t0 = t0
        self.t = t0
        self.ids = sorted(starts)
        self.index = {r: i for i, r in enumerate(self.ids)}
        self.cmap = known.copy()
        self.blocked = self.cmap == OCC
        self.bseen = seen.copy() | (self.cmap == OCC)
        self.first = np.full((len(self.ids),) + known.shape, np.inf, dtype=np.float32)
        prefix = prefix or {}
        self.p = {r: Particle(r, starts[r], prefix.get(r, []), after, self.spec) for r in self.ids}
        if previous is not None:
            # a reset keeps where each particle was heading and for what: the
            # common belief starts again from the robots, not from scratch
            for r in self.ids:
                old = previous.p.get(r)
                if old is not None:
                    self.p[r].heading = old.heading
                    if old.status == 'explore' and self.p[r].status == 'explore':
                        self.p[r].goal, self.p[r].goal_t = old.goal, old.goal_t
        self.traj = [np.array([self.p[r].pos for r in self.ids])]
        self.stat = [[self.p[r].status for r in self.ids]]
        self.done_at = None
        self.work = dict(work or {})       # tid -> how long the task takes
        self.together = {}                 # tid -> when its cohort was first all there
        self.finished = set()              # tasks done in the belief
        self.frontier_cache = (None, None)
        self.gain_cache = None
        # the unseen cells at the edge of what the robots have really seen:
        # a particle counts one as seen only from beside it, so that the
        # belief cannot call covered, through an obstacle nobody has seen, a
        # place the team's own map leaves open
        edge = self._frontier()
        self.close = np.zeros_like(self.bseen)
        if len(edge):
            idx = (edge / self.spec.cell).astype(int)
            near = np.zeros_like(self.bseen)
            near[idx[:, 0], idx[:, 1]] = True
            grown = nav.dilate(near, 1)
            self.close = grown & ~self.bseen
        self.frontier_cache = (None, None)
        for r in self.ids:
            self._sense(self.p[r])

    # ────────────────────────────────────────────────────────────── queries ──

    def k(self, t):
        return max(0, int(round((t - self.t0) / self.spec.dt)))

    def advance(self, t):
        while self.t < t - 1e-9:
            self._step()

    def pos(self, r, t):
        self.advance(t)
        return tuple(self.traj[min(self.k(t), len(self.traj) - 1)][self.index[r]])

    def status(self, r, t):
        self.advance(t)
        return self.stat[min(self.k(t), len(self.stat) - 1)][self.index[r]]

    def first_seen(self, r, cell):
        """When r's particle first saw the cell, inf if it has not (as far as
        the belief has been propagated)."""
        return float(self.first[self.index[r]][cell])

    def frontier(self):
        return self._frontier()

    # ─────────────────────────────────────────────────────────── propagation ──

    def _sense(self, q):
        c = self.area.cell_of(q.pos)
        if c == q.cell:
            return
        q.cell = c
        cells = self.area.vis.visible(c[0], c[1], self.blocked)
        if self.close.any():
            far = (np.abs(cells[0] - c[0]) > 1) | (np.abs(cells[1] - c[1]) > 1)
            keep = ~(self.close[cells] & far)
            cells = (cells[0][keep], cells[1][keep])
            self.close[cells] = False
        fresh = ~self.bseen[cells]
        self.bseen[cells] = True
        i = self.index[q.id]
        sub = self.first[i][cells]
        self.first[i][cells] = np.minimum(sub, self.t)
        if fresh.any():
            self.frontier_cache = (None, None)
            self.gain_cache = None

    def _frontier(self):
        if self.frontier_cache[0] is not None:
            return self.frontier_cache[1]
        s = self.bseen & ~self.blocked
        unseen = ~self.bseen
        nb = np.zeros_like(s)
        nb[1:, :] |= unseen[:-1, :]
        nb[:-1, :] |= unseen[1:, :]
        nb[:, 1:] |= unseen[:, :-1]
        nb[:, :-1] |= unseen[:, 1:]
        f = np.argwhere(s & nb)
        pts = (f + 0.5) * self.spec.cell
        self.frontier_cache = (True, pts)
        return pts

    def _choose(self, explorers):
        """Eqs. (9) to (12) for every exploring particle whose goal is due."""
        pts = self._frontier()
        if not len(pts):
            return False
        cs = np.array([self.p[r].pos for r in explorers])
        gain = self._gain(pts)
        d = np.hypot(pts[:, None, 0] - cs[None, :, 0], pts[:, None, 1] - cs[None, :, 1])
        owner = np.argmin(d, axis=1)
        for j, r in enumerate(explorers):
            q = self.p[r]
            if q.goal is not None and self.t - q.goal_t < self.spec.reselect and math.dist(q.pos, q.goal) > 0.5 * self.spec.cell:
                continue
            ang = np.arctan2(pts[:, 1] - q.pos[1], pts[:, 0] - q.pos[0])
            turn = np.abs(np.angle(np.exp(1j * (ang - q.heading))))
            u = d[:, j] + self.spec.heading * turn + self.spec.penalty * (owner != j) - self.spec.gain * gain
            z = int(np.argmin(u))
            new = tuple(pts[z])
            if q.goal is None or math.dist(new, q.goal) > 0.1:
                q.mover.reset()
            q.goal, q.goal_t = new, self.t
        return True

    def _gain(self, pts):
        """For every frontier point, the unseen cells within sensor range of
        it: what a particle there would see, ignoring occlusion."""
        if not self.spec.gain:
            return 0.0
        if self.gain_cache is None:
            r = int(round(self.spec.sense / self.spec.cell))
            self.gain_cache = nav.disk_sum(~self.bseen, r)
        idx = (pts / self.spec.cell).astype(int)
        return self.gain_cache[idx[:, 0], idx[:, 1]] * self.spec.cell ** 2

    def _move(self, q, goal, speed):
        vx, vy = q.mover.step(q.pos, goal, self.blocked, q.stuck, speed)
        if abs(vx) + abs(vy) > 1e-9:
            q.heading = math.atan2(vy, vx)
        nxt = (q.pos[0] + vx * self.spec.dt, q.pos[1] + vy * self.spec.dt)
        if not self.free(nxt):
            # a step that clips the corner of a known obstacle: slide along it,
            # and plan afresh if that does not help either
            for alt in ((nxt[0], q.pos[1]), (q.pos[0], nxt[1])):
                if self.free(alt):
                    nxt = alt
                    break
            else:
                nxt = q.pos
                q.stuck += 1
                q.mover.reset()
        q.pos = nxt

    def free(self, p):
        return self.area.inside(p) and not self.blocked[self.area.cell_of(p)]

    def _step(self):
        s = self.spec
        self.t += s.dt
        explorers = [r for r in self.ids if self.p[r].status == 'explore']
        if explorers and not self._choose(explorers):
            for r in explorers:
                self.p[r].status = 'home'
        for r in self.ids:
            q = self.p[r]
            if q.status == 'prefix':
                step = q.prefix[0]
                if step[0] == 'home':
                    q.status = 'home'
                    q.prefix.pop(0)
                else:
                    _, tid, tpos, cohort = step
                    work = self.work.get(tid, s.work)
                    if tid in self.finished:
                        q.prefix.pop(0)
                        q.arrived = None
                        if not q.prefix:
                            q.status = q.after
                    elif math.dist(q.pos, tpos) > s.r_task * 0.6:
                        self._move(q, tpos, s.v_particle)
                    else:
                        q.arrived = q.arrived or self.t
                        here = all(math.dist(self.p[c].pos, tpos) <= s.r_task for c in cohort if c in self.p)
                        if here:
                            self.together.setdefault(tid, self.t)
                        if here and self.t - self.together[tid] >= work:
                            # the task is done in the belief: every particle of
                            # its cohort goes on
                            self.finished.add(tid)
            elif q.status == 'explore':
                if q.goal is not None:
                    self._move(q, q.goal, s.v_particle)
            elif q.status == 'home':
                if math.dist(q.pos, s.home) > 0.5 * s.home_radius:
                    self._move(q, s.home, s.v_particle)
                else:
                    q.status = 'parked'
            self._sense(q)
        self.traj.append(np.array([self.p[r].pos for r in self.ids]))
        self.stat.append([self.p[r].status for r in self.ids])
        if self.done_at is None and all(self.p[r].status == 'parked' for r in self.ids):
            self.done_at = self.t

    def _last_arrival(self, cohort, tpos):
        return max((self.p[c].arrived or self.t) for c in cohort if c in self.p)
