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
One run of the belief and empathy demonstration.

    mission.py --dir RUN --formulation epistemic|particles|ideal|flock|timed|published|strong

Every formulation starts at the same simulated instant, worlds.START_AT, with
the three robots at the base, and ends when the task is done, or at --limit.
The robots cover by themselves until the task is found; what happens then is
the formulation's. Every formulation drives the robots through the same
requests (/<robot>/request, /<robot>/reply), so the controllers are the same.

epistemic  when the task is found, Aletheia plans over belief-empathy.epddl
           through PlanSys2, and ePlanSys's epistemic executor runs the policy;
           what each action did is the event its node reports.
particles  the aerial vehicle, out of contact, does as Bramblett and Bezzo's
           procedure does. It estimates for each ground vehicle the makespan of
           seeking it at its most likely remaining particle (the common belief
           first) and of both then reaching the site, and seeks the least. Found
           absent, it rules the particle out and estimates again; met, it learns
           the vehicle's state and estimates again with it: it assigns the task
           to the vehicle it met, or sends it on (to the base if it can no
           longer do the task) and seeks the next.
ideal      every robot hears every other: when the task is found the aerial
           vehicle knows each vehicle's place and faults, assigns the one of
           least makespan by radio, and both go.
flock      the mission keeps the flock's reference, /flock/s, moving at the
           speed of the slowest robot still in the flock, until the task is
           found; then, all in contact, it assigns as the ideal method does.

In the variant with latent tool faults (RUN/world.json says which), ideal,
published and strong are the paper's procedure at three strengths, deciding by
tools/estimate.Procedure: published, each vehicle's particles alone and a
vehicle whose attempt fails dropped; strong, every fault pattern consistent with
what the aerial vehicle has learned, the team's budget and the assumption that
some tool works, and the assignment and order of visits of least mean makespan
over them; ideal, the same knowing every vehicle's drive faults. timed runs,
through the same planner plugin and executor as epistemic, a policy computed
beforehand by a timed AND-OR search over the same EPDDL. The task is attempted
with one named vehicle at a time.

The makespan estimate is the paper's: the time to the meeting, if one is
needed, then the longer of the two journeys to the site. The paper searches
the assignments with a genetic algorithm; with one task and two vehicles there
are two, and they are compared directly.

Writes RUN/mission.json and exits; the launch shuts down when it has.
"""

import argparse
import json
import math
import os
import shutil
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetDomain, GetPlan, GetProblem
from plansys2_epistemic_msgs.srv import CheckFormula
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('belief_empathy_demo'), 'tools'))
import estimate as E  # noqa: E402
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
import worlds as W  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HAIL_EST = 2.0        # s the makespan estimate allows a hail that is answered
FORMULAS = ('done', '(K g1 task)', '(K g2 task)', '(K u p1_g1)', '(K u p2_g1)', '(K u p3_g1)',
            '(K u p1_g2)', '(K u p2_g2)', '(K u p3_g2)')
LATENT_FORMULAS = FORMULAS + ('(Kw u tool_g1)', '(Kw u tool_g2)')


def drive_time(p, goal, faults):
    return L.length(L.ground_path(p, goal)) / L.DRIVE[min(faults, 2)]


def fly_time(p, q):
    return math.dist(p, q) / L.V_UAV


class Mission(Node):

    def __init__(self, a):
        super().__init__('be_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.latent = json.load(open(os.path.join(a.dir, 'world.json'))).get('variant') == 'latent'
        self.pose, self.state = {}, {}
        self.found, self.done = None, None
        for r in L.ROBOTS:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.pose.__setitem__(
                r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        for g in L.UGVS:
            self.create_subscription(String, f'/{g}/state', lambda m, g=g: self.state.__setitem__(
                g, json.loads(m.data)), LATCHED)
        self.create_subscription(String, '/task/found', lambda m: setattr(self, 'found', json.loads(m.data)),
                                 LATCHED)
        self.create_subscription(String, '/task/done', lambda m: setattr(self, 'done', json.loads(m.data)),
                                 LATCHED)
        self.replies = {}
        self.requests = {r: self.create_publisher(String, f'/{r}/request', 10) for r in L.ROBOTS}
        for r in L.ROBOTS:
            self.create_subscription(String, f'/{r}/reply', self.on_reply, 10)
        self.flock = self.create_publisher(Float64, '/flock/s', 10)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'formulation': a.formulation, 'variant': 'latent' if self.latent else 'replica',
                       'start_at': W.START_AT, 'plans': [], 'steps': [],
                       'decisions': []}
        self.count = 0
        self.t0 = W.START_AT

    # ─────────────────────────────────────────────────────────── plumbing ──

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel(self):
        return self.sim() - self.t0

    def write(self):
        json.dump(self.record, open(os.path.join(self.a.dir, 'mission.json'), 'w'), indent=1)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def over(self):
        return self.rel() > self.a.limit

    def on_reply(self, m):
        r = json.loads(m.data)
        self.replies[r['id']] = r

    def ask(self, *calls):
        """Send each (robot, action, args) request at once and wait for all
        the answers; a list of the answers, in order."""
        ids = []
        for robot, action, args in calls:
            self.count += 1
            rid = f'mission-{action}-{self.count}'
            msg = String(data=json.dumps({'id': rid, 'action': action, 'args': args}))
            self.requests[robot].publish(msg)
            ids.append((rid, msg, robot))
            self.record['steps'].append({'t': round(self.rel(), 2), 'robot': robot, 'action': action, 'args': args})
        sent = time.time()
        while not all(rid in self.replies for rid, _, _ in ids):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.over():
                return None
            if time.time() - sent > 3.0:
                for rid, msg, robot in ids:
                    if rid not in self.replies:
                        self.requests[robot].publish(msg)
                sent = time.time()
        out = [self.replies[rid] for rid, _, _ in ids]
        for (rid, _, robot), r in zip(ids, out):
            self.record['steps'].append({'t': round(self.rel(), 2), 'robot': robot, 'reply': r})
        return out

    def post(self, robot, action, args):
        """A request whose answer nobody waits for."""
        self.count += 1
        self.requests[robot].publish(String(data=json.dumps({'id': f'mission-{action}-{self.count}',
                                                             'action': action, 'args': args})))
        self.record['steps'].append({'t': round(self.rel(), 2), 'robot': robot, 'action': action, 'args': args})

    def call(self, kind, name, request, wait=60.0):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=wait):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait)
        return future.result()

    # ─────────────────────────────────────────────────── waiting and ending ──

    def wait_to_start(self):
        start = time.time()
        while time.time() - start < 300 and (self.sim() < W.START_AT or len(self.pose) < len(L.ROBOTS)):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.t0 = W.START_AT
        self.record['t0_wall'] = time.time()

    def wait_found(self, flock=False):
        s, last = 0.0, self.sim()
        while self.found is None and not self.over():
            rclpy.spin_once(self, timeout_sec=0.05)
            if flock:
                now = self.sim()
                s += self.flock_speed() * max(0.0, now - last)
                last = now
                self.flock.publish(Float64(data=s))
        if flock:
            self.record['flock_s'] = round(s, 2)
        if self.found:
            self.record['found'] = self.found
            self.get_logger().info(f"[mission] the task is found by {self.found['by']} at {self.found['t']} s")
        return self.found is not None

    def flock_speed(self):
        v = L.SPEED[0]
        for g in L.UGVS:
            st = self.state.get(g, {})
            if st.get('mode') == 'flock':
                v = min(v, L.SPEED[min(st.get('faults', 0), 2)])
        return v

    def finish(self, result):
        end = time.time()
        while self.done is None and time.time() - end < 20.0 and not self.over():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.record['result'] = result
        self.record['done'] = self.done
        self.record['t_mission_sim'] = self.done['t'] if self.done else None
        self.record['t_end_sim'] = round(self.rel(), 2)
        self.record['t1_wall'] = time.time()
        self.record['states'] = self.state
        self.get_logger().info(f'[mission] {self.a.formulation}: {result}; task done {self.done}')

    # ──────────────────────────────────────────────── the makespan estimate ──

    def estimate(self, g, belief=None, met=None):
        """The makespan, from now, of doing the task with g: seeking it at
        particle `belief`, or, met, from where it is, with `met` faults."""
        u = self.pose['u']
        if met is not None:
            return max(fly_time(u, self.found['site']), drive_time(self.pose[g], self.found['site'], met))
        q, when = L.intercept(u, self.rel(), g, belief)
        return (when - self.rel()) + HAIL_EST + max(fly_time(q, self.found['site']),
                                                    drive_time(q, self.found['site'], belief - 1))

    def decide(self, options, why):
        best = min(options, key=lambda o: o[0])
        self.record['decisions'].append({'t': round(self.rel(), 2), 'why': why, 'chosen': best[1],
                                         'options': [[round(c, 1), o] for c, o in options]})
        self.get_logger().info(f'[mission] {why}: ' + ', '.join(f'{o} {c:.0f} s' for c, o in options)
                               + f' -> {best[1]}')
        return best[1]

    def go_and_do(self, g):
        """g has been told of the task: both to the site, then the task."""
        r = self.ask(('u', 'fly_site', ['u']), (g, 'go_site', [g]))
        if r is None or not all(x['ok'] for x in r):
            return 'FAILURE'
        r = self.ask(('u', 'perform', ['u', g]))
        return 'SUCCESS' if r and r[0]['ok'] else 'FAILURE'

    # ──────────────────────────────────────────────────────── formulations ──

    def epistemic(self):
        if not self.wait_found():
            return 'LIMIT'
        self.record['instance'] = self.a.instance
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', ClearProblemKnowledge.Request())
        start = time.time()
        while time.time() - start < 60:
            r = self.call(AddProblem, '/problem_expert/add_problem', AddProblem.Request(problem=P.problem()))
            if r is not None and r.success:
                break
            time.sleep(1.0)
        domain = self.call(GetDomain, '/domain_expert/get_domain', GetDomain.Request())
        problem = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        t, sim0 = time.time(), self.rel()
        r = self.call(GetPlan, '/planner/get_plan', GetPlan.Request(domain=domain.domain, problem=problem.problem),
                      wait=self.a.plan_timeout + 10)
        plan = r.plan if r is not None and r.success and r.plan.items else None
        written = os.path.join(self.a.dir, 'plan.json')
        if os.path.exists(written) and os.path.getmtime(written) >= t:
            shutil.copy(written, os.path.join(self.a.dir, 'policy.json'))
        self.record['plans'].append({'t_sim': round(sim0, 2), 't_sim_end': round(self.rel(), 2),
                                     'seconds': round(time.time() - t, 3),
                                     'items': [f'{i.time:.1f}: {i.action}'
                                               + (f' <{i.epistemic_action}>' if i.epistemic_action else '')
                                               for i in plan.items] if plan else None})
        if plan is None:
            return 'NO_PLAN'
        formulas = LATENT_FORMULAS if self.latent else FORMULAS
        self.record['model_before'] = {f: self.check(f) for f in formulas}
        if not self.execute.wait_for_server(timeout_sec=30.0):
            return 'NO_EXECUTOR'
        sent = self.execute.send_goal_async(ExecutePlan.Goal(plan=plan))
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        goal = sent.result()
        if goal is None or not goal.accepted:
            return 'REJECTED'
        future = goal.get_result_async()
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                res = future.result().result
                self.record['executed'] = [[i.action, ' '.join(i.arguments), i.outcome, i.message_status, i.status]
                                           for i in res.action_execution_status]
                self.record['model_after'] = {f: self.check(f) for f in formulas}
                return RESULT.get(res.result, 'UNKNOWN')
            if self.over():
                goal.cancel_goal_async()
                self.spin(1.0)
                return 'LIMIT'
        return 'INTERRUPTED'

    def check(self, formula):
        r = self.call(CheckFormula, '/epistemic_state/check_formula', CheckFormula.Request(formula=formula),
                      wait=10.0)
        return bool(r.holds) if r is not None and r.success else None

    def particles(self):
        if not self.wait_found():
            return 'LIMIT'
        excluded = {g: set() for g in L.UGVS}
        known = {}                       # faults of a vehicle met and sent on

        def seeking():
            out = []
            for g in L.UGVS:
                if g in known:
                    if known[g] < 2:     # met, capable, sent on: sought again where it now is
                        out.append((self.estimate(g, belief=known[g] + 1), f'{g}@{known[g] + 1}'))
                    continue
                left = [b for b in (1, 2, 3) if b not in excluded[g]]
                if left:
                    out.append((self.estimate(g, belief=left[0]), f'{g}@{left[0]}'))
            return out
        while not self.over():
            options = seeking()
            if not options:
                return 'NO_VEHICLE'
            g, b = self.decide(options, 'seek').split('@')
            b = int(b)
            r = self.ask(('u', 'fly_p1' if b == 1 else f'back_p{b}', ['u', g]))
            if r is None:
                return 'LIMIT'
            r = self.ask(('u', f'hail_p{b}', ['u', g]))
            if r is None:
                return 'LIMIT'
            if r[0]['outcome'] == f'e-absent-{b}':
                excluded[g].add(b)
                continue
            # Met: the vehicle answered with its particle; estimate again with it.
            faults = int(r[0]['why'].split()[-1]) - 1
            known[g] = faults
            options = [(self.estimate(g, met=faults), g)] if faults < 2 else []
            options += [o for o in seeking() if not o[1].startswith(g + '@')]
            choice = self.decide(options, f'met {g}') if options else None
            if choice == g:
                self.ask(('u', 'tell', ['u', g, 'site']))
                return self.go_and_do(g)
            self.ask(('u', 'tell', ['u', g, 'base' if faults >= 2 else 'cover']))
            if faults >= 2:
                self.post(g, 'go_base', [g])
        return 'LIMIT'

    def ideal(self, flock=False):
        if not self.wait_found(flock=flock):
            return 'LIMIT'
        options = [(self.estimate(g, met=self.state[g]['faults']), g) for g in L.UGVS if self.state[g]['capable']]
        g = self.decide(options, 'assign')
        r = self.ask(('u', 'tell', ['u', g, 'site']))
        if r is None or not r[0]['ok']:
            return 'NOT_TOLD'
        return self.go_and_do(g)

    def procedure(self, strength):
        """The variant with latent tool faults: the paper's procedure at the
        given strength, deciding by estimate.Procedure."""
        if not self.wait_found():
            return 'LIMIT'
        site = tuple(self.found['site'])
        proc = E.Procedure(strength, site)
        if strength == 'ideal':
            proc.observe_drives({g: self.state[g]['faults'] for g in L.UGVS})
        for _ in range(24):
            if self.over():
                return 'LIMIT'
            u, t = self.pose['u'], self.rel()
            what, options = proc.decide(u, t)
            self.record['decisions'].append({'t': round(t, 2), 'chosen': list(what),
                                             'options': [[round(v, 1) if math.isfinite(v) else None,
                                                          '+'.join(k) if isinstance(k, tuple) else k]
                                                         for k, v in options.items()]})
            self.get_logger().info(f'[mission] {strength}: {what}')
            if what[0] == 'seek':
                g, b = what[1], what[2]
                if strength == 'ideal':
                    if self.ask(('u', 'tell', ['u', g, 'site'])) is None:
                        return 'LIMIT'
                    self.ask((g, 'go_site', [g]))
                    proc.sent[g] = self.rel() + E.drive_to_site(self.pose[g], site, self.state[g]['faults'])
                    continue
                if self.ask(('u', 'fly_p1' if b == 1 else f'back_p{b}', ['u', g])) is None:
                    return 'LIMIT'
                r = self.ask(('u', f'hail_p{b}', ['u', g]))
                if r is None:
                    return 'LIMIT'
                met = r[0]['outcome'] == f'e-met-{b}'
                proc.observe_hail(g, b, met)
                if not met:
                    continue
                u, t = self.pose['u'], self.rel()
                if strength == 'published':
                    # the paper's procedure, met, estimates again with what it learned
                    here = max(t + E.fly(u, site), t + E.drive_to_site(u, site, b - 1))
                    alt = {h: proc.estimate_published(h, u, t) for h in L.UGVS
                           if h != g and h not in proc.failed and proc.candidates(h)}
                    if alt and min(alt.values()) < here:
                        self.ask(('u', 'tell', ['u', g, 'cover']))
                        continue
                self.ask(('u', 'tell', ['u', g, 'site']))
                self.ask((g, 'go_site', [g]))
                proc.sent[g] = t + E.drive_to_site(u, site, b - 1)
            else:
                for g in sorted((g for g in proc.sent if g not in proc.failed), key=lambda g: proc.sent[g]):
                    r = self.ask(('u', 'perform', ['u', g]))
                    if r is None:
                        return 'LIMIT'
                    done = r[0]['outcome'] == 'e-done'
                    proc.observe_attempt(g, done)
                    if done:
                        return 'SUCCESS'
                proc.sent = {g: a for g, a in proc.sent.items() if g not in proc.failed}
        return 'LIMIT'

    def go(self):
        self.wait_to_start()
        f = self.a.formulation
        if self.latent and f in ('published', 'strong', 'ideal'):
            result = self.procedure(f)
        elif f in ('epistemic', 'timed'):
            result = self.epistemic()
        elif f == 'particles':
            result = self.particles()
        else:
            result = self.ideal(flock=f == 'flock')
        self.finish(result)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=W.FORMULATIONS + W.LATENT_FORMULATIONS)
    ap.add_argument('--instance', default='')
    ap.add_argument('--limit', type=float, default=1500.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=120.0)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Mission(a)
    try:
        node.go()
    finally:
        node.write()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
