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
One run of replanning on news.

    mission.py --dir RUN --act 2 --formulation replan|null|keep|classical

The courier must reach the resident, crossing the fire-door wall by one of
three doors, of which at least one is open.

replan, null, keep
            the in-process epistemic solver over the act's EPDDL, the policy
            committed late by tools/lazy.py and executed by ePlanSys. When the policy fails, which it does when the
            news node announces into the epistemic state while an action runs,
            the mission asks for a policy again: the planner plans from the
            model the state publishes, the mission as it got, and the news if
            it was announced. Up to ATTEMPTS policies.
classical   POPF over the classical model. The mission keeps the valuations of
            the doors consistent with what it knows: the prior, the news when
            the news states something of the doors, and every door the courier
            found shut. It plans in the most convenient of them: the door
            possibly open whose trip is shortest from where the courier is, the
            only one asserted open. When the news rules that door out the
            execution is cancelled and planned again; when the crossing finds
            the door shut, the drive FAILS with "shut", the door is ruled out,
            and the mission plans again.

Every mission begins at the same simulated instant, worlds.START_AT. Writes
RUN/mission.json and exits; the launch shuts down when it has.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('replanning_news_demo'), 'tools'))
import layout as L  # noqa: E402
import lazy as Z  # noqa: E402
import make_pddl as P  # noqa: E402
import model as M  # noqa: E402
import worlds as W  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}
ATTEMPTS = 6
LATCHED = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Mission(Node):

    def __init__(self, a):
        super().__init__('news_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.robots = W.ROBOTS[a.act]
        self.pose = {}
        self.news = None
        self.belief_version = None
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        self.create_subscription(String, '/news', self.on_news, LATCHED)
        self.create_subscription(String, '/epistemic_state/state', self.on_state, LATCHED)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'act': a.act, 'formulation': a.formulation, 'start_at': W.START_AT, 'news_at': W.NEWS_AT,
                       'plans': [], 'steps': []}

    def on_odom(self, r, m):
        self.pose[r] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_news(self, m):
        if self.news is None:
            self.news = json.loads(m.data)
            self.news['heard_at'] = round(self.sim() - self.t0, 2)

    def on_state(self, m):
        try:
            self.belief_version = json.loads(m.data).get('belief_version')
        except ValueError:
            pass

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write(self):
        json.dump(self.record, open(os.path.join(self.a.dir, 'mission.json'), 'w'), indent=1)

    def call(self, kind, name, request, wait=60.0):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=wait):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait)
        return future.result()

    def load(self, text):
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', ClearProblemKnowledge.Request())
        start = time.time()
        while time.time() - start < 60:
            r = self.call(AddProblem, '/problem_expert/add_problem', AddProblem.Request(problem=text))
            if r is not None and r.success:
                return True
            time.sleep(1.0)
        return False

    def plan(self, text, why):
        self.load(text)
        domain = self.call(GetDomain, '/domain_expert/get_domain', GetDomain.Request())
        problem = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        if domain is None or problem is None:
            return None
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        t, ts = time.time(), self.sim()
        r = self.call(GetPlan, '/planner/get_plan', GetPlan.Request(domain=domain.domain, problem=problem.problem),
                      wait=self.a.plan_timeout + 10)
        took = time.time() - t
        plan = r.plan if r is not None and r.success else None
        written = os.path.join(self.a.dir, 'plan.json')
        if os.path.exists(written) and os.path.getmtime(written) >= t:
            shutil.copy(written, os.path.join(self.a.dir, f"plan_{len(self.record['plans'])}.json"))
        self.record['plans'].append({
            'why': why, 't_sim': round(ts - self.t0, 2), 't_ready': round(self.sim() - self.t0, 2),
            'seconds': round(took, 3), 'belief_version': self.belief_version, 'problem': text,
            'items': [f'{i.time:.1f}: {i.action}' + (f' <{i.epistemic_action}>' if i.epistemic_action else '')
                      + (f' -> {list(i.children)}' if len(i.children) > 1 else '')
                      for i in plan.items] if plan is not None else None})
        self.get_logger().info(f'[mission] plan ({why}) in {took:.2f} s: '
                               f'{len(plan.items) if plan is not None else "none"} items')
        return plan

    def run(self, plan, stop=None):
        """Execute a plan to its end, or until stop() says so; the result, and what each action reported."""
        if not self.execute.wait_for_server(timeout_sec=30.0):
            return 'NO_EXECUTOR', []
        sent = self.execute.send_goal_async(ExecutePlan.Goal(plan=plan))
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        goal = sent.result()
        if goal is None or not goal.accepted:
            return 'REJECTED', []
        future = goal.get_result_async()
        stopped = False
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
            if future.done():
                res = future.result().result
                infos = [(i.action, list(i.arguments), i.outcome, i.message_status, i.status)
                         for i in res.action_execution_status]
                return ('STOPPED' if stopped else RESULT.get(res.result, 'UNKNOWN')), infos
            if self.sim() - self.t0 > self.a.limit:
                goal.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return 'LIMIT', []
            if stop is not None and not stopped and stop():
                stopped = True
                goal.cancel_goal_async()
        return 'INTERRUPTED', []

    def step(self, name, plan, stop=None):
        result, infos = self.run(plan, stop) if plan is not None else ('NO_PLAN', [])
        self.record['steps'].append({'step': name, 't_end': round(self.sim() - self.t0, 2), 'result': result,
                                     'belief_version': self.belief_version,
                                     'reported': [[a, ' '.join(g), o, m, s] for a, g, o, m, s in infos]})
        self.get_logger().info(f'[mission] {name}: {result} at {self.sim() - self.t0:.1f} s')
        return result, infos

    def wait_to_start(self):
        start = time.time()
        while time.time() - start < 300 and (self.sim() < W.START_AT or len(self.pose) < len(self.robots)):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.t0 = self.sim()
        self.record['t0_sim'] = self.t0
        self.record['t0_wall'] = time.time()

    def ask(self, formula):
        r = self.call(CheckFormula, '/epistemic_state/check_formula', CheckFormula.Request(formula=formula),
                      wait=10.0)
        return bool(r.holds) if r is not None and r.success else None

    def finish(self, result):
        for _ in range(4):
            rclpy.spin_once(self, timeout_sec=0.3)
        self.record['t_mission_sim'] = round(self.sim() - self.t0, 2)
        self.record['t1_wall'] = time.time()
        self.record['result'] = result
        self.record['news'] = self.news
        p = self.pose.get('c')
        self.record['courier_at_end'] = [round(v, 3) for v in p] if p else None
        self.record['delivered'] = bool(p and math.dist(p, L.GOAL) < 0.5)
        if self.a.formulation != 'classical':
            self.record['model'] = {f: self.ask(f) for f in
                                    ('at-r_c', '(K c open-w)', '(K c open-m)', '(K c open-e)', '(Kw k open-e)')}
        self.get_logger().info(f'[mission] act {self.a.act} {self.a.formulation}: {result} after '
                               f'{self.record["t_mission_sim"]:.1f} s; delivered {self.record["delivered"]}')

    # -- the epistemic formulations -------------------------------------------------------

    def model_place(self):
        """The courier's place in the epistemic state's model."""
        for p in ('vw', 'vm', 've', 'n', 'r'):
            if self.ask(f'at-{p}_c'):
                return p
        return 'st'

    def committed_late(self, plan):
        """The policy with the courier's drives moved past the looks and messages that do not need them."""
        if plan is None or not plan.items:
            return plan
        late, (moved, dropped) = Z.lazy(plan, 'c', self.model_place())
        rec = self.record['plans'][-1]
        rec['as_planned'] = Z.lines(plan)
        rec['committed_late'] = Z.lines(late)
        rec['drives_moved'], rec['drives_dropped'] = moved, dropped
        return late

    def epistemic(self):
        text = P.problem_vocabulary(self.robots)
        result = 'NO_PLAN'
        for k in range(ATTEMPTS):
            plan = self.plan(text, 'start' if k == 0 else 'the policy failed')
            if plan is not None and not plan.items:
                result = 'SUCCESS' if self.ask('at-r_c') else 'NO_PLAN'
                break
            plan = self.committed_late(plan)
            result, _ = self.step('policy' if k == 0 else f'replan {k}', plan)
            if result != 'FAILURE':
                break
        self.finish(result)

    # -- the classical formulation --------------------------------------------------------

    def consistent(self):
        """The valuations consistent with the prior, the news and the doors found shut."""
        out = []
        for v in M.valuations():
            if any(v[d] for d in self.found_shut):
                continue
            if self.news is not None and self.a.act in (1, 2) and not M.holds(self.a.act, self.a.act, (v, True)):
                continue
            out.append(v)
        return out

    def place(self):
        """The courier's place for the classical problem: where it last arrived, or the nearest."""
        if self.arrived:
            return self.arrived
        nodes = L.roadmap()[0]
        p = self.pose.get('c', nodes['st'])
        return min(('st', 'vw', 'vm', 've'), key=lambda n: math.dist(p, nodes[n]))

    def classical(self):
        self.found_shut = []
        self.arrived = 'st'
        self.record['found_shut'] = []
        self.record['chosen'] = []
        result = 'NO_PLAN'
        for k in range(ATTEMPTS):
            here = self.place()
            possible = sorted({d for v in self.consistent() for d in 'wme' if v[d]}, key=lambda d: P.cost(here, d))
            if not possible:
                result = 'NO_PLAN'
                break
            door = possible[0]
            news_seen = self.news is not None
            self.record['chosen'].append({'t': round(self.sim() - self.t0, 2), 'from': here, 'door': door,
                                          'possible': possible, 'news_known': news_seen})
            plan = self.plan(P.problem_classical(here, [door]), 'start' if k == 0 else 'replan')

            def ruled_out(door=door, news_seen=news_seen):
                return not news_seen and self.news is not None and not any(v[door] for v in self.consistent())
            result, infos = self.step('plan' if k == 0 else f'replan {k}', plan, stop=ruled_out)
            for a, _, _, msg, s in infos:
                if s == 4 and a.startswith('goto_'):
                    self.arrived = a.split('_')[1]
            if result == 'SUCCESS':
                break
            if result == 'STOPPED':
                self.arrived = None
                self.record['news_replan'] = round(self.sim() - self.t0, 2)
                continue
            failed = [(a, msg) for a, _, _, msg, s in infos if s == 3]
            if result == 'FAILURE' and failed and failed[0][1] == 'shut' and failed[0][0].startswith('cross_'):
                d = failed[0][0].split('_')[1]
                self.found_shut.append(d)
                self.arrived = f'v{d}'
                self.record['found_shut'].append({'t': round(self.sim() - self.t0, 2), 'door': d})
                continue
            break
        self.finish(result)

    def go(self):
        self.wait_to_start()
        if self.a.formulation == 'classical':
            self.classical()
        else:
            self.epistemic()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True, choices=W.FORMULATIONS)
    ap.add_argument('--limit', type=float, default=900.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=60.0)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Mission(a)
    node.t0 = 0.0
    try:
        node.go()
    finally:
        node.write()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
