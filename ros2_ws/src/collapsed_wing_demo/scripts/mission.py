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
One run of the collapsed wing.

    mission.py --dir RUN --formulation epistemic|optimistic|gated

The medic must reach the victim in the north wing, by one of three ways any of
which rubble may block.

epistemic   Aletheia over collapsed-wing.epddl, the policy executed by ePlanSys:
            the scouts drive to their doorways, look and say what they found;
            the medic crosses only where it knows the way is clear, the middle
            passage included when it is known by elimination.
optimistic  POPF over wing-optimistic.pddl, every way assumed clear: the medic
            takes the shortest. When its own laser finds rubble the drive FAILS
            with "blocked"; the mission takes the problem expert's state,
            removes that way's (assumed_clear_<d>), puts the medic in the way,
            and replans, which backs it out to the hall and sends it by another.
gated       POPF over wing-gated.pddl, the medic's crossing gated on a scout's
            check: a check that finds rubble FAILS, and the mission replans from
            the problem expert's state. When nothing is left that a check can
            give, there is no plan, and the run ends with NO_PLAN.

Every mission begins at the same simulated instant, worlds.START_AT. Writes
RUN/mission.json and exits; the launch shuts down when it has.
"""

import argparse
import json
import math
import os
import re
import shutil
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetDomain, GetPlan, GetProblem
from plansys2_epistemic_msgs.srv import CheckFormula, LoadTask
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('collapsed_wing_demo'), 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
import worlds as W  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}


class Mission(Node):

    def __init__(self, a):
        super().__init__('wing_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.pose = {}
        self.decision = {}
        for r in ('s1', 's2', 'm'):
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        for r, d in (('s1', 'e'), ('s2', 'w')):
            self.create_subscription(String, f'/{r}/decision_{d}',
                                     lambda m, k=f'{r}/{d}': self.decision.__setitem__(k, m.data), 10)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'formulation': a.formulation, 'start_at': W.START_AT, 'plans': [], 'steps': []}

    def on_odom(self, r, m):
        self.pose[r] = (m.pose.pose.position.x, m.pose.pose.position.y)

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
        """Replace the problem expert's problem with this one."""
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', ClearProblemKnowledge.Request())
        start = time.time()
        while time.time() - start < 60:
            r = self.call(AddProblem, '/problem_expert/add_problem', AddProblem.Request(problem=text))
            if r is not None and r.success:
                return True
            time.sleep(1.0)
        return False

    def plan(self, text):
        self.load(text)
        domain = self.call(GetDomain, '/domain_expert/get_domain', GetDomain.Request())
        problem = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        if domain is None or problem is None:
            return None
        t = time.time()
        r = self.call(GetPlan, '/planner/get_plan', GetPlan.Request(domain=domain.domain, problem=problem.problem),
                      wait=self.a.plan_timeout + 10)
        took = time.time() - t
        plan = r.plan if r is not None and r.success and r.plan.items else None
        written = os.path.join(self.a.dir, 'plan.json')
        if os.path.exists(written) and os.path.getmtime(written) >= t:
            shutil.copy(written, os.path.join(self.a.dir, f"plan_{len(self.record['plans'])}.json"))
        self.record['plans'].append({
            't_sim': round(self.sim() - self.t0, 2), 'seconds': round(took, 3), 'problem': text,
            'items': [f'{i.time:.1f}: {i.action} [{i.duration:.1f}]'
                      + (f' <{i.epistemic_action}>' if i.epistemic_action else '')
                      for i in plan.items] if plan else None})
        return plan

    def run(self, plan):
        """Execute a plan to its end; the executor's result, and what each action reported."""
        if not self.execute.wait_for_server(timeout_sec=30.0):
            return 'NO_EXECUTOR', []
        sent = self.execute.send_goal_async(ExecutePlan.Goal(plan=plan))
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        goal = sent.result()
        if goal is None or not goal.accepted:
            return 'REJECTED', []
        future = goal.get_result_async()
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                res = future.result().result
                infos = [(i.action, list(i.arguments), i.outcome, i.message_status, i.status)
                         for i in res.action_execution_status]
                return RESULT.get(res.result, 'UNKNOWN'), infos
            if self.sim() - self.t0 > self.a.limit:
                goal.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return 'LIMIT', []
        return 'INTERRUPTED', []

    def step(self, name, plan):
        result, infos = self.run(plan) if plan is not None else ('NO_PLAN', [])
        self.record['steps'].append({'step': name, 't_end': round(self.sim() - self.t0, 2), 'result': result,
                                     'reported': [[a, ' '.join(g), o, m] for a, g, o, m, _ in infos]})
        self.get_logger().info(f'[mission] {name}: {result} at {self.sim() - self.t0:.1f} s')
        return result, infos

    @staticmethod
    def failed(infos):
        """The first action that FAILED (ActionExecutionInfo.FAILED is 3), as (name, message)."""
        for a, _, _, m, s in infos:
            if s == 3:
                return a, m
        return None, None

    def wait_to_start(self):
        start = time.time()
        while time.time() - start < 300 and (self.sim() < W.START_AT or len(self.pose) < 3):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.t0 = self.sim()
        self.record['t0_sim'] = self.t0
        self.record['t0_wall'] = time.time()

    def ask(self, formula):
        r = self.call(CheckFormula, '/epistemic_state/check_formula', CheckFormula.Request(formula=formula),
                      wait=10.0)
        return bool(r.holds) if r is not None and r.success else None

    def state(self):
        r = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        return r.problem if r is not None else ''

    def finish(self, result):
        for _ in range(4):
            rclpy.spin_once(self, timeout_sec=0.3)
        self.record['t_mission_sim'] = round(self.sim() - self.t0, 2)
        self.record['t1_wall'] = time.time()
        self.record['result'] = result
        self.record['decision_at_end'] = dict(self.decision)
        p = self.pose.get('m')
        self.record['medic_at_end'] = [round(v, 3) for v in p] if p else None
        self.record['medic_with_victim'] = bool(p and math.dist(p, L.VICTIM) < 0.5)
        if self.a.formulation == 'epistemic':
            self.record['model'] = {f: self.ask(f) for f in
                                    ('with-victim_m', '(K m clear-w)', '(K m clear-m)', '(K m clear-e)')}
        self.get_logger().info(f'[mission] {self.a.formulation}: {result} after '
                               f'{self.record["t_mission_sim"]:.1f} s; medic with the victim '
                               f'{self.record["medic_with_victim"]}; model {self.record.get("model")}')

    def go(self):
        self.wait_to_start()
        f = self.a.formulation
        if f == 'epistemic':
            result, _ = self.step('policy', self.plan(P.problem_vocabulary()))
            self.finish(result)
            return
        text = P.problem_start(gated=f == 'gated')
        for k in range(6):
            result, infos = self.step('plan' if k == 0 else f'replan {k}', self.plan(text) if text else None)
            what, why = self.failed(infos)
            if result != 'FAILURE' or what is None or why != 'blocked':
                break
            # Replan from what the problem expert holds now.
            text = self.state()
            d = what.split('_')[-1]
            if f == 'optimistic' and what.startswith('cross_'):
                text = re.sub(rf'\(\s*assumed_clear_{d}\s*\)', '', text)
                text = re.sub(r'\(\s*:init', f'( :init (in_way_{d} m)', text, count=1)
            self.record.setdefault('found_blocked', []).append({'t': round(self.sim() - self.t0, 2),
                                                                'action': what, 'way': d})
        self.finish(result)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=W.FORMULATIONS)
    ap.add_argument('--limit', type=float, default=900.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=120.0)
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
