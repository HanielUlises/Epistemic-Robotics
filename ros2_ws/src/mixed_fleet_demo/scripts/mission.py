#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
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
One run of belief revision in a mixed fleet.

    mission.py --dir RUN --act 3 --formulation perspective

Waits until the porter is back in his lodge from his round at one, then asks
ePlanSys's planner for a policy. The planner's plugin is the
AletheiaPlanSolver with tools/plan.py as its planner: it plans on the act's
EPDDL task under the formulation, and the plugin validates the policy against
the task before handing it over. The policy is executed once, by the epistemic
executor; the revision happens inside it, in the epistemic state: a look or a
report of evidence removes worlds, and a report of belief raises the grades of
a listener that takes the speaker's word. Writes RUN/mission.json and exits;
the launch shuts down when it has.
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

sys.path.insert(0, os.path.join(get_package_share_directory('mixed_fleet_demo'), 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402
import ranks as R  # noqa: E402
import worlds as W  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Mission(Node):

    def __init__(self, a):
        super().__init__('fleet_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.robots = W.robots(a.act)
        self.ward = {r.name: r.ward for r in R.fleet(a.act)}
        self.pose = {}
        self.round1 = False
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        self.create_subscription(String, '/porter/done', self.on_porter, LATCHED)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'act': a.act, 'formulation': a.formulation, 'plans': []}
        self.t0 = 0.0

    def on_odom(self, r, m):
        self.pose[r] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_porter(self, m):
        if m.data == 'round1':
            self.round1 = True

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def call(self, kind, name, request, wait=60.0):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=wait):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait)
        return future.result()

    def ask(self, formula):
        r = self.call(CheckFormula, '/epistemic_state/check_formula', CheckFormula.Request(formula=formula), wait=10.0)
        return bool(r.holds) if r is not None and r.success else None

    def load(self, text):
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', ClearProblemKnowledge.Request())
        start = time.time()
        while time.time() - start < 60:
            r = self.call(AddProblem, '/problem_expert/add_problem', AddProblem.Request(problem=text))
            if r is not None and r.success:
                return True
            time.sleep(1.0)
        return False

    def plan(self):
        self.load(P.problem(self.robots))
        domain = self.call(GetDomain, '/domain_expert/get_domain', GetDomain.Request())
        problem = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        if domain is None or problem is None:
            return None
        t = time.time()
        r = self.call(GetPlan, '/planner/get_plan', GetPlan.Request(domain=domain.domain, problem=problem.problem),
                      wait=self.a.plan_timeout + 10)
        took = time.time() - t
        plan = r.plan if r is not None and r.success else None
        self.record['plans'].append({
            't_sim': round(self.sim() - self.t0, 2), 'seconds': round(took, 3),
            'items': [f'{i.time:.1f}: {i.action}' + (f' <{i.epistemic_action}>' if i.epistemic_action else '')
                      + (f' -> {list(i.children)}' if len(i.children) > 1 else '')
                      for i in plan.items] if plan is not None else None})
        self.get_logger().info(f'[mission] plan in {took:.2f} s: {len(plan.items) if plan is not None else "none"} items')
        return plan

    def run(self, plan):
        if not self.execute.wait_for_server(timeout_sec=30.0):
            return 'NO_EXECUTOR', []
        sent = self.execute.send_goal_async(ExecutePlan.Goal(plan=plan))
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        goal = sent.result()
        if goal is None or not goal.accepted:
            return 'REJECTED', []
        future = goal.get_result_async()
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
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

    def go(self):
        start = time.time()
        while time.time() - start < 600 and (not self.round1 or len(self.pose) < len(self.robots)):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.t0 = self.sim()
        self.record['t0_sim'] = self.t0
        self.record['t0_wall'] = time.time()
        plan = self.plan()
        result, infos = self.run(plan) if plan is not None else ('NO_PLAN', [])
        for _ in range(4):
            rclpy.spin_once(self, timeout_sec=0.3)
        self.record['t_mission_sim'] = round(self.sim() - self.t0, 2)
        self.record['t1_wall'] = time.time()
        self.record['result'] = result
        self.record['reported'] = [[a, ' '.join(g), o, m, s] for a, g, o, m, s in infos]
        where = {}
        for r in self.robots:
            p = self.pose.get(r)
            target = L.BEDS[self.ward[r]]
            where[r] = {'at': [round(v, 3) for v in p] if p else None,
                        'arrived': bool(p and math.dist(p, target) < 0.6)}
        self.record['robots'] = where
        self.record['arrived'] = all(v['arrived'] for v in where.values())
        written = os.path.join(self.a.dir, 'plan.json')
        if os.path.exists(written):
            shutil.copy(written, os.path.join(self.a.dir, 'policy.json'))
        self.get_logger().info(f'[mission] act {self.a.act} {self.a.formulation}: {result} after '
                               f'{self.record["t_mission_sim"]:.1f} s; arrived {self.record["arrived"]}')


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True, choices=W.FORMULATIONS)
    ap.add_argument('--limit', type=float, default=1500.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=60.0)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Mission(a)
    try:
        node.go()
    finally:
        json.dump(node.record, open(os.path.join(a.dir, 'mission.json'), 'w'), indent=1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
