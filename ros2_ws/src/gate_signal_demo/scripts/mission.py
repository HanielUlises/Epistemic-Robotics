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
The two deliveries, given three ways.

    mission.py --dir RUN --formulation epistemic
    mission.py --dir RUN --formulation optimistic|cautious --robot r1|r2

epistemic    One mission for both robots. The problem expert holds only the
             executor's vocabulary; the planner configured is ePlanSys's, which
             plans over the EPDDL task, in which neither robot knows the other's
             mind and r2 does not know the order, and returns a policy. The
             policy is executed once, each robot's actions dispatched to its
             own nodes, independent actions of the two together.
optimistic   One mission per robot, each with its own PlanSys2 and POPF over
cautious     the classical domain, and no knowledge of the other's. r1 reads
             its order and plans its delivery: the back route, the shorter.
             r2 cannot know the order, and the formulation is how it acts
             without it:
               optimistic  it plans its delivery as if both bays were free,
                           and POPF sends it to bay X;
               cautious    the strongest r2 without a radio. It drives into
                           bay X, looks over it from the side door, and keeps
                           looking until it sees r1 there or until the latest
                           time at which r1 could still arrive (--deadline,
                           simulated seconds from the start, set from the
                           slowest classical r1 measured); then it unloads in
                           bay X, or crosses to bay Y and unloads there.

Every mission begins at the same simulated instant, --start-at, so that the
robots of the classical formulations, with stacks of their own, start
together. Writes RUN/mission[_ROBOT].json and exits; the launch shuts down
when every mission has.
"""

import argparse
import json
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetDomain, GetPlan, GetProblem
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('gate_signal_demo'), 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}


def bay_of(x, y):
    for b in ('x', 'y'):
        x0, y0, x1, y1 = L.bay_region(b)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return b
    return ''


class Mission(Node):

    def __init__(self, a):
        super().__init__('gate_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.ns = '' if a.formulation == 'epistemic' else f'/{a.robot}'
        self.pose = {}
        for r in ('r1', 'r2'):
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
        self.execute = ActionClient(self, ExecutePlan, f'{self.ns}/execute_plan')
        self.record = {'formulation': a.formulation, 'robot': a.robot, 'plans': [], 'steps': []}

    def on_odom(self, r, m):
        self.pose[r] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write(self):
        name = 'mission.json' if self.a.formulation == 'epistemic' else f'mission_{self.a.robot}.json'
        json.dump(self.record, open(os.path.join(self.a.dir, name), 'w'), indent=1)

    def call(self, kind, name, request, wait=60.0):
        client = self.create_client(kind, self.ns + name)
        if not client.wait_for_service(timeout_sec=wait):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait)
        return future.result()

    def load(self, text):
        """Replace the problem expert's problem with this one."""
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge',
                  ClearProblemKnowledge.Request())
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
        r = self.call(GetPlan, '/planner/get_plan',
                      GetPlan.Request(domain=domain.domain, problem=problem.problem),
                      wait=self.a.plan_timeout + 10)
        took = time.time() - t
        plan = r.plan if r is not None and r.success and r.plan.items else None
        self.record['plans'].append({
            't_sim': round(self.sim() - self.t0, 2), 'seconds': round(took, 3),
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
                infos = [(i.action, list(i.arguments), i.outcome, i.message_status)
                         for i in res.action_execution_status]
                return RESULT.get(res.result, 'UNKNOWN'), infos
            if self.sim() - self.t0 > self.a.limit:
                goal.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return 'LIMIT', []
        return 'INTERRUPTED', []

    def step(self, name, plan):
        result, infos = self.run(plan) if plan is not None else ('NO_PLAN', [])
        self.record['steps'].append({'step': name, 't_end': round(self.sim() - self.t0, 2),
                                     'result': result,
                                     'reported': [[a, ' '.join(g), o, m] for a, g, o, m in infos]})
        self.get_logger().info(f'[mission] {self.a.robot or "team"} {name}: {result} at '
                               f'{self.sim() - self.t0:.1f} s')
        return result, infos

    def reported(self, infos, action):
        for a, _, outcome, message in infos:
            if a == action:
                return outcome or message
        return ''

    def wait_to_start(self):
        start = time.time()
        while time.time() - start < 300 and (self.sim() < self.a.start_at or len(self.pose) < 2):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.t0 = self.sim()
        self.record['t0_sim'] = self.t0
        self.record['t0_wall'] = time.time()

    def finish(self, result):
        rclpy.spin_once(self, timeout_sec=0.3)
        self.record['t_mission_sim'] = round(self.sim() - self.t0, 2)
        self.record['t1_wall'] = time.time()
        self.record['result'] = result
        self.record['final_bay'] = {r: bay_of(*p) for r, p in self.pose.items()}
        self.get_logger().info(f'[mission] {self.a.formulation} {self.a.robot or "team"}: {result} after '
                               f'{self.record["t_mission_sim"]:.1f} s; bays {self.record["final_bay"]}')

    def go(self):
        self.wait_to_start()
        f, r = self.a.formulation, self.a.robot
        if f == 'epistemic':
            result, _ = self.step('policy', self.plan(P.problem_vocabulary()))
        elif r == 'r1':
            result, infos = self.step('read', self.plan(P.problem_read()))
            order = self.reported(infos, 'read_order')
            self.record['order_read'] = order
            if result == 'SUCCESS' and order in ('x', 'y'):
                result, _ = self.step('deliver', self.plan(P.problem_r1(order)))
            elif result == 'SUCCESS':
                result = 'NO_ORDER'
        elif f == 'optimistic':
            result, _ = self.step('deliver', self.plan(P.problem_r2()))
        else:
            result, _ = self.step('drive in', self.plan(P.problem_r2(goal='(in r2 bay_x)')))
            seen = 'empty'
            while result == 'SUCCESS':
                result, infos = self.step('look', self.plan(P.problem_r2(at='x', goal='(looked r2 bay_x)')))
                seen = self.reported(infos, 'look_bay')
                if seen == 'occupied' or self.sim() - self.t0 >= self.a.deadline:
                    break
            self.record['bay_x_seen'] = seen
            if result == 'SUCCESS':
                free = ('y',) if seen == 'occupied' else ('x',)
                result, _ = self.step('deliver', self.plan(P.problem_r2(at='x', free=free)))
        self.finish(result)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=['epistemic', 'optimistic', 'cautious'])
    ap.add_argument('--robot', default='', choices=['', 'r1', 'r2'])
    ap.add_argument('--start-at', type=float, default=25.0, help='simulated seconds')
    ap.add_argument('--deadline', type=float, default=50.0, help='simulated seconds from the start')
    ap.add_argument('--limit', type=float, default=400.0, help='simulated seconds')
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
