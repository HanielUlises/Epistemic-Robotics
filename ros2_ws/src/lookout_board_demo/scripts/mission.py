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
The order, given three ways.

    mission.py --dir RUN --formulation epistemic|replan|conformant

epistemic    The problem expert is populated with the classical problem, which
             is only the executor's vocabulary; the planner configured is
             ePlanSys's, which plans over the EPDDL task and returns a policy.
             The policy is executed once.
replan       The strongest classical planner for this task: POPF, in a loop
             that monitors execution. It plans to know the zone from the
             nearest lookout not yet found screened, executes, reads the map,
             and if the zone is still undecided marks that lookout screened
             and plans again from where the robot stands. Every action of the
             epistemic domain is in its model, reading the board included.
conformant   The classical plan that knows without monitoring: a look from
             every lookout, in the shortest order there is, found by trying
             all twenty-four (POPF's own tour is longer), and executed as one
             plan.

The order is given once the mapper has published its first grid, since before
it the drive node has no pose. Writes RUN/mission.json and exits; the launch
shuts down with it.
"""

import argparse
import itertools
import json
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetPlan
from plansys2_msgs.srv import GetDomain, GetProblem
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('lookout_board_demo'), 'tools'))
import layout as L  # noqa: E402
import make_pddl as P  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}
FREE_BELOW, OCCUPIED_ABOVE = 25, 65


def classify(grid, box):
    """The perception's rule over the grid: blocked, clear or unknown."""
    if grid is None:
        return 'unknown'
    info = grid.info
    r, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    all_free, any_cell = True, False
    for j in range(info.height):
        cy = oy + (j + 0.5) * r
        if cy < box[1] or cy > box[3]:
            continue
        row = j * info.width
        for i in range(info.width):
            cx = ox + (i + 0.5) * r
            if cx < box[0] or cx > box[2]:
                continue
            any_cell = True
            v = grid.data[row + i]
            if v > OCCUPIED_ABOVE:
                return 'blocked'
            if v < 0 or v >= FREE_BELOW:
                all_free = False
    return 'clear' if any_cell and all_free else 'unknown'


class Mission(Node):

    def __init__(self, args):
        super().__init__('lookout_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = args
        self.grid = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'formulation': args.formulation, 'plans': [], 'visited': []}

    def on_map(self, m):
        self.grid = m

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
        self.call(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge',
                  ClearProblemKnowledge.Request())
        start = time.time()
        while time.time() - start < 60:
            r = self.call(AddProblem, '/problem_expert/add_problem', AddProblem.Request(problem=text))
            if r is not None and r.success:
                return True
            time.sleep(1.0)
        return False

    def plan(self):
        domain = self.call(GetDomain, '/domain_expert/get_domain', GetDomain.Request())
        problem = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
        if domain is None or problem is None:
            return None, 0.0
        domain, problem = domain.domain, problem.problem
        t = time.time()
        r = self.call(GetPlan, '/planner/get_plan', GetPlan.Request(domain=domain, problem=problem),
                      wait=self.a.plan_timeout + 10)
        took = time.time() - t
        if r is None or not r.success or not r.plan.items:
            return None, took
        return r.plan, took

    def run(self, plan):
        """Execute a plan to its end; returns the executor's result."""
        if not self.execute.wait_for_server(timeout_sec=10.0):
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
                return RESULT.get(future.result().result.result, 'UNKNOWN')
            if self.sim() - self.record['t0_sim'] > self.a.limit:
                goal.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return 'LIMIT'
        return 'INTERRUPTED'

    def zone(self):
        rclpy.spin_once(self, timeout_sec=0.5)
        return classify(self.grid, L.zone_region())

    def wait_for_map(self):
        start = time.time()
        while self.grid is None and time.time() - start < 120:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.grid is not None

    def items_text(self, plan):
        return [f'{i.time:.1f}: {i.action} [{i.duration:.1f}]'
                + (f' <{i.epistemic_action}>' if i.epistemic_action else '') for i in plan.items]

    def go(self):
        if not self.wait_for_map():
            self.record.update(result='NO_MAP', success=False)
            return
        self.record['t0_sim'] = self.sim()
        self.record['t0_wall'] = time.time()
        f = self.a.formulation
        if f == 'epistemic':
            self.load(P.problem())
            plan, took = self.plan()
            self.record['plans'].append({'t_sim': self.sim(), 'seconds': took,
                                         'items': self.items_text(plan) if plan else None})
            if plan is None:
                self.record.update(result='NO_PLAN', success=False)
                return
            self.record['policy_nodes'] = len(plan.items)
            result = self.run(plan)
        elif f == 'replan':
            at, screened, result = 'start', [], 'NO_PLAN'
            while self.sim() - self.record['t0_sim'] < self.a.limit:
                self.load(P.problem(at=at, screened=screened))
                plan, took = self.plan()
                self.record['plans'].append({'t_sim': self.sim(), 'seconds': took, 'at': at,
                                             'screened': list(screened),
                                             'items': self.items_text(plan) if plan else None})
                if plan is None:
                    result = 'NO_PLAN'
                    break
                target = plan.items[-1].action.strip('()').split()[-1]
                result = self.run(plan)
                self.record['visited'].append(target)
                if result != 'SUCCESS':
                    break
                z = self.zone()
                self.get_logger().info(f'[mission] after the look from {target} the zone reads {z}')
                if z != 'unknown':
                    break
                screened.append(target)
                at = target
        else:
            order = min(itertools.permutations(P.LOOKOUTS),
                        key=lambda o: sum(P.seconds(a, b) for a, b in zip(('start',) + o, o)))
            self.load(P.problem(goal='conformant'))
            plan, t, prev = Plan(), 0.0, 'start'
            for lk in order:
                d = P.seconds(prev, lk)
                plan.items.append(PlanItem(time=t, action=f'(goto r1 {prev} {lk})', duration=d))
                t += d + 0.001
                plan.items.append(PlanItem(time=t, action=f'(look_zone r1 {lk})', duration=3.0))
                t += 3.001
                prev = lk
            self.record['plans'].append({'t_sim': self.sim(), 'seconds': 0.0,
                                         'items': self.items_text(plan), 'order': list(order)})
            self.record['visited'] = list(order)
            result = self.run(plan)
        self.record['t1_sim'] = self.sim()
        self.record['t1_wall'] = time.time()
        self.record['t_mission_sim'] = self.record['t1_sim'] - self.record['t0_sim']
        self.record['result'] = result
        self.record['zone_on_map'] = self.zone()
        self.record['success'] = result == 'SUCCESS'
        self.get_logger().info(
            f'[mission] {f}: {result} after {self.record["t_mission_sim"]:.1f} s; '
            f'the zone on the map reads {self.record["zone_on_map"]}')


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=['epistemic', 'replan', 'conformant'])
    ap.add_argument('--limit', type=float, default=600.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=60.0)
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
