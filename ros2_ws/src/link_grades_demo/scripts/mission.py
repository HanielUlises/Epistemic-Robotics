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
The mission of the link grades, given four ways.

    mission.py --dir RUN --formulation epistemic|visit|report|replan --grade full|partial|none

Both robots must come to know whether the corridor is blocked.

epistemic  Aletheia over link-grades.epddl. The link is an atom of the model,
           and the mission asks comm_monitor for its state before planning:
           up, the policy has r1 look and share what its map decides, a
           message that waits for its acknowledgement; down, both robots go
           and look. If the share fails -- no acknowledgement within the wait
           -- the mission loads the task in which r1 knows, r2 does not and
           the link is down, into the executor's model and the planner, and
           replans.
epistemic_open
           The same domain, the link's state unknown to both robots: the
           problem holds a world in which it is up and one in which it is
           down, and nothing tells them apart. A share may then be lost, and
           the planner, which guarantees the goal in the worst case, finds
           that trying the link and falling back is never shorter than not
           trying: both robots go and look, as in visit, and the link is not
           read and not used.
visit      POPF over link-classical.pddl without (link): both go and look.
report     POPF with (link), whatever the link's state: r1 looks and reports,
           fire and forget.
replan     POPF with (link) if comm_monitor says it is up. The report waits
           for its acknowledgement like the share, with the same wait, and
           fails without it; the mission then takes the problem expert's
           state, removes (link) from it, and replans.

Every mission begins at the same simulated instant, grades.START_AT. Writes
RUN/mission.json and exits; the launch shuts down when it has.
"""

import argparse
import json
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
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory('link_grades_demo'), 'tools'))
import grades as R  # noqa: E402
import make_pddl as P  # noqa: E402

RESULT = {ExecutePlan.Result.SUCCESS: 'SUCCESS', ExecutePlan.Result.PREEMPT: 'PREEMPT',
          ExecutePlan.Result.FAILURE: 'FAILURE'}


class Mission(Node):

    def __init__(self, a):
        super().__init__('link_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.pose = {}
        self.decision = {'r1': '', 'r2': ''}
        for r in ('r1', 'r2'):
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
            self.create_subscription(String, f'/{r}/decision', lambda m, r=r: self.decision.__setitem__(r, m.data),
                                     10)
        self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
        self.record = {'formulation': a.formulation, 'grade': a.grade, 'ack_wait': R.ACK_WAIT,
                       'start_at': R.START_AT, 'plans': [], 'steps': []}

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

    def link_up(self):
        """comm_monitor's state of the link, as the system reads it before planning."""
        r = self.call(Trigger, '/comm_monitor/state', Trigger.Request(), wait=10.0)
        up = bool(r.success) if r is not None else None
        self.record.setdefault('link_read', []).append({'t': round(self.sim() - self.t0, 2), 'up': up})
        return up

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

    def use_task(self, which):
        """The epistemic task the next plan is made for: copied to the file the
        planner reads, and loaded into the executor's model."""
        src = os.path.join(self.a.dir, 'task', which, f'problem_{which}.json')
        shutil.copy(src, os.path.join(self.a.dir, 'task', 'current.json'))
        r = self.call(LoadTask, '/epistemic_state/load_task',
                      LoadTask.Request(task_file=os.path.join(self.a.dir, 'task', 'current.json')))
        self.record.setdefault('tasks', []).append({'t': round(self.sim() - self.t0, 2), 'task': which,
                                                    'loaded': bool(r and r.success)})
        return bool(r and r.success)

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
        # Aletheia writes its policy to RUN/plan.json; a replan overwrites it, so each is kept by its number.
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
    def failed(infos, action):
        # ActionExecutionInfo.FAILED is 3
        return any(a == action and s == 3 for a, _, _, _, s in infos)

    def wait_to_start(self):
        start = time.time()
        while time.time() - start < 300 and (self.sim() < R.START_AT or len(self.pose) < 2):
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
        self.record['decision_at_end'] = dict(self.decision)
        if self.a.formulation.startswith('epistemic'):
            self.record['model'] = {f: self.ask(f) for f in ('(Kw r1 blocked)', '(Kw r2 blocked)')}
        self.get_logger().info(f'[mission] {self.a.formulation}/{self.a.grade}: {result} after '
                               f'{self.record["t_mission_sim"]:.1f} s; maps decide {self.decision}; '
                               f'model {self.record.get("model")}')

    def go(self):
        self.wait_to_start()
        f = self.a.formulation
        if f == 'epistemic':
            up = self.link_up()
            self.use_task('up' if up else 'down')
            result, infos = self.step('policy', self.plan(P.problem_vocabulary()))
            if result == 'FAILURE' and self.failed(infos, 'share'):
                self.use_task('lost')
                result, _ = self.step('replan', self.plan(P.problem_vocabulary()))
        elif f == 'epistemic_open':
            self.use_task('open')
            result, _ = self.step('policy', self.plan(P.problem_vocabulary()))
        elif f == 'visit':
            result, _ = self.step('plan', self.plan(P.problem_start(link=False)))
        elif f == 'report':
            result, _ = self.step('plan', self.plan(P.problem_start(link=True)))
        else:
            up = self.link_up()
            result, infos = self.step('plan', self.plan(P.problem_start(link=bool(up))))
            if result == 'FAILURE' and self.failed(infos, 'report'):
                # Replan from what the problem expert holds now, the link removed.
                state = self.call(GetProblem, '/problem_expert/get_problem', GetProblem.Request())
                text = re.sub(r'\(\s*link\s*\)', '', state.problem) if state is not None else ''
                assert not text or not re.search(r'\(\s*link\s*\)', text)
                self.record['replanned_from'] = text
                result, _ = self.step('replan', self.plan(text) if text else None)
        self.finish(result)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=R.FORMULATIONS)
    ap.add_argument('--grade', required=True, choices=R.GRADES)
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
