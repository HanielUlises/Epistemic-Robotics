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
One run of the gossip demonstration in Gazebo.

    mission.py --dir RUN --formulation proposed|ideal|flock|epistemic

The robots are three Burgers. What every formulation decides is decided by
the library the kinematic simulator runs (tools/gossip): the coverage from
the common belief, the radio, the paper's allocation and gossip, the flock;
the simulator's robots are the Burgers, whose positions are their odometry
and whose motions become velocity commands. A robot's range sensor and the
tasks it finds are the simulator's, over the arena's true map
(tools/arena.py); the laser stops a robot short of anything in front of it.

Under the epistemic formulation the decision at a task found is not the
library's own executor: the planning task is written as an EPDDL problem and
grounded by plank, ePlanSys's epistemic state loads it, ePlanSys's planner
plans it with Aletheia (through plansys2/AletheiaPlanSolver), and ePlanSys's
epistemic executor runs the policy. Each action reaches this node through a
relay (src/relay_node.cpp) on /<robot>/request, is carried out by the
library's action semantics, and is answered on /<robot>/reply with the event
that occurred, by which the executor chooses the branch. One executor runs one
policy at a time; a task found while it is busy waits for it.

Writes RUN/mission.json and RUN/frames.jsonl, and exits; the launch shuts
down when it has.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('gossip_coverage_demo')
sys.path.insert(0, os.path.join(SHARE, 'tools'))
import arena as A  # noqa: E402
import make_pddl as V  # noqa: E402
from gossip import epddl as P  # noqa: E402
from gossip import epistemic as EP  # noqa: E402
from gossip import nav  # noqa: E402
from gossip import sim as S  # noqa: E402
from gossip.env import OCC  # noqa: E402

TURN = 0.8            # rad of heading error beyond which a robot turns on the spot
STOP = 0.22           # m, a return this close ahead stops the robot
LOOK = 0.5            # rad either side of the heading that counts as ahead
START_AT = 5.0        # s of simulated time before the robots set out


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GazeboSim(S.Sim):
    """The library's simulator, whose robots are the Burgers."""

    def __init__(self, node, area, case, method, seed):
        self.node = node
        self.cmd = {}
        self.inflated = {}
        super().__init__(area, len(A.ROBOTS), case, method, seed)
        if method == 'epistemic':
            self.planner = Bridge(self, node)

    def blocked(self, r):
        """A robot plans around what it knows to be occupied, a cell wider."""
        key = (r.id, r.version)
        if key not in self.inflated:
            self.inflated = {k: v for k, v in self.inflated.items() if k[0] != r.id}
            self.inflated[key] = nav.dilate(r.known == OCC, 1)
        return self.inflated[key]

    def drive(self, r, goal, speed):
        w = r.mover.waypoint(r.pos, goal, self.blocked(r), r.version)
        self.cmd[r.id] = (w, speed, goal)


class BridgeRun(EP.Run):
    """A policy that ePlanSys plans and executes. The library's semantics of
    every action are kept; what is dispatched when, and which branch is taken,
    is the executor's."""

    def __init__(self, planner, finder, tid, inst, task_path):
        super().__init__(planner, finder, tid, inst, None, [], None)
        self.task_path = task_path
        self.state = 'load'
        self.requests = {}           # request id -> (robot, item, st)
        self.policy = None
        self.done_steps = []         # (action, event) executed
        self.future = None
        self.goal = None

    def remaining_agents(self):
        """The robots named by an action not yet done on any branch of the
        policy that agrees with what has been done."""
        if self.policy is None:
            return set(self.inst.agents)
        out = set()
        done = list(self.done_steps)

        def walk(node, todo):
            if node is None:
                return [todo]
            paths = []
            for b in node['branches']:
                ev = b['event']
                paths += walk(b['subtree'], todo + [(node['action'], ev)])
            return paths
        events = self.task['actions']
        for path in walk(self.policy, []):
            names = [(a, events[a]['designated'][e] if isinstance(e, int) else e) for a, e in path]
            if all(d in names for d in done):
                for a, e in names:
                    if (a, e) not in done:
                        out |= EP.SCH.agents_of(a)
        return out

    def step(self):
        """Advance the actions the executor has asked for."""
        if self.ended:
            return
        for rid, (robot, it, st) in list(self.requests.items()):
            if self.ended:
                return
            if not st['done']:
                self.advance(it, st)
            if st['done'] and not st.get('answered'):
                st['answered'] = True
                self.done_steps.append((it.action, st['event']))
                self.p.reply(robot, rid, True, st['event'])
                self.release_idle()

    def release_idle(self):
        left = self.remaining_agents()
        done = self.sim.tasks[self.tid]['done']
        busy = {it.agents_owner for _, it, st in self.requests.values() if not st['done']} \
            if self.requests else set()
        for m in list(self.members):
            if m not in left and m not in busy and self.p.member.get(m) is self:
                if self.sim.robots[m].status == 'task' and not done:
                    continue
                self.p.release(m, self)


class Item:
    """The library's view of one dispatched action."""

    def __init__(self, action):
        self.action = action
        self.agents = EP.SCH.agents_of(action)
        self.agents_owner = action.split('_')[1]


class Bridge(EP.Planner):
    """The epistemic method in Gazebo: the planning task is the library's
    (EP.Planner.instance); planning and execution are ePlanSys's."""

    def __init__(self, sim, node):
        super().__init__(sim)
        self.node = node
        self.queue = []
        self.current = None

    def start(self, r, tid, comp):
        if self.current is not None:
            if all(q[1] != tid for q in self.queue):
                self.queue.append((r.id, tid))
                self.sim.log('queued', robot=r.id, task=tid)
            return
        inst = self.instance(r, tid, comp)
        out = os.path.join(self.node.run, 'policies', inst.name)
        try:
            task_path = P.export(inst, out)
        except Exception as e:     # noqa: BLE001
            self.sim.log('no-policy', robot=r.id, task=tid, why=str(e)[:200])
            return
        run = BridgeRun(self, r.id, tid, inst, task_path)
        run.task = json.load(open(task_path))
        self.current = run
        self.enlist(run)
        self.sim.log('policy', robot=r.id, task=tid, agents=inst.agents, knowers=sorted(inst.knowers),
                     may_be_busy=inst.may_be_busy, instance=inst.name)
        self.sim.counts['policies'] += 1
        self.node.begin_policy(run)

    def request(self, robot, msg):
        run = self.current
        if run is None or run.ended:
            self.reply(robot, msg['id'], False, '', 'no policy is running')
            return
        if msg['id'] in run.requests:
            st = run.requests[msg['id']][2]
            if st['done']:
                self.reply(robot, msg['id'], True, st['event'])
            return
        action = vocab_to_epddl(msg['action'], msg['args'])
        it = Item(action)
        st = {'done': False, 'event': None, 't0': self.sim.t}
        run.requests[msg['id']] = (robot, it, st)
        run.dispatch(it, st)

    def reply(self, robot, rid, ok, outcome, why=''):
        self.node.reply(robot, {'id': rid, 'ok': ok, 'outcome': outcome or '', 'why': why})

    def finished(self, run, result):
        if run.ended:
            return
        self.sim.log('policy-result', task=run.tid, result=result)
        if result != 'SUCCESS' and not self.sim.tasks[run.tid]['done']:
            self.retry_at[run.tid] = (self.sim.t + 10.0, run.finder)
        run.end()
        self.current = None

    def fail(self, run, why, fallback=None):
        # an await out of patience: the executor is told the action failed,
        # and the policy ends when it reports so
        self.sim.log('policy-failed', task=run.tid, why=why)
        self.sim.counts['failed'] += 1
        for rid, (robot, it, st) in run.requests.items():
            if not st['done']:
                st['done'], st['answered'] = True, True
                self.reply(robot, rid, False, '', why)
        self.node.cancel_policy(run)

    def cancel_all(self):
        if self.current is not None:
            self.node.cancel_policy(self.current)
            run = self.current
            run.ended = True
            for m in list(run.members):
                if self.member.get(m) is run:
                    self.member.pop(m, None)
                    self.sim.robots[m].pjob = None
            if run in self.runs:
                self.runs.remove(run)
            self.current = None
        self.queue.clear()
        self.pending.clear()

    def step(self):
        s = self.sim
        if self.current is None and self.queue:
            g, tid = self.queue.pop(0)
            r = s.robots[g]
            if not s.tasks[tid]['done'] and self.free(r):
                comp = next(c for c in s.components() if g in c)
                if len(comp) < len(s.ids):
                    self.start(r, tid, comp)
        for tid, (t, g) in list(self.retry_at.items()):
            if s.tasks[tid]['done']:
                del self.retry_at[tid]
            elif s.t >= t and self.free(s.robots[g]) and self.current is None:
                del self.retry_at[tid]
                comp = next(c for c in s.components() if g in c)
                if len(comp) < len(s.ids):
                    self.start(s.robots[g], tid, comp)
        if self.current is not None:
            self.current.step()


def vocab_to_epddl(action, args):
    if action in ('call', 'await', 'leave'):
        return f'{action}_{args[0]}_{args[1]}'
    if action.startswith('go_site_'):
        return f"go-site-{action.rsplit('_', 1)[1]}_{args[0]}"
    if action.startswith('do_task_'):
        return f"do-task-{action.rsplit('_', 1)[1]}_{args[0]}"
    raise ValueError(action)


class Mission(Node):

    def __init__(self, a):
        super().__init__('gc_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.run = a.dir
        self.world = A.load(a.dir)
        self.area = A.area(self.world)
        self.pose, self.front = {}, {r: math.inf for r in A.ROBOTS}
        for r in A.ROBOTS:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.on_odom(r, m), 10)
            self.create_subscription(LaserScan, f'/{r}/scan', lambda m, r=r: self.on_scan(r, m),
                                     rclpy.qos.qos_profile_sensor_data)
            self.create_subscription(String, f'/{r}/request', lambda m, r=r: self.on_request(r, m), 50)
        self.cmd = {r: self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in A.ROBOTS}
        self.replies = {r: self.create_publisher(String, f'/{r}/reply', 10) for r in A.ROBOTS}
        self.events_pub = self.create_publisher(String, '/gc/events', 50)
        self.frame_pub = self.create_publisher(String, '/gc/frame', 10)
        self.sim = None
        self.stuck = {r: [None, 0.0, 0.0] for r in A.ROBOTS}     # (pose, since, turning until)
        self.t0 = None
        self.frames = open(os.path.join(a.dir, 'frames.jsonl'), 'w')
        self.record = {'formulation': a.formulation, 'seed': self.world['seed'], 'case': self.world['case'],
                       'policies': []}
        self.policy_calls = {}
        self.inbox = []
        if a.formulation == 'epistemic':
            from plansys2_msgs.action import ExecutePlan
            from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetDomain, GetPlan
            from plansys2_epistemic_msgs.srv import LoadTask
            self.ExecutePlan = ExecutePlan
            self.srv = {'load': self.create_client(LoadTask, '/epistemic_state/load_task'),
                        'plan': self.create_client(GetPlan, '/planner/get_plan'),
                        'domain': self.create_client(GetDomain, '/domain_expert/get_domain'),
                        'add': self.create_client(AddProblem, '/problem_expert/add_problem'),
                        'clear': self.create_client(ClearProblemKnowledge,
                                                    '/problem_expert/clear_problem_knowledge')}
            self.types = {'load': LoadTask, 'plan': GetPlan, 'domain': GetDomain, 'add': AddProblem,
                          'clear': ClearProblemKnowledge}
            self.execute = ActionClient(self, ExecutePlan, '/execute_plan')
            self.domain_text = None

    # ──────────────────────────────────────────────────────────── inputs ──

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, r, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[r] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_scan(self, r, m):
        best = math.inf
        for i, d in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if math.isfinite(d) and d > m.range_min and abs(wrap(ang)) < LOOK:
                best = min(best, d)
        self.front[r] = best

    def on_request(self, r, m):
        self.inbox.append((r, json.loads(m.data)))

    def event(self, kind, **kw):
        self.events_pub.publish(String(data=json.dumps(dict(kw, kind=kind))))

    def reply(self, robot, body):
        self.replies[robot].publish(String(data=json.dumps(body)))

    # ─────────────────────────────────────────────────────── ePlanSys calls ──

    def call(self, name, request, wait=60.0):
        client = self.srv[name]
        if not client.wait_for_service(timeout_sec=wait):
            return None
        future = client.call_async(request)
        end = time.time() + wait
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def setup_epistemic(self):
        T = self.types
        start = time.time()
        while time.time() - start < 120:
            self.call('clear', T['clear'].Request(), wait=30.0)
            r = self.call('add', T['add'].Request(problem=V.problem()), wait=30.0)
            if r is not None and r.success:
                break
            time.sleep(1.0)
        d = self.call('domain', T['domain'].Request(), wait=60.0)
        self.domain_text = d.domain if d is not None else ''
        if not self.execute.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('no executor')

    def begin_policy(self, run):
        """Load the task into the epistemic state, plan it with ePlanSys's
        planner, and hand the policy to its executor. The robots stand while
        the planner works, and the time it takes counts in the mission's."""
        T = self.types
        t_wall, t_sim = time.time(), self.sim.t
        for rr in A.ROBOTS:
            self.cmd[rr].publish(Twist())
        r = self.call('load', T['load'].Request(task_file=run.task_path), wait=30.0)
        if r is None or not r.success:
            self.get_logger().error(f'load_task failed: {getattr(r, "error", None)}')
        task_json = open(run.task_path).read()
        plan = self.call('plan', T['plan'].Request(domain=self.domain_text, problem=task_json),
                         wait=self.a.plan_timeout + 10)
        seconds = round(time.time() - t_wall, 3)
        items = plan.plan.items if plan is not None and plan.success else []
        written = os.path.join(self.run, 'plan.json')
        if os.path.exists(written):
            shutil.copy(written, os.path.join(os.path.dirname(run.task_path), 'policy.json'))
            run.policy = json.load(open(written))
        self.record['policies'].append({'task': run.tid, 'finder': run.finder, 'instance': run.inst.name,
                                        't_sim': round(t_sim, 2), 'seconds': seconds, 'items': len(items),
                                        'agents': run.inst.agents})
        if not items:
            self.sim.log('no-policy', robot=run.finder, task=run.tid)
            self.sim.planner.finished(run, 'NO_PLAN')
            return
        sent = self.execute.send_goal_async(self.ExecutePlan.Goal(plan=plan.plan))
        end = time.time() + 10.0
        while not sent.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        goal = sent.result() if sent.done() else None
        if goal is None or not goal.accepted:
            self.sim.planner.finished(run, 'REJECTED')
            return
        run.goal = goal
        run.future = goal.get_result_async()
        self.event('policy', task=run.tid, finder=run.finder, items=len(items))

    def cancel_policy(self, run):
        if run.goal is not None:
            run.goal.cancel_goal_async()

    def poll_policy(self):
        b = self.sim.planner if isinstance(self.sim.planner, Bridge) else None
        if b is None or b.current is None or b.current.future is None:
            return
        f = b.current.future
        if f.done():
            res = f.result().result
            names = {self.ExecutePlan.Result.SUCCESS: 'SUCCESS', self.ExecutePlan.Result.PREEMPT: 'PREEMPT',
                     self.ExecutePlan.Result.FAILURE: 'FAILURE'}
            out = names.get(res.result, 'UNKNOWN')
            self.record['policies'][-1]['result'] = out
            self.record['policies'][-1]['executed'] = [[i.action, ' '.join(i.arguments), i.outcome, i.status]
                                                       for i in res.action_execution_status]
            b.finished(b.current, out)

    # ─────────────────────────────────────────────────────────── control ──

    def steer(self, r, target, speed):
        x, y, yaw = self.pose[r]
        d = math.dist((x, y), target)
        err = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        out = Twist()
        st = self.stuck[r]
        t = self.now()
        if st[2] > t:
            out.angular.z = 1.0
            self.cmd[r].publish(out)
            return
        if d < 0.04:
            self.cmd[r].publish(out)
            return
        if abs(err) > TURN:
            out.angular.z = math.copysign(1.0, err)
        else:
            v = min(speed, 0.22, 0.8 * d + 0.03)
            if self.front[r] < STOP:
                v = 0.0
            out.linear.x = v
            out.angular.z = max(-1.5, min(1.5, 2.5 * err))
        # a robot that wants to move and has not for four seconds turns aside
        if st[0] is None or math.dist(st[0], (x, y)) > 0.05:
            self.stuck[r] = [(x, y), t, 0.0]
        elif t - st[1] > 4.0 and self.front[r] < STOP + 0.1:
            self.stuck[r] = [(x, y), t, t + 1.2]
        self.cmd[r].publish(out)

    # ─────────────────────────────────────────────────────────────── run ──

    def frame(self):
        s = self.sim
        return {'t': round(s.t, 2), 'wall': round(time.time(), 3),
                'robots': {r.id: [round(r.pos[0], 3), round(r.pos[1], 3), r.status,
                                  r.pjob[1] if r.pjob else (r.job[1] if r.job and r.job[0] == 'seek' else None)]
                           for r in s.robots.values()},
                'particles': ({r: [round(v, 3) for v in s.cb.pos(r, s.t)] for r in s.ids} if s.cb else None),
                'flock': list(s.flock['c']) if s.flock else None,
                'tasks': {tid: [x['found_t'] is not None, x['done']] for tid, x in s.tasks.items()},
                'knows': {r.id: sorted(t for t in r.k.tasks) for r in s.robots.values()}}

    def go(self):
        start = time.time()
        while time.time() - start < 300 and (len(self.pose) < len(A.ROBOTS) or self.now() < START_AT):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.a.formulation == 'epistemic':
            self.setup_epistemic()
        self.t0 = self.now()
        self.record['t0_wall'] = time.time()
        self.sim = GazeboSim(self, self.area, self.world['case'], self.a.formulation, self.world['seed'])
        for r in self.sim.robots.values():
            r.pos = self.pose[r.id][:2]
            r.k.tasks.clear()
            self.area.sense(r.pos, r.known, r.seen)
        self.sim.begin()
        last_frame, n_events = -1.0, 0
        result = 'LIMIT'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            t = self.now() - self.t0
            if t - self.sim.t < self.sim.spec.dt:
                continue
            for r in self.sim.robots.values():
                p = self.pose[r.id][:2]
                r.travelled += math.dist(r.pos, p)
                r.pos = p
            while self.inbox:
                robot, msg = self.inbox.pop(0)
                if isinstance(self.sim.planner, Bridge):
                    self.sim.planner.request(robot, msg)
            self.sim.cmd = {}
            over = self.sim.step(t=round(t, 3))
            if self.a.formulation == 'epistemic':
                self.poll_policy()
            for r in A.ROBOTS:
                c = self.sim.cmd.get(r)
                if c is None:
                    self.cmd[r].publish(Twist())
                else:
                    self.steer(r, c[0], c[1])
            for e in self.sim.events[n_events:]:
                self.events_pub.publish(String(data=json.dumps(e)))
            n_events = len(self.sim.events)
            if t - last_frame >= 0.5:
                last_frame = t
                fr = self.frame()
                self.frames.write(json.dumps(fr) + '\n')
                self.frame_pub.publish(String(data=json.dumps(fr)))
            if over:
                result = 'SUCCESS'
                break
            if t > self.a.limit:
                break
        for r in A.ROBOTS:
            self.cmd[r].publish(Twist())
        out = self.sim.result()
        out.update(self.record, result=result, t1_wall=time.time())
        self.frames.close()
        json.dump(out, open(os.path.join(self.run, 'mission.json'), 'w'), indent=1)
        self.get_logger().info(f'[mission] {self.a.formulation}: {result} at {out["mission"]} s')


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--formulation', required=True, choices=('proposed', 'ideal', 'flock', 'epistemic'))
    ap.add_argument('--limit', type=float, default=900.0, help='simulated seconds')
    ap.add_argument('--plan-timeout', type=float, default=60.0)
    a = ap.parse_args(argv)
    rclpy.init()
    node = Mission(a)
    try:
        node.go()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
