#!/usr/bin/env python3
"""Walk one branch of the policy through the epistemic state, asking the three
formulas after every step. No simulator: this is the model half alone, and it
is what the Gazebo run is supposed to reproduce."""
import sys, time
import rclpy
from rclpy.node import Node
from plansys2_epistemic_msgs.srv import CheckFormula, ApplyAction, LoadTask

WATCH = [
    ('Kw_r1_P',      '(Kw r1 pallet-at_bay2)'),
    ('K_r2_Kw_r1_P', '(K r2 (Kw r1 pallet-at_bay2))'),
    ('Kw_r2_P',      '(Kw r2 pallet-at_bay2)'),
]

class T(Node):
    def __init__(self):
        super().__init__('trace')
        self.load = self.create_client(LoadTask, 'epistemic_state/load_task')
        self.chk = self.create_client(CheckFormula, 'epistemic_state/check_formula')
        self.app = self.create_client(ApplyAction, 'epistemic_state/apply_action')
        for c in (self.load, self.chk, self.app):
            c.wait_for_service(timeout_sec=20.0)

    def call(self, cli, req):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        return f.result()

    def row(self, label):
        cells = []
        for name, text in WATCH:
            r = self.call(self.chk, CheckFormula.Request(formula=text))
            cells.append('TRUE ' if (r and r.success and r.holds)
                         else ('FALSE' if (r and r.success) else 'ERR:' + (r.error if r else '?')))
        print('%-34s %s' % (label, '  '.join('%-13s %s' % (n, c) for (n, _), c in zip(WATCH, cells))))
        return cells

def main():
    task, steps = sys.argv[1], sys.argv[2:]
    rclpy.init()
    t = T()
    r = t.call(t.load, LoadTask.Request(task_file=task))
    print('load:', r.success, r.error)
    t.row('init')
    for s in steps:
        action, _, outcome = s.partition('@')
        r = t.call(t.app, ApplyAction.Request(epistemic_action=action, observed_outcome=outcome))
        if not (r and r.success):
            print('  !! apply %s failed: %s' % (s, r.error if r else 'no reply'))
            break
        t.row('after ' + s)
    rclpy.shutdown()

main()
