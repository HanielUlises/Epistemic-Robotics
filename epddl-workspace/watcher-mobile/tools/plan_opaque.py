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
A policy that keeps the order from a watcher that knows the policy.

    plan_opaque.py TASK.json OUT_DIR [--planner BIN] [--timeout S]

The product update decides what the watcher knows from the events it sees, and
the planner's policy is correct for that watcher. A watcher that also knows
the policy reads more off the same sights: why the carrier came to its window,
or went to a gate. Whether that tells it the order is checked after planning,
by tools/opacity_sightlines.py. This closes the loop: plan, with each action
branching on what its actor observes (Aletheia, --subjective auto); check;
and while the check finds a run that tells the watcher the order, remove from
the task the action schema at which that run first shows the watcher something
no run of the other order shows, and plan again. Writes OUT_DIR/plan.json, the
first opaque policy, OUT_DIR/task.json, the task it was found in, and
OUT_DIR/loop.json, one entry per round.
"""

import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import opacity_sightlines as O  # noqa: E402


def plan(planner, task_path, plan_path, timeout):
    t = time.time()
    r = subprocess.run([planner, '--task', task_path, '--plan', plan_path, '--conditional', '--subjective', 'auto',
                        '--timeout', str(timeout)], capture_output=True, text=True)
    text = r.stdout + r.stderr
    ok = 'Solution found' in text and os.path.exists(plan_path)
    exp = next((w.split('=')[1] for w in text.split() if w.startswith('Expanded=')), None)
    return ok, {'expanded': int(exp) if exp else None, 'seconds': round(time.time() - t, 2)}


def first_divergence(task, policy):
    """The action schema at which a run first shows the watcher what no run of a
    world of the other order, indistinguishable at the start, shows it."""
    init = task['initial-state']
    every = {w: O.runs(task, policy, init['labels'][w]) for w in init['designated']}
    rel = init['relations']['r3']
    for w in init['designated']:
        for v in rel[w]:
            if ('dest-x' in init['labels'][w]) == ('dest-x' in init['labels'][v]):
                continue
            others = [t for _, t in every[v]]
            for steps, trace in every[w]:
                if trace in others:
                    continue
                # walk the run's steps, recomputing the trace, to the first
                # observation no run of v shows at that position
                seen = []
                world = set(init['labels'][w])
                for step in steps:
                    name, event = step.split(':')
                    action = task['actions'][name]
                    for atom, val in ((action.get('effects') or {}).get(event) or {}).items():
                        (world.add if O.holds(val, world) else world.discard)(atom)
                    obs = O.observe(name, event, world)
                    if obs is None:
                        continue
                    seen.append(obs)
                    if not any(tuple(seen) == t[:len(seen)] for t in others):
                        return name.split('_')[0], w, v, step
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--planner', default=os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
    ap.add_argument('--timeout', type=int, default=600)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    task = json.load(open(a.task))
    loop, banned = [], []
    while True:
        tp, pp = os.path.join(a.out, 'task.json'), os.path.join(a.out, 'plan.json')
        json.dump(task, open(tp, 'w'))
        ok, stats = plan(a.planner, tp, pp, a.timeout)
        entry = {'banned': list(banned), 'solved': ok, **stats}
        if not ok:
            loop.append(entry)
            print(f'round {len(loop)}: no policy with {banned} removed')
            break
        policy = json.load(open(pp))
        div = first_divergence(task, policy)
        entry['opaque'] = div is None
        loop.append(entry)
        if div is None:
            print(f'round {len(loop)}: opaque, with {banned or "nothing"} removed; {stats["expanded"]} expansions')
            break
        schema, w, v, step = div
        entry['divergence'] = {'schema': schema, 'worlds': [w, v], 'step': step}
        print(f'round {len(loop)}: not opaque; in {w} against {v} the watcher first tells them apart at {step}; '
              f'removing {schema}')
        banned.append(schema)
        task = dict(task)
        task['actions'] = {k: x for k, x in task['actions'].items() if k.split('_')[0] != schema}
    json.dump(loop, open(os.path.join(a.out, 'loop.json'), 'w'), indent=1)
    return 0 if loop and loop[-1].get('opaque') else 1


if __name__ == '__main__':
    sys.exit(main())
