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
The epistemic strategy on one instance: ground, solve, replay.

The instance is written as EPDDL (domain.py), grounded by plank, and solved
by Aletheia with consistent beliefs required, as the executor's plugin solves
it. The policy is then replayed through the product update by trace.py's
model, which checks the goal at its leaf, and reports what the strategy
comparison needs: the messages sent, the haulers that delivered, the bay each
crossed, and each robot's beliefs about the bays at the end.
"""

import importlib.util
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from domain import task_text  # noqa: E402

PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER',
                         os.path.expanduser('~/eplansys_ws/install/aletheia/bin/epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))
MAPS = os.path.join(HERE, '..', 'epddl', 'maps.epddl')


def _trace():
    spec = importlib.util.spec_from_file_location('study_trace', os.path.join(HERE, '..', 'tools', 'trace.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


T = _trace()

MESSAGE = re.compile(r'^(send-open|send-blocked|tell-open|tell-shut)_(r\d+)_(r\d+)_(t\d+)$')


def solve(inst, out, timeout=120, threads=None):
    """Ground and solve the instance under `out`. Returns a dict with
    `solved`, `seconds`, and, when solved, the replay."""
    os.makedirs(out, exist_ok=True)
    name = f'{inst.regime}-{inst.seed}'
    domain, problem = task_text(inst, name)
    dpath, ppath = os.path.join(out, 'domain.epddl'), os.path.join(out, 'problem.epddl')
    with open(dpath, 'w') as fh:
        fh.write(domain)
    with open(ppath, 'w') as fh:
        fh.write(problem)
    libs = [LIB] + ([MAPS] if inst.regime != 'elimination' else [])
    g = subprocess.run([PLANK, 'export', '-d', dpath, '-p', ppath, '-l', *libs, '-o', out],
                       capture_output=True, text=True)
    task_path = os.path.join(out, 'problem.json')
    if g.returncode != 0 or not os.path.exists(task_path):
        return {'solved': False, 'seconds': 0.0, 'error': 'plank: ' + (g.stdout + g.stderr)[-400:]}
    plan_path = os.path.join(out, 'plan.json')
    cmd = [PLANNER, '--task', task_path, '--plan', plan_path, '--timeout', str(timeout),
           '--consistent-beliefs']
    if threads:
        cmd += ['--threads', str(threads)]
    started = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True)
    seconds = time.time() - started
    text = r.stdout + r.stderr
    if 'Solution found' not in text:
        why = 'unsolvable' if ('exhausted' in text or 'unreachable' in text.lower()) else 'timeout'
        return {'solved': False, 'seconds': seconds, 'error': why}
    with open(task_path) as fh:
        task = json.load(fh)
    with open(plan_path) as fh:
        plan = json.load(fh)
    actions = _linear(plan)
    rep = replay(task, actions, inst)
    raw = len(rep.get('messages', []))
    if rep.get('goal'):
        actions = eliminate(task, actions, inst)
        rep = replay(task, actions, inst)
    rep.update({'solved': True, 'seconds': seconds, 'messages_before_elimination': raw})
    return rep


def eliminate(task, actions, inst):
    """Action elimination: drop each message, latest first, whose removal
    leaves every action applicable and the goal reached. The planner's
    replanning search does not minimise messages, and a comparison of
    message counts has to be with a policy that sends none it does not
    need."""
    kept = list(actions)
    for k in range(len(kept) - 1, -1, -1):
        if not MESSAGE.match(kept[k]):
            continue
        trial = kept[:k] + kept[k + 1:]
        r = replay(task, trial, inst)
        if r.get('goal') and r.get('consistent'):
            kept = trial
    return kept


def _linear(plan):
    """The actions of a policy that never branches, in order."""
    if isinstance(plan, list):
        return list(plan)
    out, node = [], plan
    while node:
        out.append(node['action'])
        branches = node.get('branches') or []
        if len(branches) > 1:
            raise ValueError('the study domains do not branch')
        node = branches[0]['subtree'] if branches else None
    return out


def replay(task, actions, inst):
    model = T.Model.of(task)
    messages, crossed = [], {}
    for name in actions:
        act = task['actions'][name]
        if not model.applicable(act):
            return {'goal': False, 'error': f'{name} does not apply', 'actions': actions}
        model = model.update(act)
        m = MESSAGE.match(name)
        if m:
            messages.append((m.group(2), m.group(3), m.group(4), m.group(1)))
        parts = name.split('_')
        if parts[0] == 'cross':
            crossed[parts[1]] = parts[2]
        elif parts[0] == 'cross-joint':
            crossed[parts[1]] = parts[3]
            crossed[parts[2]] = parts[3]
    goal = all(model.holds(w, task['goal']['formula']) for w in model.designated)
    atom = 'open_{}' if inst.regime == 'elimination' else 'blocked_{}'
    beliefs = {a: {t: model.belief([a], atom.format(t)) for t in inst.bays} for a in inst.agents}
    return {'goal': goal, 'actions': actions, 'messages': messages, 'crossed': crossed,
            'beliefs': beliefs, 'consistent': model.collapse() is None}
