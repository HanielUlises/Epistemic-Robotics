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
Replays a policy of belief-empathy.epddl in each of its worlds, and says what
every robot knows after every step.

    trace_policy.py TASK.json POLICY.json [--json OUT] [--quiet]

For each designated world of the initial model, the policy is followed along
the branch that world takes: at each node, the event whose precondition the
actual world satisfies. The model is updated by the product update of the
repository's tools/plan_trace.py, and the formulas below are evaluated at the
actual world after every step with tools/model_check.py's labelling
algorithm. The vehicle sought first is s and the other o (g2 and g1 when the
site is in the east, g1 and g2 in the west).

The formulas are Bramblett and Bezzo's update rules, read in the language of
the domain:

  own        K_s p_s: each ground vehicle knows the particle it tracks, eq. (13)
  empathy    K_s ¬Kw_u p1_s: s knows the aerial vehicle does not know whether
             s is at its first particle: the empathy state
  ruled-out  K_u ¬p1_s: u, found nobody at s's first particle, knows s is not
             tracking it, eq. (11)
  open       ¬K_u ¬p2_s ∧ ¬K_u ¬p3_s: and still holds both later particles
             possible, where eq. (12) believes the next
  last       K_u p3_s: u knows s is at its third particle without going there
  budget     K_u p1_o: and, from the fault budget, that o has had no fault
  told       K_s task, K_o task: who knows of the task
  higher     K_u K_s task: u knows that s knows
  unsure     ¬K_o ¬K_s task: o cannot rule out that s was told
  done       the task is done

It also reports, for every agent, how many worlds it considers possible at the
actual world; none means the agent believes a contradiction, and every
knowledge formula then holds of it vacuously.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_TOOLS = os.environ.get('ER_TOOLS', os.path.join(HERE, '..', '..', '..', '..', 'tools'))
sys.path.insert(0, REPO_TOOLS)
from model_check import extensions, model_of, parse  # noqa: E402
from plan_trace import convert, update  # noqa: E402

AGENTS = ('u', 'g1', 'g2')


def formulas(s, o):
    return {
        'own': f'(or (K {s} p1_{s}) (K {s} p2_{s}) (K {s} p3_{s}))',
        'empathy': f'(K {s} (not (Kw u p1_{s})))',
        'ruled-out': f'(K u (not p1_{s}))',
        'open': f'(and (not (K u (not p2_{s}))) (not (K u (not p3_{s}))))',
        'last': f'(K u p3_{s})',
        'budget': f'(K u p1_{o})',
        f'K_{s} task': f'(K {s} task)',
        f'K_{o} task': f'(K {o} task)',
        'higher': f'(K u (K {s} task))',
        'unsure': f'(not (K {o} (not (K {s} task))))',
        'done': 'done',
    }


def branch_event(action, branch):
    e = branch.get('event')
    return action['designated'][e] if isinstance(e, int) else e


def replay(task, policy, world, table=None):
    """The steps the policy takes from one designated world, with the formulas
    at the actual world after each: those above, or the table given, as
    {name: formula}."""
    model = model_of(task['initial-state'])
    west = 'west_u' in model.labels[world]
    s, o = ('g1', 'g2') if west else ('g2', 'g1')
    table = {k: parse(f) for k, f in (table or formulas(s, o)).items()}
    actual = world

    def look(label):
        ext = {}
        for k, f in table.items():
            ext[k] = actual in extensions(model, f)[f]
        sizes = {a: len(model.image(a, actual)) for a in AGENTS}
        return {'step': label, 'holds': ext, 'considered': sizes, 'worlds': len(model.worlds)}
    steps = [look('initial state')]
    node = policy
    while node is not None:
        action = task['actions'][node['action']]
        chosen = None
        for b in node['branches']:
            e = branch_event(action, b)
            pre = convert(action['preconditions'][e])
            if actual in extensions(model, pre)[pre]:
                chosen = (b, e)
                break
        if chosen is None:
            raise SystemExit(f'{node["action"]}: no branch fits world {world}')
        b, e = chosen
        model, pairs = update(model, action, list(AGENTS), [e])
        actual = next(n for n, (w, ev) in pairs.items() if w == actual and ev == e)
        steps.append(look(f'{node["action"]} / {e}'))
        node = b['subtree']
    return {'world': world, 'particles': {g: next(k for k in (1, 2, 3) if f'p{k}_{g}' in
                                                  model_of(task['initial-state']).labels[world])
                                          for g in ('g1', 'g2')},
            'sought': s, 'other': o, 'steps': steps}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('policy')
    ap.add_argument('--json')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    task, policy = json.load(open(a.task)), json.load(open(a.policy))
    out = [replay(task, policy, w) for w in sorted(task['initial-state']['designated'],
                                                   key=lambda w: int(w[1:]))]
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)
    if not a.quiet:
        keys = list(out[0]['steps'][0]['holds'])
        for r in out:
            print(f"world {r['world']}: g1 at particle {r['particles']['g1']}, g2 at {r['particles']['g2']}; "
                  f"sought first {r['sought']}")
            print('  ' + ' ' * 34 + ' '.join(f'{k[:9]:>9}' for k in keys) + '   considered u/g1/g2')
            for st in r['steps']:
                marks = ' '.join(f'{"yes" if st["holds"][k] else "-":>9}' for k in keys)
                c = st['considered']
                print(f"  {st['step'][:34]:<34}{marks}   {c['u']}/{c['g1']}/{c['g2']}")
    return out


if __name__ == '__main__':
    main()
