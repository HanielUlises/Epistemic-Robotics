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
Replays an executed run against the epistemic state and measures it.

The run log records the size of the model after each product update and
nothing else. The quantities that characterise the run are per agent: how many
worlds each one considers possible, whether its perspective still contains the
actual world, and which formulas hold for it. Those are available from
`get_agent_perspective` and `check_formula`, so this drives a standalone
epistemic state through the same action sequence and records them at every
step.

    analyse_run.py --task out/problem_2.json \\
                   --log out/hotel-l3_suite.log \\
                   --agents inspector porter guest \\
                   --formula 'safe' --formula '(leak-at l2_suite)' \\
                   --out out/analysis.json

The action sequence is read from the log, so what is measured is the run that
happened rather than the plan that was intended.
"""

import argparse
import json
import re
import subprocess

APPLIED = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*epistemic_state\] applied (?P<action>\S+?)'
    r'(?: -> (?P<outcome>\S+?))?: (?P<worlds>\d+) worlds, (?P<designated>\d+) designated')


def call(service, kind, request, timeout=25):
    """One `ros2 service call`, parsed out of its printed response."""
    out = subprocess.run(
        ['ros2', 'service', 'call', service, kind, request],
        capture_output=True, text=True, timeout=timeout).stdout
    body = out.split('response:', 1)[-1]
    fields = {}
    for key, pattern in (('success', r"success=(True|False)"),
                         ('holds', r"holds=(True|False)"),
                         ('includes_actual', r"includes_actual=(True|False)"),
                         ('worlds', r"worlds=(\d+)"),
                         ('designated', r"designated=(\d+)"),
                         ('model', r"model='(.*?)', includes_actual"),
                         ('error', r"error='([^']*)'")):
        m = re.search(pattern, body, re.S)
        if m:
            value = m.group(1)
            fields[key] = (value == 'True') if value in ('True', 'False') \
                else (int(value) if value.isdigit() else value)
    return fields


def trace_from_log(path):
    steps = []
    with open(path, errors='replace') as handle:
        for line in handle:
            m = APPLIED.search(line)
            if m:
                steps.append({
                    'stamp': float(m.group('t')),
                    'action': m.group('action'),
                    'outcome': m.group('outcome') or '',
                    'worlds': int(m.group('worlds')),
                    'designated': int(m.group('designated')),
                })
    return steps


def ask(formula, agent=''):
    answer = call('/epistemic_state/check_formula',
                  'plansys2_epistemic_msgs/srv/CheckFormula',
                  f'{{formula: "{formula}", agent: "{agent}"}}')
    if not answer.get('success'):
        raise SystemExit(f"{formula!r} did not evaluate: {answer.get('error')}")
    return bool(answer.get('holds'))


def structure(model_json):
    """Graph properties of one agent's accessibility relation.

    The relation is a digraph on the worlds. Two properties characterise the
    run. Reflexivity distinguishes knowledge from belief: an agent whose
    relation is reflexive everywhere cannot be wrong, which is S5, and one
    whose relation drops a world's self-loop can be, which is KD45. The number
    of strongly connected components the agent's arcs induce is the number of
    distinct information states it can be in.
    """
    try:
        model = json.loads(model_json)
    except (TypeError, ValueError):
        return {}

    worlds = model.get('worlds', [])
    designated = set(model.get('designated', []))
    result = {'worlds': worlds, 'designated': sorted(designated),
              'relations': model.get('relations', {}),
              'labels': model.get('labels', {})}

    varying = sorted({
        atom
        for atom in set().union(*[set(v) for v in model.get('labels', {}).values()] or [set()])
        if len({atom in set(v) for v in model.get('labels', {}).values()}) > 1
    })
    result['varying_atoms'] = varying
    return result


def reflexive(relation, worlds):
    """True when every world can see itself, which is what S5 requires."""
    return all(w in relation.get(w, []) for w in worlds)


def measure(agents, formulas, model_formulas):
    """The model as each agent sees it, and what holds there."""
    snapshot = {'model': {f: ask(f) for f in model_formulas}}
    for agent in agents:
        view = call('/epistemic_state/get_agent_perspective',
                    'plansys2_epistemic_msgs/srv/GetAgentPerspective',
                    f"{{agent: '{agent}'}}")
        graph = structure(view.get('model'))
        entry = {
            'worlds': view.get('worlds'),
            'designated': view.get('designated'),
            'structure': graph,
            'reflexive': (reflexive(graph.get('relations', {}).get(agent, {}),
                                    graph.get('worlds', []))
                          if graph else None),
            # False here is the signature of a belief the world contradicts,
            # which S5 cannot represent and KD45 can.
            'includes_actual': view.get('includes_actual'),
            'formulas': {},
        }
        for formula in formulas:
            entry['formulas'][formula] = ask(formula, agent)
        snapshot[agent] = entry
    return snapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', required=True, help='grounded task JSON')
    parser.add_argument('--log', required=True, help='the run to replay')
    parser.add_argument('--agents', nargs='+', required=True)
    parser.add_argument('--formula', action='append', default=[],
                        help='repeatable; evaluated in every agent\'s own '
                             'perspective at every step. Atoms are bare tokens: '
                             '`safe`, `leak-at_l3_suite`.')
    parser.add_argument('--model-formula', action='append', default=[],
                        help='repeatable; evaluated in the model itself, which '
                             'is where a goal such as `(K porter safe)` is '
                             'checked')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    steps = trace_from_log(args.log)
    if not steps:
        raise SystemExit(f'no applied actions in {args.log}')
    print(f'{len(steps)} actions to replay')

    loaded = call('/epistemic_state/load_task',
                  'plansys2_epistemic_msgs/srv/LoadTask',
                  f"{{task_file: '{args.task}'}}")
    if not loaded.get('success'):
        raise SystemExit(f"could not load the task: {loaded.get('error')}")

    record = {'task': args.task, 'log': args.log, 'steps': []}
    record['formulas'] = {'agent': args.formula, 'model': args.model_formula}
    record['initial'] = measure(args.agents, args.formula, args.model_formula)

    start = steps[0]['stamp']
    for index, step in enumerate(steps):
        applied = call('/epistemic_state/apply_action',
                       'plansys2_epistemic_msgs/srv/ApplyAction',
                       f"{{epistemic_action: '{step['action']}', "
                       f"observed_outcome: '{step['outcome']}'}}")
        if not applied.get('success'):
            raise SystemExit(
                f"step {index} ({step['action']}) was refused: {applied.get('error')}")

        record['steps'].append({
            **step,
            'at': round(step['stamp'] - start, 2),
            'agents': measure(args.agents, args.formula, args.model_formula),
        })
        print(f"  {index + 1}/{len(steps)}  {step['action']}")

    with open(args.out, 'w') as handle:
        json.dump(record, handle, indent=2)
    print(f'-> {args.out}')


if __name__ == '__main__':
    main()
