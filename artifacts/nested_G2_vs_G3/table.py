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

"""The three goals side by side, read out of what plank and ALETHEIA wrote.

Nothing here is typed in. The modal depth is the field plank puts in the
ground task; the expansions are the line ALETHEIA's search prints; whether a
policy announces is whether an announcement appears in the policy tree. A
table assembled by hand would be a claim about the run; this one is the run.
"""

import json
import re
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
GOALS = ['g1', 'g2', 'g3']
WRITTEN = {
    'g1': '[Kw r1] P',
    'g2': '[r2] [Kw r1] P',
    'g3': '[Kw r2] P',
}


def walk(node):
    """Every action in a policy, and how many leaves it has."""
    if node is None:
        return [], 1
    actions, leaves = [node['action']], 0
    for branch in node.get('branches', []):
        below, count = walk(branch.get('subtree'))
        actions += below
        leaves += count
    return actions, max(leaves, 1)


def row(goal):
    task = json.load(open(os.path.join(HERE, 'problems', 'nested_%s.json' % goal)))
    info = task['planning-task-info']
    plan = json.load(open(os.path.join(HERE, 'policies', 'nested_%s-plan.json' % goal)))
    log = open(os.path.join(HERE, 'logs', 'nested_%s-search.log' % goal)).read()

    found = re.search(r'depth (\d+)\s+Expanded=(\d+)\s+Generated=(\d+)', log)
    depth, expanded, _generated = found.groups() if found else ('?', '?', '?')
    validated = re.search(r'OK — (\d+) leaves, (\d+) branches', log)

    actions, _leaves = walk(plan)
    announces = [a for a in actions if a.startswith('report-')]
    public = [a for a in actions if a.startswith('pickup') or a.startswith('report-')]

    return {
        'goal': goal.upper(),
        'formula': WRITTEN[goal],
        'modal depth': info['goal-modal-depth'],
        'ground actions': info['actions-number'],
        'worlds': info['initial-worlds-number'],
        'plan depth': depth,
        'expanded': expanded,
        'policy actions': len(actions),
        'leaves': validated.group(1) if validated else '?',
        'branches': validated.group(2) if validated else '?',
        'announce?': 'yes' if announces else 'no',
        'inspect alone?': 'no' if public else 'yes',
    }


def main():
    rows = [row(goal) for goal in GOALS]
    head = list(rows[0])
    width = [max(len(h), *(len(str(r[h])) for r in rows)) for h in head]
    line = lambda cells: '  ' + '  '.join(
        str(c).ljust(width[i]) for i, c in enumerate(cells))
    print(line(head))
    print('  ' + '  '.join('-' * w for w in width))
    for r in rows:
        print(line([r[h] for h in head]))
    if '--json' in sys.argv:
        json.dump(rows, open(os.path.join(HERE, 'table.json'), 'w'), indent=1)


main()
