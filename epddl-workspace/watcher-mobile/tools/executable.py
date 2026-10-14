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
Can the agents executing a policy follow its branches?

    executable.py TASK.json PLAN.json

A policy branches after an action on the event that occurred. The agent that
acts can take the branch only if it can tell the events apart. This finds, for
every node with more than one branch, the action's actor -- the first agent
its grounded name gives after the schema, the convention of every domain
written for ePlanSys -- and the observability type that actor has of the
action. Where that type is unconditional, the node is followable iff no two of
its branch events are related by the type's relation; where it depends on the
world, the node is reported as not checked. Exits non-zero if any node cannot
be followed.
"""

import json
import sys


def actor(name, agents):
    for part in name.split('_')[1:]:
        if part in agents:
            return part
    return None


def main():
    task, plan = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    agents = task['language']['agents']
    bad, unchecked, nodes = [], [], 0

    def walk(node):
        nonlocal nodes
        if node is None:
            return
        nodes += 1
        a = task['actions'][node['action']]
        if len(node['branches']) > 1:
            who = actor(node['action'], agents)
            cases = (a.get('observability-conditions') or {}).get(who)
            kinds = [k for k, c in (cases or {}).items()]
            if cases is None:
                kind = 'Fully'
            elif len(kinds) == 1 and cases[kinds[0]].get('formula') == 'true':
                kind = kinds[0]
            else:
                kind = None
            if kind is None:
                unchecked.append(node['action'])
            else:
                rel = a['relations'][kind]
                evs = [a['designated'][b['event']] for b in node['branches']]
                for i, e in enumerate(evs):
                    for f in evs[i + 1:]:
                        if f in rel.get(e, []):
                            bad.append(f'{node["action"]}: its actor {who} cannot tell {e} from {f}')
        for b in node['branches']:
            walk(b['subtree'])
    walk(plan)
    for b in sorted(set(bad)):
        print('NOT FOLLOWABLE', b)
    for u in sorted(set(unchecked)):
        print('not checked (conditional observability for the actor):', u)
    print(f'{nodes} nodes; {len(set(bad))} branch points the actor cannot follow')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
