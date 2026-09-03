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
Writes the action map that joins the epistemic vocabulary to the PlanSys2 one.

    make_mapping.py --task out/problem_2.json --objects lobby l2_suite l3_suite \
                    --translate pddl/translation.json --out pddl/mapping.json

plank grounds an action into a single token by joining the action name and its
arguments with underscores, and object names contain underscores too:
`deploy_inspector_porter_lobby_l2_suite_lobby_l3_suite`. Splitting that on
underscores is ambiguous, and getting it wrong is quiet -- the executor is
handed `(deploy inspector porter lobby l2 suite lobby_l3_suite)`, reports that
a precondition does not hold, and retries forever. So the names are read
against the vocabularies the task itself declares, longest match first.
"""

import argparse
import json
import re


def vocabularies(task, objects):
    """The names this task grounds over, longest first so `l2_suite` wins.

    A plank export declares its agents and not its objects, so the zone names
    are given on the command line. Guessing them out of the atom names would
    work until a domain had an atom whose predicate ended in a zone name.
    """
    agents = [a['name'] if isinstance(a, dict) else a
              for a in task.get('language', {}).get('agents', [])]
    return sorted(set(agents) | set(objects), key=len, reverse=True)


def parse(name, vocabulary):
    """`deploy_inspector_porter_lobby_l2_suite` -> ('deploy', [...])."""
    head, _, rest = name.partition('_')
    arguments = []
    while rest:
        for candidate in vocabulary:
            if rest == candidate:
                arguments.append(candidate)
                rest = ''
                break
            if rest.startswith(candidate + '_'):
                arguments.append(candidate)
                rest = rest[len(candidate) + 1:]
                break
        else:
            return head, None      # a token this task never declared
    return head, arguments


# Epistemic action -> the PlanSys2 action it is executed as, how long the
# executor should allow for it, and how many arguments it takes. Read from a
# file, because the two demos in this workspace speak different domains and
# this tool has no business knowing either.
#
#   {"go": {"action": "goto_zone", "duration": 180.0, "arity": 3}, ...}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', required=True, help='plank export JSON')
    parser.add_argument('--objects', nargs='+', required=True,
                        help='the object names the task grounds over')
    parser.add_argument('--translate', required=True,
                        help='JSON saying which PlanSys2 action each epistemic '
                             'action is executed as')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    task = json.load(open(args.task))
    declared = json.load(open(args.translate))
    translation = {
        name: (entry['action'], float(entry['duration']), int(entry['arity']))
        for name, entry in declared.items()
    }
    translation_padding = {
        name: list(entry.get('pad', [])) for name, entry in declared.items()
    }
    vocabulary = vocabularies(task, args.objects)
    if not vocabulary:
        raise SystemExit('the task declares no agents or objects to parse against')

    names = [a['name'] if isinstance(a, dict) else a for a in task['actions']]

    mapping = {}
    unparsed = []
    trivial = 0
    for name in names:
        head, arguments = parse(name, vocabulary)
        if head not in translation:
            unparsed.append(name)
            continue
        target, duration, arity = translation[head]
        if arguments is None or len(arguments) != arity:
            unparsed.append(name)
            continue

        # A grounding whose precondition can never hold: moving where you are,
        # briefing yourself, sending one robot to two places. It is grounded
        # because the lifted action allows it and it can never be dispatched,
        # so it needs no performer.
        if head in ('go',) and arguments[1] == arguments[2]:
            trivial += 1
            continue
        if head == 'deploy':
            i, j, from_i, to_i, from_j, to_j = arguments
            if i == j or to_i == to_j or from_i == to_i or from_j == to_j:
                trivial += 1
                continue
        if head.startswith('brief') and arguments[0] == arguments[1]:
            trivial += 1
            continue

        # An epistemic action may take fewer arguments than the PlanSys2 one
        # it runs as. `page-safe` names no suite and `page` takes one, so the
        # translation may pad with a fixed tail.
        padding = translation_padding.get(head, [])
        expression = f'({target} ' + ' '.join(arguments + padding) + ')'

        mapping[name] = {'action': expression, 'duration': duration}

    with open(args.out, 'w') as handle:
        json.dump(dict(sorted(mapping.items())), handle, indent=2)
        handle.write('\n')

    print(f'{len(mapping)} mapped, {trivial} never dispatchable, '
          f'{len(unparsed)} unrecognised -> {args.out}')
    if unparsed:
        raise SystemExit(
            'these ground actions were not understood, and an unmapped action '
            'is a policy the executor will refuse:\n  ' +
            '\n  '.join(unparsed[:10]))


if __name__ == '__main__':
    main()
