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
Writes the action mapping the executor joins the two halves of the demo with.

The mapping says that `inspect_r1_bay2` on the epistemic side is
`(look_into r1 bay2)` on the classical one. It used to be typed out by hand,
which was fine while there was one robot and fifteen entries and stopped being
fine at three, where there are ninety-six -- and where a missing entry does not
fail at build time or at plan time but halfway through a run, as an action the
executor cannot dispatch.

plank already knows every ground action, because it made them. So the mapping
is generated from the grounding and then *checked against it*: every name this
writes has to be a name plank ground, and every ground action the policy could
plausibly use has to come out with a mapping. A name that appears in one and
not the other is the bug this file exists to make impossible, and it is an
error here rather than a stall in Gazebo.

  python3 make_mapping.py --task ../../../epddl-workspace/robot-warehouse/out/problem_3.json

Writes ros2_ws/src/warehouse_demo/pddl/warehouse-mapping.json, which is checked
in: nothing in the build runs this, exactly as nothing in the build runs the
rasteriser. Run it again when the instance gains a robot.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEFAULT_TASK = os.path.join(
    ROOT, 'epddl-workspace', 'robot-warehouse', 'out', 'problem_3.json')
DEFAULT_OUT = os.path.join(
    ROOT, 'ros2_ws', 'src', 'warehouse_demo', 'pddl', 'warehouse-mapping.json')

# The durations are the classical domain's, and have to agree with it: the
# executor takes the plan item's duration from the PDDL and this is what tells
# the epistemic side how long to expect the action to take.
#
# A drive is 120 s because crossing this warehouse takes over a minute and the
# route is not known when the plan is made. A look is 30 s because that is how
# long perception may spend re-offering a reading the state is not ready for.
DURATION = {
    'go': 120.0,
    'inspect': 30.0,
    'pickup': 6.0,
    'unload': 6.0,
    'report-pallet-at': 4.0,
    'report-pallet-not-at': 4.0,
}

# What each epistemic verb is, as something a robot does.
CLASSICAL = {
    'go': 'goto_zone',
    'inspect': 'look_into',
    'pickup': 'pick_up',
    'unload': 'drop_off',
    'report-pallet-at': 'announce',
    'report-pallet-not-at': 'announce',
}


def split(name, agents, zones):
    """`go_r1_dock_south_lane` -> ('go', ['r1', 'dock_south', 'lane']).

    Splitting on underscores does not work: half the zones have one in the
    middle of them. So the objects are matched against the ones the task
    actually declares, longest first, which is unambiguous because no object
    name is a prefix of another at an underscore boundary.
    """
    objects = sorted(list(agents) + list(zones), key=len, reverse=True)
    for verb in sorted(CLASSICAL, key=len, reverse=True):
        if not name.startswith(verb + '_'):
            continue
        rest, args = name[len(verb) + 1:], []
        while rest:
            for obj in objects:
                if rest == obj:
                    args.append(obj)
                    rest = ''
                    break
                if rest.startswith(obj + '_'):
                    args.append(obj)
                    rest = rest[len(obj) + 1:]
                    break
            else:
                return None
        return verb, args
    return None


def policy_actions(path):
    """Every action named anywhere in a policy, down both sides of a branch."""
    found = set()

    def walk(node):
        if isinstance(node, list):
            for child in node:
                walk(child)
        elif isinstance(node, dict):
            name = node.get('action')
            if isinstance(name, str) and name:
                found.add(name)
            walk(node.get('branches'))
            walk(node.get('subtree'))

    with open(path) as handle:
        walk(json.load(handle))
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default=DEFAULT_TASK,
                        help='a plank grounding (plank export -o out)')
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--check-plan', nargs='*', default=None,
                        help='policies every action of which must be mapped; '
                             'defaults to the three beside the task')
    args = parser.parse_args()

    with open(args.task) as handle:
        task = json.load(handle)

    ground = task['actions']

    # The objects, taken from the task rather than guessed. The agents are
    # listed outright. The zones are not, but `bay` is a rigid fact over every
    # zone -- true of two of them and false of the rest, and ground for all --
    # so the atoms it was ground into name them exactly, underscores and all.
    agents = set(task['language']['agents'])
    zones = {atom[len('bay_'):]
             for atom in task['language']['atoms'] if atom.startswith('bay_')}
    if not agents or not zones:
        sys.exit('no agents or zones in {}: cannot name the mapping'
                 .format(args.task))

    mapping, skipped = {}, []
    for name in sorted(ground):
        parsed = split(name, agents, zones)
        if parsed is None:
            sys.exit('cannot read the objects out of ground action ' + name)
        verb, objects = parsed

        # A `go` from a zone to itself is ground because grounding is
        # mechanical, and is never in a plan because its precondition is a
        # contradiction. Mapping it would be harmless; leaving it out keeps the
        # file readable and makes a mapping that *is* used easier to find.
        if verb == 'go' and objects[1] == objects[2]:
            skipped.append(name)
            continue

        # An announcement is only ever about the pallet, which is only ever in
        # a bay. The classical `announce` asks for `(is_bay ?z)`, so mapping
        # the others would produce entries that could not be dispatched.
        if verb.startswith('report-') and not objects[-1].startswith('bay'):
            skipped.append(name)
            continue

        mapping[name] = {
            'action': '({} {})'.format(CLASSICAL[verb], ' '.join(objects)),
            'duration': DURATION[verb],
        }

    # The check this file exists for: every name written is a name plank
    # ground, and nothing that was ground has been silently dropped except the
    # two kinds above.
    unknown = sorted(set(mapping) - set(ground))
    if unknown:
        sys.exit('wrote actions that are not in the grounding: ' + str(unknown))
    missing = sorted(set(ground) - set(mapping) - set(skipped))
    if missing:
        sys.exit('ground actions with no mapping: ' + str(missing))

    # And the check that matters more than either of those, because it is the
    # one a run would otherwise discover: every action the planner actually
    # put in a policy has somewhere to be dispatched to. A grounding is large
    # and mostly unused; a policy is what will be executed.
    plans = args.check_plan
    if plans is None:
        beside = os.path.dirname(args.task)
        plans = [os.path.join(beside, name) for name in sorted(os.listdir(beside))
                 if name.endswith('-plan.json')]
    for plan in plans:
        unmapped = sorted(a for a in policy_actions(plan) if a not in mapping)
        if unmapped:
            sys.exit('{} uses actions with no mapping: {}'
                     .format(os.path.basename(plan), unmapped))

    with open(args.out, 'w') as handle:
        json.dump(mapping, handle, indent=2, sort_keys=True)
        handle.write('\n')

    print('{} ground actions -> {} mappings ({} skipped) -> {}'
          .format(len(ground), len(mapping), len(skipped), args.out))
    for agent in sorted(agents):
        count = sum(1 for k in mapping if '_' + agent + '_' in k + '_')
        print('  {}: {} actions'.format(agent, count))
    for plan in plans:
        print('  every action of {} is mapped'.format(os.path.basename(plan)))


if __name__ == '__main__':
    main()
