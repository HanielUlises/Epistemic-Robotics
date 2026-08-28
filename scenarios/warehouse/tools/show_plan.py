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
Reads an epistemic planning instance out and shows what it is made of: the
pointed model it starts from, the goal, the actions, the plan the planner
returned -- and the state of knowledge after every step of it.

The model, the events and the plan are the tools' own: plank grounds the task,
the planner searches it.  What this adds is the trace, and a trace needs the
product update, so that much is computed here.  It is computed against the
planner rather than instead of it: at every node the action's designated event
must be applicable, and at every leaf the goal must hold.  If this file's
semantics ever drift from the planner's, those checks fail and the trace says
so instead of quietly printing something plausible.

  python3 show_plan.py --task out/problem_1.json --plan out/plan_1.json
"""

import argparse
import json
import sys
import textwrap
import time

RULE = '=' * 74
THIN = '-' * 74
WIDTH = 96

# Seconds to wait between steps, so a screen recording is readable. Zero by
# default: the whole showcase prints in half a second, which is right for a
# terminal and useless for a camera.
PACE = 0.0


def beat(weight=1.0):
    if PACE:
        time.sleep(PACE * weight)


def wrap(text, indent):
    """Long formulas, folded so nothing runs off the side of a recording."""
    return textwrap.fill(text, width=WIDTH, initial_indent=indent,
                         subsequent_indent=indent + '    ',
                         break_long_words=False, break_on_hyphens=False)


# -- formulas ----------------------------------------------------------------
#
# Printed in the surface syntax the EPDDL sources are written in, so that a
# precondition on screen can be found in the domain file by searching for it.

def formula(node):
    if isinstance(node, str):
        return node
    if not isinstance(node, dict):
        return str(node)

    if 'modality-name' in node:
        index = ', '.join(node.get('modality-index', []))
        name = node['modality-name']
        inner = formula(node.get('formula'))
        if name == 'box':
            return f'[{index}] {inner}'
        if name == 'diamond':
            return f'<{index}> {inner}'
        if name == 'Kw.box':
            return f'[Kw. {index}] {inner}'
        if name == 'Kw.diamond':
            return f'<Kw. {index}> {inner}'
        return f'{name}[{index}] {inner}'

    connective = node.get('connective')
    if connective == 'not':
        return f'not {formula(node.get("formula"))}'
    if connective in ('and', 'or'):
        parts = [formula(f) for f in node.get('formulas', [])]
        joiner = f' {connective} '
        return '(' + joiner.join(parts) + ')'
    if 'formula' in node:
        return formula(node['formula'])
    return json.dumps(node)


# -- the model, and what happens to it ---------------------------------------
#
# A pointed Kripke model and a product update with an event model, in the S5
# reading plank exports. Worlds are valuations; an agent's relation says which
# of them it cannot tell apart; the designated worlds are the ones nobody has
# ruled out.

class Model:
    """Worlds, one relation per agent, a valuation each, and the designated set."""

    def __init__(self, worlds, relations, labels, designated):
        self.worlds = list(worlds)
        self.relations = relations        # agent -> world -> [worlds]
        self.labels = {w: set(a) for w, a in labels.items()}
        self.designated = list(designated)

    @staticmethod
    def of(task):
        state = task['initial-state']
        return Model(state['worlds'], state['relations'], state['labels'],
                     state['designated'])

    def sees(self, agent, world):
        """The worlds @p agent cannot tell @p world apart from."""
        return self.relations.get(agent, {}).get(world, [world])

    def holds(self, world, node):
        """Is @p node true at @p world?"""
        if node is None:
            return True
        if isinstance(node, str):
            if node == 'true':
                return True
            if node == 'false':
                return False
            return node in self.labels[world]

        if 'modality-name' in node:
            name = node['modality-name']
            agent = node['modality-index'][0]
            inner = node['formula']
            reach = self.sees(agent, world)
            knows = all(self.holds(v, inner) for v in reach)
            if name == 'box':
                return knows
            if name == 'diamond':
                return any(self.holds(v, inner) for v in reach)
            # "Knowing whether" is knowing which way round it is: either the
            # agent rules out every world where it fails, or every world where
            # it holds. The diamond is the dual -- not knowing whether -- and
            # that is the precondition an inspection carries, because looking
            # at something you have already settled is not an action.
            settled = knows or all(not self.holds(v, inner) for v in reach)
            if name == 'Kw.box':
                return settled
            if name == 'Kw.diamond':
                return not settled
            raise ValueError('unknown modality ' + name)

        connective = node.get('connective')
        if connective == 'not':
            return not self.holds(world, node['formula'])
        if connective == 'and':
            return all(self.holds(world, f) for f in node['formulas'])
        if connective == 'or':
            return any(self.holds(world, f) for f in node['formulas'])
        if 'formula' in node:
            return self.holds(world, node['formula'])
        raise ValueError('unreadable formula ' + json.dumps(node))

    def satisfies(self, node):
        """True when the formula holds at every designated world."""
        return all(self.holds(w, node) for w in self.designated)

    def knowledge(self, agent, atom):
        """What @p agent makes of @p atom, as far as it can tell."""
        answers = set()
        for w in self.designated:
            for v in self.sees(agent, w):
                answers.add(atom in self.labels[v])
        if answers == {True}:
            return 'knows'
        if answers == {False}:
            return 'knows-not'
        return 'unsure'

    def update(self, action):
        """The product update: this model, after @p action.

        A world of the result is a pair -- a world of this model and an event
        of the action whose precondition held there. Two of them are
        indistinguishable to an agent when it could not tell the worlds apart
        *and* cannot tell which event it was. That second half is the whole of
        the difference between a public action and a private one: after a
        public pickup every agent tells the events apart, so the pairing
        splits the worlds it came from; after a semi-private inspection only
        the robot that looked does.
        """
        events = action['events']
        pre = action.get('preconditions', {})
        eff = action.get('effects', {})

        pairs = [(w, e) for w in self.worlds for e in events
                 if self.holds(w, (pre.get(e) or {}).get('formula'))]
        name = {(w, e): f'{w}.{e}' for w, e in pairs}

        labels = {}
        for w, e in pairs:
            value = set(self.labels[w])
            for atom, assignment in (eff.get(e) or {}).items():
                if self.holds(w, assignment.get('formula')):
                    value.add(atom)
                else:
                    value.discard(atom)
            labels[name[(w, e)]] = value

        relations = {}
        for agent in self.relations:
            # Which of the action's observability classes this agent is in.
            # A class it does not appear under at all sees nothing of the
            # action, so it cannot tell any two events apart.
            classes = action.get('observability-conditions', {}).get(agent, {})
            table = {}
            for label in classes:
                table = action.get('relations', {}).get(label, {})
                break
            relations[agent] = {
                name[(w, e)]: [
                    name[(v, f)] for v, f in pairs
                    if v in self.sees(agent, w) and
                    f in table.get(e, events)
                ]
                for w, e in pairs
            }

        designated = [name[(w, e)] for w, e in pairs
                      if w in self.designated and e in action['designated']]
        return Model([name[p] for p in pairs], relations, labels, designated)


# -- the initial situation ---------------------------------------------------

def show_initial_state(task):
    state = task['initial-state']
    worlds = state['worlds']
    designated = set(state['designated'])
    labels = state['labels']

    beat(3.0)
    print(THIN)
    print(' THE SITUATION IT STARTS IN')
    print(THIN)

    # A world is a valuation, and most of what is true in one is true in all of
    # them -- the floor plan, which does not change. What makes these separate
    # worlds is the handful of atoms they disagree about, so that is what is
    # worth printing.
    shared = set.intersection(*(set(labels[w]) for w in worlds))
    varying = set.union(*(set(labels[w]) for w in worlds)) - shared

    print(f'{len(worlds)} world(s), {len(designated)} of them designated -- '
          'a designated world is one')
    print('the fleet has not ruled out, so the situation is "any of these, '
          'for all we know".')
    print()
    for world in sorted(worlds):
        mark = '*' if world in designated else ' '
        differs = sorted(a for a in labels[world] if a in varying)
        print(f'  {mark} {world}   ' + ('  '.join(differs) if differs
                                        else '(agrees with every other world)'))
    if varying:
        print()
        print(f'  {len(shared)} atom(s) hold in every world and are left out '
              'above: the building,')
        print('  which no action changes and nobody is in doubt about.')

    print()
    print('  what each agent can tell apart')
    for agent, relation in sorted(state['relations'].items()):
        classes = []
        for world in sorted(worlds):
            reachable = frozenset(relation.get(world, [world]))
            if reachable not in classes:
                classes.append(reachable)
        rendered = '  '.join('{' + ', '.join(sorted(c)) + '}' for c in classes)
        if all(len(c) == 1 for c in classes):
            note = 'every world on its own: this agent knows which it is in'
        elif len(classes) == 1:
            note = 'one cell: this agent cannot tell these worlds apart'
        else:
            note = 'partly settled'
        print(f'    {agent}: {rendered}')
        print(f'         {note}')


def show_goal(task):
    print()
    beat(3.0)
    print(THIN)
    print(' WHAT IS ASKED')
    print(THIN)
    print('  ' + formula(task['goal']['formula']))
    print()
    print('  A conjunct under [Kw. i] is a goal about an agent rather than')
    print('  about the warehouse. No amount of moving the pallet satisfies it;')
    print('  only coming to know does, which is why a plan has to sense.')


def show_actions(task, used):
    print()
    beat(3.0)
    print(THIN)
    print(' THE ACTIONS THE PLAN USES')
    print(THIN)
    actions = task['actions']
    by_type = {}
    for name in sorted(used):
        if name in actions:
            by_type.setdefault(actions[name]['action-type'], []).append(name)

    for action_type in sorted(by_type):
        print(f'  {action_type}')
        for name in by_type[action_type]:
            spec = actions[name]
            print(f'    {name}')
            for event in spec['events']:
                precondition = spec['preconditions'].get(event, {})
                effects = spec.get('effects', {}).get(event) or {}
                print(f'      {event}')
                if precondition:
                    print(wrap('when: ' + formula(precondition.get('formula')),
                               '        '))
                if effects:
                    changes = ', '.join(
                        f'{atom} := {formula(value.get("formula"))}'
                        for atom, value in sorted(effects.items()))
                    print(wrap('then: ' + changes, '        '))
        print()


# -- the plan ----------------------------------------------------------------

def walk(node, actions, indent='  ', label=None, leaves=None, prefix=None):
    """Prints the plan tree and collects each root-to-leaf action sequence."""
    prefix = list(prefix or [])
    leaves = leaves if leaves is not None else []

    if label is not None:
        print(f'{indent}+-- {label}')
        indent += '    '

    name = node.get('action')
    spec = actions.get(name, {})
    kind = spec.get('action-type', '?')
    print(f'{indent}{name}   [{kind}]')
    prefix.append(name)

    branches = [b for b in (node.get('branches') or []) if b.get('subtree')]
    if not branches:
        leaves.append(list(prefix))
        print(f'{indent}=> goal reached')
        return leaves

    if len(branches) == 1:
        walk(branches[0]['subtree'], actions, indent, None, leaves, prefix)
        return leaves

    # A real branch: the plan splits because the action it followed could
    # have gone more than one way, and which way is a fact about the
    # warehouse rather than about the plan. Each branch is named by the
    # event that produced it, and the event's precondition says what was
    # the case for it to have been that one.
    events = spec.get('events', [])
    for branch in branches:
        index = branch.get('event')
        event = events[index] if isinstance(index, int) and index < len(events) \
            else str(index)
        condition = spec.get('preconditions', {}).get(event, {}).get('formula')
        told = distinguishing(spec, event)
        label = f'{event}' + (f'   ({told})' if told else '')
        walk(branch['subtree'], actions, indent + '  ', label, leaves, prefix)
    return leaves


def distinguishing(spec, event):
    """The part of this event's precondition the other events contradict.

    An action with several events is an action that could turn out several
    ways; what separates them is the handful of literals they disagree on, and
    printing those instead of the whole conjunction is the difference between
    a readable branch label and a wall of text.
    """
    def literals(name):
        node = spec.get('preconditions', {}).get(name, {}).get('formula')
        if isinstance(node, dict) and node.get('connective') == 'and':
            return {formula(f) for f in node.get('formulas', [])}
        return {formula(node)} if node else set()

    mine = literals(event)
    others = set()
    for other in spec.get('events', []):
        if other != event:
            others |= literals(other)
    only_mine = sorted(mine - others)
    return ' and '.join(only_mine)


def show_plan(task, plan):
    print()
    beat(3.0)
    print(THIN)
    print(' THE PLAN')
    print(THIN)
    leaves = walk(plan, task['actions'])
    print()
    print(f'  {len(leaves)} leaf/leaves. A plan with more than one leaf is not '
          'a sequence:')
    print('  it is a policy, and the branch taken is decided by what the robot')
    print('  finds rather than by the planner.')
    return leaves


# -- the trace ----------------------------------------------------------------

WORDS = {'knows': 'knows it is there',
         'knows-not': 'knows it is not',
         'unsure': 'does not know whether'}


def questions(model):
    """The atoms the designated worlds disagree about: the open questions."""
    if len(model.designated) < 2:
        first = model.designated[0]
        pool = {v for w in model.designated for a in model.relations
                for v in model.sees(a, w)}
    else:
        pool = set(model.designated)
    seen = [model.labels[w] for w in pool]
    if not seen:
        return []
    return sorted(set.union(*seen) - set.intersection(*seen))


def show_knowledge(model, atoms, indent):
    for agent in sorted(model.relations):
        for atom in atoms:
            state = model.knowledge(agent, atom)
            mark = ' ' if state == 'unsure' else '*'
            print(f'{indent}{mark} {agent} {WORDS[state]}: {atom}')


def trace(node, task, model, atoms, indent='  ', label=None, problems=None,
          settled_by=None):
    """Walks the policy, updating the model, printing what each agent knows."""
    problems = problems if problems is not None else []
    settled_by = settled_by if settled_by is not None else {}
    actions = task['actions']

    if label is not None:
        print(f'{indent}+-- {label}')
        indent += '    '

    name = node.get('action')
    action = actions.get(name)
    if action is None:
        problems.append(f'{name}: not in the grounded task')
        return problems

    # The planner said this action applies here. If it does not apply in this
    # trace, the trace is wrong -- so say so rather than print a state nobody
    # should believe.
    #
    # Applicable means every designated world satisfies *some* designated
    # event, not every one of them. An inspection has two designated events
    # whose preconditions are contradictory -- the bay holds the pallet, or it
    # does not -- and no world satisfies both. Requiring both is how you
    # "prove" that looking into a bay is never allowed.
    pre = action.get('preconditions', {})
    for world in model.designated:
        if not any(model.holds(world, (pre.get(event) or {}).get('formula'))
                   for event in action['designated']):
            problems.append(
                f'{name}: no designated event applies at {world}, but the '
                'planner used it')

    after = model.update(action)
    kind = action['action-type']
    print(f'{indent}{name}   [{kind}]')
    note = ''
    if len(action['designated']) > 1:
        # A sensing action does not settle anything by being taken. It splits
        # the situation in two and the world decides which half is real, so the
        # designated set only shrinks at the branch below.
        note = '   (both outcomes still open; they split at the branch)'
    print(f'{indent}   {len(after.worlds)} world(s), '
          f'{len(after.designated)} designated' + note)
    show_knowledge(after, atoms, indent + '   ')
    beat()

    # Who this action settled the question for, if anyone.
    for agent in sorted(after.relations):
        was = {model.knowledge(agent, a) for a in atoms}
        now = {after.knowledge(agent, a) for a in atoms}
        if 'unsure' in was and 'unsure' not in now:
            print(f'{indent}   ^ this is where {agent} came to know, '
                  f'from a {kind} action')
            if settled_by.get(agent) is None:
                settled_by[agent] = name

    branches = [b for b in (node.get('branches') or []) if b.get('subtree')]
    if not branches:
        if after.satisfies(task['goal']['formula']):
            print(f'{indent}   => the goal holds')
        else:
            problems.append(f'{name}: the planner ends here, but the goal '
                            'does not hold in this trace')
            print(f'{indent}   => THE GOAL DOES NOT HOLD (see the warning below)')
        return problems

    if len(branches) == 1:
        return trace(branches[0]['subtree'], task, after, atoms, indent, None,
                     problems, settled_by)

    events = action.get('events', [])
    for branch in branches:
        index = branch.get('event')
        event = events[index] if isinstance(index, int) and index < len(events) \
            else str(index)
        told = distinguishing(action, event)
        # Each branch continues from the model this action produced, restricted
        # to the worlds where that event is the one that happened.
        restricted = Model(after.worlds, after.relations, after.labels,
                           [w for w in after.designated
                            if w.endswith('.' + event)])
        if not restricted.designated:
            problems.append(f'{name}: branch {event} has no world to stand in')
            restricted = after

        # The branch is the observation itself: not an action the robot takes
        # but the answer it gets. For whoever was watching closely enough,
        # this is the moment the question stops being open.
        learned = [a for a in sorted(after.relations)
                   if 'unsure' in {after.knowledge(a, x) for x in atoms} and
                   'unsure' not in {restricted.knowledge(a, x) for x in atoms}]
        inner = indent + '  '
        beat(2.0)
        print(f'{inner}+-- {event}' + (f'   ({told})' if told else ''))
        if learned:
            print(f'{inner}    ^ seeing this is where '
                  f'{", ".join(learned)} came to know')
        trace(branch['subtree'], task, restricted, atoms, inner + '    ',
              None, problems, dict(settled_by))
    return problems


def show_trace(task, plan):
    print()
    beat(3.0)
    print(THIN)
    print(' THE SITUATION, STEP BY STEP')
    print(THIN)
    print('  The model after each action, by product update from the one plank')
    print('  exported and the events it ground. A line marked * is an agent')
    print('  that has the question settled; a blank one is an agent that does')
    print('  not.')
    print()

    model = Model.of(task)
    atoms = questions(model)
    print('  start')
    print(f'     {len(model.worlds)} world(s), {len(model.designated)} designated')
    show_knowledge(model, atoms, '     ')
    print()

    problems = trace(plan, task, model, atoms)

    print()
    if problems:
        print('  !! this trace disagrees with the planner:')
        for problem in problems:
            print(f'     - {problem}')
        print('     The plan is the planner\'s; the trace is this file\'s, and')
        print('     where they differ it is the trace that is wrong. Do not')
        print('     read the states above as the semantics.')
    else:
        print('  Every action the planner used applies in this trace, and the')
        print('  goal holds at every leaf: the states above agree with the')
        print('  planner that produced the plan.')
    return problems


# -- main --------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', required=True)
    parser.add_argument('--plan', required=True)
    parser.add_argument('--title', default=None)
    parser.add_argument('--pace', type=float, default=0.0,
                        help='seconds between steps, for screen recording')
    args = parser.parse_args()

    global PACE
    PACE = args.pace

    task = json.load(open(args.task))
    plan = json.load(open(args.plan))
    info = task.get('planning-task-info', {})

    print()
    print(RULE)
    print(' ' + (args.title or info.get('problem', args.task)))
    print(RULE)
    print(f"  domain        {info.get('domain')}")
    print(f"  agents        {info.get('agents-number')}")
    print(f"  atoms         {info.get('atoms-number')}"
          f"   facts {info.get('facts-number')}")
    print(f"  ground actions {info.get('actions-number')}")
    print(f"  initial worlds {info.get('initial-worlds-number')}")
    print(f"  goal modal depth {info.get('goal-modal-depth')}")
    print()

    show_initial_state(task)
    show_goal(task)

    leaves = show_plan(task, plan)
    problems = show_trace(task, plan)
    used = {name for leaf in leaves for name in leaf}
    show_actions(task, used)

    print(THIN)
    print(' EACH LEAF, AS A SEQUENCE')
    print(THIN)
    print('  Hand any of these to `plank validate -a ...` to check it against')
    print('  the domain; the branch that does not apply is the point of the')
    print('  policy.')
    for i, leaf in enumerate(leaves, 1):
        print(wrap(f'{i}. ' + ' '.join(leaf), '  '))
    print()
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
