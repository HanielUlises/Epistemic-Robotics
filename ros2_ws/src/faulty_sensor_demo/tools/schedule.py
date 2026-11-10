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
The executor's schedule of a policy, as ePlanSys 0.3 makes it.

    schedule.py --act 2 --formulation sensors [--night AB-s]

ePlanSys does not dispatch a policy as written. Its scheduler
(plansys2_epistemic_executor/src/policy_schedule.cpp) moves an action above
its parent when the two are independent and the action could start sooner,
above a branch only when every branch begins with it, and then starts chains
of pairwise independent actions together, a group ending when its longest
member ends. Independence is classical, from the vocabulary's conditions and
effects, and epistemic, from the names the actions carry: two actions are
independent only if they name no name in common, places and passages
included, and neither's knowledge requirement mentions a name the other
carries. A robot's places carry its name (tools/make_pddl.py), so the drives
of two robots along the hall are independent.

This is that scheduler, ported, over the vocabulary tools/make_pddl.py writes,
and the time of a policy on a night when its groups are run as the executor
runs them. tools/plan.py plans on groups of its own making, one action a
robot and a reading last; this is what ePlanSys makes of the policy it is
handed.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
_MAPPING = {}


def mapping(act):
    if act not in _MAPPING:
        _MAPPING[act] = json.load(open(os.path.join(HERE, '..', 'pddl', f'fleet-mapping-act{act}.json')))
    return _MAPPING[act]


class Item:
    """One policy node: what ePlanSys knows of it."""

    def __init__(self, robot, action, formulation, act):
        self.robot, self.a = robot, action
        self.name = P.grounded(robot, action, formulation)
        self.vocab = mapping(act)[self.name]['action']
        self.duration = mapping(act)[self.name]['duration']
        words = self.vocab.strip('()').split()
        self.kind = words[0]
        self.args = words[1:]
        names = set(self.args)
        names.update(p for p in self.name.split('_')[1:] if p)
        self.names = names
        # the robots a knowledge requirement mentions: the actor's own modalities
        self.requires = {robot} if action[0] in ('enter', 'cross', 'say') else set()

    def same(self, other):
        return self.name == other.name


def classical(a, b):
    """Independent in the vocabulary: no passage both hold, and no report among them
    (a report writes (radio), which every action reads)."""
    if a.kind == 'say' or b.kind == 'say':
        return False
    held = {x for x in a.args if x.startswith('pass_')} & {x for x in b.args if x.startswith('pass_')}
    return not held


def epistemic(a, b):
    if a.names & b.names:
        return False
    if a.requires & b.names or b.requires & a.names:
        return False
    return True


def overlap(a, b):
    return epistemic(a, b) and classical(a, b)


class Node:
    def __init__(self, item, nxt=None, outcomes=None):
        self.item = item
        self.next = nxt or []          # one continuation, or one per outcome (open, shut)
        self.outcomes = outcomes       # for a look: the outcome of each continuation
        self.group = 1

    def copy(self):
        n = Node(self.item, list(self.next), self.outcomes)
        n.group = self.group
        return n


def written(steps, formulation, act):
    """The policy as a chain of nodes: steps in order, a reading branching by outcome."""
    head = None
    for st in reversed(steps):
        if st[0] == 'branch':
            _, r, a, branches = st
            outs = [o for o in ('open', 'shut') if o in branches]
            nxt = [written(branches[o], formulation, act) for o in outs]
            head = Node(Item(r, a, formulation, act), nxt, outs)
        else:
            r, a = st
            head = Node(Item(r, a, formulation, act), [head] if head else [])
    return head


class Scheduler:
    """policy_schedule.cpp's Scheduler: settle (with lift) and group."""

    def settle(self, node, above):
        if node is None:
            return None
        start = self.earliest(node.item, above)
        copy = node.copy()
        below = above + [(node.item, start)]
        copy.next = [self.settle(n, below) for n in copy.next]
        lifted = self.lift(copy, above, start)
        return lifted if lifted else copy

    def earliest(self, item, above):
        start = 0.0
        for anc, s in above:
            if not overlap(anc, item):
                start = max(start, s + max(0.0, anc.duration))
        return start

    def lift(self, parent, above, parent_start):
        if not parent.next or any(n is None for n in parent.next):
            return None
        first = parent.next[0]
        if not all(n.item.same(first.item) for n in parent.next):
            return None
        if not overlap(parent.item, first.item):
            return None
        start = self.earliest(first.item, above)
        if not start < parent_start:
            return None
        slots = max(len(first.next), 1)
        moved = Node(first.item, [], first.outcomes)
        below = above + [(first.item, start)]
        for slot in range(slots):
            c = parent.copy()
            c.next = [(None if not twin.next else twin.next[slot]) for twin in parent.next]
            moved.next.append(self.settle(c, below))
        return moved

    def group(self, node, seen=None):
        seen = set() if seen is None else seen
        if node is None or id(node) in seen:
            return
        seen.add(id(node))
        members = [node.item]
        frontier = [node]
        while True:
            following, whole = [], True
            for at in frontier:
                if not at.next:
                    whole = False
                    break
                for n in at.next:
                    if n is None:
                        whole = False
                        break
                    following.append(n)
                if not whole:
                    break
            if not whole or not following:
                break
            cand = following[0].item
            if not all(o.item.same(cand) for o in following):
                break
            if not all(overlap(m, cand) for m in members):
                break
            members.append(cand)
            frontier = following
        node.group = len(members)
        for at in frontier:
            for n in at.next:
                self.group(n, seen)


def schedule(tree, formulation, act):
    root = written(P.flat(tree), formulation, act)
    s = Scheduler()
    root = s.settle(root, [])
    s.group(root)
    return root


def outcome_next(node, night):
    """The continuation of a node on a night: by the reading, or the only one."""
    if not node.next:
        return None
    if node.outcomes is None:
        return node.next[0]
    return node.next[node.outcomes.index(P.outcome_on(night, node.item.robot, node.item.a))]


def timeline(root, night, duration=None):
    """The time of the last delivery on a night when the schedule's groups run as ePlanSys
    runs them: a group's members start together and the group ends with its longest."""
    duration = duration or (lambda it: it.duration)
    t, node, groups, last = 0.0, root, [], 0.0
    while node is not None:
        members, at = [node], node
        for _ in range(node.group - 1):
            at = outcome_next(at, night)
            members.append(at)
        t += max(duration(m.item) for m in members)
        if any(m.item.a[0] == 'deliver' for m in members):
            last = t
        groups.append([m.item.name for m in members])
        node = outcome_next(members[-1], night)
    return last or t, groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--act', type=int, required=True)
    ap.add_argument('--formulation', required=True)
    ap.add_argument('--night')
    a = ap.parse_args()
    sys.setrecursionlimit(10000)
    pl, tree = P.plan(a.act, a.formulation)
    root = schedule(tree, a.formulation, a.act)
    for night in R.nights(a.act):
        if a.night and R.tag(night) != a.night:
            continue
        t, groups = timeline(root, night)
        mine = P.timeline(P.trace(tree, night), pl.fleet)[0]
        print(f'{R.tag(night)}: {t:.0f} s by the executor\'s groups, {mine:.0f} s by the planner\'s')
        if a.night:
            for g in groups:
                print('   ', ' | '.join(g))


if __name__ == '__main__':
    main()
