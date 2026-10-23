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
ePlanSys's arrangement of a policy for dispatch, so that the kinematic
simulator runs a policy as the executor does in Gazebo.

A port of plansys2_epistemic_executor's schedule_policy (policy_schedule.cpp
and policy_parallel.cpp, release 0.3.0). An action is moved above its parent
when the two are independent and it could start earlier, by declared
duration; above a branch only when every branch begins with it, the branch
being copied under each of its outcomes. Then every chain of pairwise
independent actions is started together, and the chain's outcomes choose
what follows once all of its members have finished.

Two actions are independent when the agents they name are disjoint and
neither's knowledge requirements mention an agent the other names (the
epistemic half), and when neither's effects disturb what the other requires in
the executor's vocabulary (the classical half). The vocabulary of this
demonstration (vocabulary() below, and the PDDL that make_pddl.py writes from
it) gives every action (ok ?r) of the robots it names to read and write; an
arrival at the site, go-site-n, also reads (count n) and writes (count n+1),
and the arrival that does the task, do-task-k, reads (count k-1): arrivals are
dispatched in the order of the counter, which the epistemic state requires,
since each arrival's event needs the count the one before it set.
"""

import re

MAX_NODES = 20000


def vocabulary(action):
    """The facts a grounded action reads and writes in the executor's
    vocabulary: (requirements, effects)."""
    head, *args = action.split('_')
    ok = {f'ok_{a}' for a in args}
    if head.startswith('go-site-'):
        n = int(head.rsplit('-', 1)[1])
        return ok | ({f'count_{n}'} if n else set()), ok | {f'count_{n + 1}'}
    if head.startswith('do-task-'):
        k = int(head.rsplit('-', 1)[1])
        return ok | ({f'count_{k - 1}'} if k > 1 else set()), ok
    return ok, ok


class Item:
    """One node of the policy as written."""

    __slots__ = ('index', 'action', 'agents', 'requires', 'duration', 'children', 'reads', 'writes')

    def __init__(self, index, action, agents, requires, duration):
        self.index, self.action, self.agents = index, action, agents
        self.requires, self.duration = requires, duration
        self.children = []         # one per outcome; None ends the policy
        self.reads, self.writes = vocabulary(action)


class Node:

    __slots__ = ('item', 'next', 'group')

    def __init__(self, item, nxt=None, group=1):
        self.item = item
        self.next = list(nxt or [])
        self.group = group


def agents_of(name):
    """The agents a grounded action's name names: what follows its first
    underscore, part by part (item_agents)."""
    return set(p for p in name.split('_')[1:] if p)


def mentions(text, name):
    return re.search(r'(?<![\w-])' + re.escape(name) + r'(?![\w-])', text) is not None


def independent(a, b):
    """may_overlap: the epistemic half, then the classical one."""
    if a.agents & b.agents:
        return False
    if a.writes & b.reads or b.writes & a.reads or a.writes & b.writes:
        return False
    for one in a.agents:
        if any(mentions(r, one) for r in b.requires):
            return False
    for other in b.agents:
        if any(mentions(r, other) for r in a.requires):
            return False
    return True


def items_of(policy, task, duration_of, requires_of):
    """The policy's nodes as Items, the root first. `policy` is Aletheia's
    JSON tree; each branch names an outcome by an event of the action."""
    items = []

    def walk(node):
        if node is None:
            return None
        it = Item(len(items), node['action'], agents_of(node['action']), requires_of(node['action']),
                  duration_of(node['action']))
        items.append(it)
        kids = []
        for b in node['branches']:
            c = walk(b['subtree'])
            kids.append(c.index if c is not None else None)
        it.children = kids
        return it
    walk(policy)
    return items


class Schedule:
    """schedule_policy over the Items of one policy."""

    def __init__(self, items, events):
        self.items = items
        self.events = events        # item index -> [event name per child], or None when it has one child
        self.overlap_cache = {}
        self.made = 0
        self.seen = set()

    def overlap(self, a, b):
        key = (min(a, b), max(a, b))
        if key not in self.overlap_cache:
            self.overlap_cache[key] = independent(self.items[a], self.items[b])
        return self.overlap_cache[key]

    def same_action(self, a, b):
        return self.items[a].action == self.items[b].action and self.events.get(a) == self.events.get(b)

    def written(self, index, made):
        if index in made:
            return made[index]
        node = Node(index)
        made[index] = node
        it = self.items[index]
        if len(it.children) == 1:
            node.next = [self.written(it.children[0], made)] if it.children[0] is not None else []
        elif len(it.children) > 1:
            node.next = [None if c is None else self.written(c, made) for c in it.children]
        return node

    def fresh(self, node):
        self.made += 1
        return Node(node.item, node.next, node.group)

    def too_big(self):
        return self.made > MAX_NODES

    def earliest(self, item, above):
        start = 0.0
        for ancestor, ancestor_start in above:
            if not self.overlap(ancestor, item):
                start = max(start, ancestor_start + max(0.0, self.items[ancestor].duration))
        return start

    def settle(self, node, above):
        if node is None or self.too_big():
            return node
        start = self.earliest(node.item, above)
        copy = self.fresh(node)
        below = above + [(node.item, start)]
        copy.next = [self.settle(n, below) for n in copy.next]
        lifted = self.lift(copy, above, start)
        return lifted if lifted is not None else copy

    def lift(self, parent, above, parent_start):
        if not parent.next or any(n is None for n in parent.next):
            return None
        first = parent.next[0]
        if any(not self.same_action(n.item, first.item) for n in parent.next):
            return None
        if not self.overlap(parent.item, first.item):
            return None
        start = self.earliest(first.item, above)
        if not start < parent_start:
            return None
        slots = max(len(first.next), 1)
        moved = self.fresh(first)
        moved.next = []
        below = above + [(first.item, start)]
        for slot in range(slots):
            copy = self.fresh(parent)
            copy.next = [None if not twin.next else twin.next[slot] for twin in parent.next]
            moved.next.append(self.settle(copy, below))
        return moved

    def group(self, node):
        if node is None or id(node) in self.seen:
            return
        self.seen.add(id(node))
        members = [node.item]
        frontier = [node]
        while True:
            following = []
            whole = True
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
            candidate = following[0].item
            twins = all(self.same_action(o.item, candidate) for o in following)
            free = all(self.overlap(m, candidate) for m in members)
            if not twins or not free:
                break
            members.append(candidate)
            frontier = following
        node.group = len(members)
        for at in frontier:
            for n in at.next:
                self.group(n)

    def build(self):
        made = {}
        root = self.written(0, made)
        root = self.settle(root, [])
        if self.too_big():
            root = self.written(0, {})
        self.group(root)
        return root


def schedule(policy, task, duration_of, requires_of):
    """The arranged policy: (root Node, Items). A Node's `group` says how many
    nodes, it first, start together along `next`."""
    items = items_of(policy, task, duration_of, requires_of)
    # the outcomes of every branching item, so that twins compare alike
    events = {}
    index = 0

    def walk(node):
        nonlocal index
        if node is None:
            return
        i = index
        index += 1
        if len(node['branches']) > 1:
            events[i] = tuple(b['event'] for b in node['branches'])
        for b in node['branches']:
            walk(b['subtree'])
    walk(policy)
    sched = Schedule(items, events)
    return sched.build(), items
