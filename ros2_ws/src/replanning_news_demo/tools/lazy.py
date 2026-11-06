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
Lazy commitment: a policy's drives moved past the looks and messages that do
not need them.

AO* minimises the worst branch's length, so a drive placed before a branch
point costs it nothing when the worst branch needs the drive anyway, even if
the other branches do not. The executor then completes the drive before it
learns which branch it is on, and on a branch that sends the robot elsewhere
the drive is wasted, in both directions.

A drive of one robot and a look or a message of another commute: neither
reads what the other writes, and the drive is public and ontic, so it changes
nobody's knowledge of the doors. So the drive can be moved below the look or
message, into each of its branches; on a branch where the robot's next action
is a drive somewhere else, the moved drive's only effect is overwritten before
anything reads it, and it is dropped. The result is a solution wherever the
original is, and no branch is longer. The drive is not moved past another
robot's drive, which the executor runs beside it anyway, nor where the
robot's next drive would return it to where it stands.

    lazy(plan, robot, place)  -> (plan, moved)

works on a plansys2_msgs/Plan whose items carry the epistemic fields; place is
the robot's place in the model (st, vw, vm, ve, ks).
"""

import copy

DONE = 4294967295


def _actor(item):
    body = item.action.strip('()').split()
    return body[0], (body[1] if len(body) > 1 else '')


def _tree(plan):
    def node(i):
        if i == DONE:
            return None
        it = plan.items[i]
        return {'item': it, 'next': [(o, node(c)) for o, c in zip(it.outcomes, it.children)]}
    return node(0) if plan.items else None


def _flatten(root, template):
    items = []

    def visit(n):
        if n is None:
            return DONE
        k = len(items)
        it = copy.deepcopy(n['item'])
        items.append(it)
        it.children = [visit(child) for _, child in n['next']]
        it.outcomes = [o for o, _ in n['next']]
        return k
    visit(root)
    out = copy.deepcopy(template)
    out.items = items
    return out


def _push(move, target, robot, place, n, count):
    """The subtree n with the drive `move` (to `target`) moved as far down as it may go."""
    if n is None:
        count[1] += 1
        return None
    name, who = _actor(n['item'])
    if who == robot:
        if name.startswith('goto_') and name[5:] != target and name[5:] != place:
            count[1] += 1           # overwritten before it is read: dropped
            return n
        return {'item': move['item'], 'next': [(move['next'][0][0], n)]}
    if name.startswith(('look_', 'tell_')):
        return {'item': n['item'],
                'next': [(o, _push(move, target, robot, place, c, count)) for o, c in n['next']]}
    return {'item': move['item'], 'next': [(move['next'][0][0], n)]}


def _walk(n, robot, place, count):
    if n is None:
        return None
    name, who = _actor(n['item'])
    if who == robot and name.startswith('goto_') and len(n['next']) == 1 and n['next'][0][1] is not None:
        nxt = n['next'][0][1]
        nname, nwho = _actor(nxt['item'])
        if nwho != robot and nname.startswith(('look_', 'tell_')) and len(nxt['next']) > 1:
            count[0] += 1
            moved = _push(n, name[5:], robot, place, nxt, count)
            return _walk(moved, robot, place, count)
    return {'item': n['item'], 'next': [(o, _walk(c, robot, place, count)) for o, c in n['next']]}


def lazy(plan, robot='c', place='st'):
    """The plan with the robot's drives committed late; and (drives moved, drives dropped)."""
    root = _tree(plan)
    if root is None:
        return plan, (0, 0)
    count = [0, 0]
    out = _walk(root, robot, place, count)
    return _flatten(out, plan), tuple(count)


def lines(plan):
    """The plan as indented text, for the record."""
    out = []

    def visit(i, d):
        if i == DONE:
            return
        it = plan.items[i]
        out.append('  ' * d + it.action + (f'  [{", ".join(it.outcomes)}]' if len(it.children) > 1 else ''))
        for c in it.children:
            visit(c, d + (1 if len(it.children) > 1 else 0))
    if plan.items:
        visit(0, 0)
    return out
