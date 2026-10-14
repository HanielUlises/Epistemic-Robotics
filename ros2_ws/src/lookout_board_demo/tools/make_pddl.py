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
The classical half of the lookout board: the PDDL domain, the mapping from
each epistemic action to the PDDL action the executor dispatches, and the
problems the classical planners are given.

    make_pddl.py --out pddl/

The PDDL domain is what the robot physically does, and it serves two purposes.
Under the epistemic policy it is only the executor's vocabulary: the mapping
says that read-2_r1 is (read_slot r1 board s2). Under the classical
formulations it is the whole model, and it is as strong as a classical model
of this task can be:

  * each drive lasts the time its route takes, from the same roadmap the robot
    drives, so the planner knows which lookout is nearest;
  * a look needs no knowledge, only a lookout not yet found screened;
  * every action of the epistemic domain has its counterpart, reading the
    board included, so nothing the policy does is unavailable to it.

What it cannot say is what the robot will know after an action, which is what
decides whether reading the board is worth a detour.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

SPEED = 0.18           # m/s, the drive node's cruising speed with its turns
TURN = 4.0             # s, turning at each end of a drive
PLACES = ['start', 'board', 'l1', 'l2', 'l3', 'l4']
LOOKOUTS = ['l1', 'l2', 'l3', 'l4']
SLOTS = ['s1', 's2', 's3', 's4']

DOMAIN = """;; THE LOOKOUT BOARD, CLASSICAL. Written by tools/make_pddl.py.
;;
;; What the robot physically does. Under the epistemic policy this is only the
;; executor's vocabulary; under the classical formulations it is the whole
;; model. A look needs no knowledge, only a lookout not yet found screened, and
;; makes the zone known: the classical planner assumes it will see. Nothing
;; here can say what the robot will know after reading the board, which is the
;; one thing that would make reading it worth a detour.

(define (domain lookout-board)
(:requirements :strips :typing :durative-actions :fluents)

(:types
robot
place
slot
)

(:predicates
(robot_at ?r - robot ?p - place)
(is_board ?p - place)
(is_lookout ?p - place)
(open_view ?p - place)       ; not yet found screened
(read ?r - robot ?s - slot)
(checked ?r - robot ?p - place)
(looked ?r - robot ?p - place)
(zone_known)
)

(:functions
(route ?a ?b - place)
)

(:durative-action goto
    :parameters (?r - robot ?from ?to - place)
    :duration ( = ?duration (route ?from ?to))
    :condition (and
        (at start (robot_at ?r ?from))
    )
    :effect (and
        (at start (not (robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
    )
)

(:durative-action read_slot
    :parameters (?r - robot ?b - place ?s - slot)
    :duration ( = ?duration 2)
    :condition (and
        (over all (robot_at ?r ?b))
        (at start (is_board ?b))
    )
    :effect (and
        (at end (read ?r ?s))
    )
)

(:durative-action check_view
    :parameters (?r - robot ?l - place)
    :duration ( = ?duration 2)
    :condition (and
        (over all (robot_at ?r ?l))
        (at start (is_lookout ?l))
    )
    :effect (and
        (at end (checked ?r ?l))
    )
)

(:durative-action look_zone
    :parameters (?r - robot ?l - place)
    :duration ( = ?duration 3)
    :condition (and
        (over all (robot_at ?r ?l))
        (at start (is_lookout ?l))
        (at start (open_view ?l))
    )
    :effect (and
        (at end (looked ?r ?l))
        (at end (zone_known))
    )
)
)
"""


def seconds(a, b):
    return round(L.shortest(a, b)[1] / SPEED + TURN, 1)


def problem(at='start', screened=(), goal='replan'):
    """The classical problem: from where the robot stands, with the lookouts
    already found screened. goal is 'replan' (the zone known) or 'conformant'
    (a look from every lookout)."""
    objs = ['(:objects', '  r1 - robot', '  ' + ' '.join(PLACES) + ' - place',
            '  ' + ' '.join(SLOTS) + ' - slot', ')']
    init = ['(:init', f'  (robot_at r1 {at})', '  (is_board board)']
    init += [f'  (is_lookout {lk})' for lk in LOOKOUTS]
    # A lookout found screened loses open_view. Written positively: POPF dumps
    # core on this domain with the negative precondition (not (screened ?l)).
    init += [f'  (open_view {lk})' for lk in LOOKOUTS if lk not in screened]
    init += [f'  (= (route {a} {b}) {seconds(a, b)})' for a in PLACES for b in PLACES if a != b]
    init += [')']
    if goal == 'conformant':
        g = '(:goal (and ' + ' '.join(f'(looked r1 {lk})' for lk in LOOKOUTS) + '))'
    else:
        g = '(:goal (and (zone_known)))'
    return '\n'.join(['(define (problem lookout-board-1)', '(:domain lookout-board)']
                     + objs + init + [g, ')']) + '\n'


def mapping():
    """Each epistemic action, as the PDDL action the executor dispatches."""
    m = {'goto-board_r1': {'action': '(goto r1 start board)', 'duration': seconds('start', 'board')}}
    for k in range(1, 5):
        m[f'goto-l{k}_r1'] = {'action': f'(goto r1 board l{k})',
                              'duration': seconds('board', f'l{k}')}
        m[f'read-{k}_r1'] = {'action': f'(read_slot r1 board s{k})', 'duration': 2.0}
        m[f'check-{k}_r1'] = {'action': f'(check_view r1 l{k})', 'duration': 2.0}
        m[f'look-{k}_r1'] = {'action': f'(look_zone r1 l{k})', 'duration': 3.0}
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'lookout-board.pddl'), 'w').write(DOMAIN)
    open(os.path.join(a.out, 'problem-replan.pddl'), 'w').write(problem())
    open(os.path.join(a.out, 'problem-conformant.pddl'), 'w').write(problem(goal='conformant'))
    json.dump(mapping(), open(os.path.join(a.out, 'lookout-board-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
