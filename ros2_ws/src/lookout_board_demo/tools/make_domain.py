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
Writes the lookout-board domain and its problem for n lookouts.

    make_domain.py --lookouts 4 --out epddl/

One robot must come to know whether a zone is blocked. It can see the zone from
any of n lookouts, but a screen stands in front of all of them except one, and
the robot does not know which. It can find out in two ways: at a lookout, by
checking the view from there; or at a status board near the start, whose slot
k is lit exactly when the view from lookout k is clear.

The sensing events carry no condition that the robot not know the answer
yet. On the robot every slot of the board resolves at once, and the epistemic
state applies each observation as soon as its event is applicable; with such a
condition, the dark slots applied first would make the lit one known by
elimination and its read inapplicable, and the executor would refuse the very
action the policy runs next. Reading what is already known changes nothing.

Every value is its own atom and every place its own action: plank numbers
domain constants ahead of the agents, so an action over a constant would index
past the agents' table (see the repository's notes on plank).
"""

import argparse
import os

HEAD = """;; THE LOOKOUT BOARD, {n} LOOKOUTS. Written by tools/make_domain.py.
;;
;; One robot must know whether a zone is blocked. It can see the zone from any
;; of {n} lookouts, but a screen hides the zone from every lookout except one,
;; and which one is open is not known. The robot can learn it at a lookout, by
;; checking the view from there, or at a status board near its start, whose
;; slot k is lit when the view from lookout k is clear. Looking at the zone from
;; lookout k requires knowing that the view from k is clear.
;;
;; Nothing in the goal mentions the board. Whether reading it pays is for the
;; planner to find out from what the robot would know after reading it.
"""


def domain(n):
    L = range(1, n + 1)
    out = [HEAD.format(n=n), '(define (domain lookout-board)', '',
           '  (:requirements',
           '    :equality :partial-observability :list-comprehensions :lists',
           '    :ontic-actions :group-modalities',
           '    :negative-preconditions :modal-preconditions :knowing-whether',
           '  )', '', '  (:action-type-libraries intermediate)', '', '  (:predicates',
           '    (blocked-z)                ; the zone is blocked']
    out += [f'    (clear-{k})                  ; the view from lookout {k} is clear' for k in L]
    out += ['    (at-board ?i - agent)      ; ?i stands at the status board']
    out += [f'    (at-l{k} ?i - agent)         ; ?i stands at lookout {k}' for k in L]
    out += ['  )', '', '  (:event nil)', '']
    places = ['board'] + [f'l{k}' for k in L]
    for p in places:
        others = [f'(not (at-{q} ?i))' for q in places if q != p]
        out += [f'  ;; Driving to {"the board" if p == "board" else "lookout " + p[1:]}: '
                'public, and the robot is in one place at a time.',
                f'  (:event e-goto-{p}', '    :parameters (?i - agent)',
                f'    :precondition (not (at-{p} ?i))',
                f'    :effects (:and (at-{p} ?i) {" ".join(others)})', '  )',
                f'  (:action goto-{p}', '    :parameters (?i - agent)',
                f'    :action-type (public-ontic (e-goto-{p} ?i))',
                '    :observability-conditions (default Fully)', '  )', '']
    for k in L:
        out += [f'  ;; Slot {k} of the board: lit when the view from lookout {k} is clear.',
                f'  (:event e-read{k}-lit', '    :parameters (?i - agent)',
                f'    :precondition (and (at-board ?i) (clear-{k}) )', '  )',
                f'  (:event e-read{k}-dark', '    :parameters (?i - agent)',
                f'    :precondition (and (at-board ?i) (not (clear-{k})) )',
                '  )',
                f'  (:action read-{k}', '    :parameters (?i - agent)',
                f'    :action-type (semi-private-sensing (e-read{k}-lit ?i) (e-read{k}-dark ?i))',
                '    :observability-conditions (:and (?i Fully) (default Partially))', '  )', '']
        out += [f'  ;; At lookout {k}: is the view screened?',
                f'  (:event e-check{k}-clear', '    :parameters (?i - agent)',
                f'    :precondition (and (at-l{k} ?i) (clear-{k}) )', '  )',
                f'  (:event e-check{k}-screened', '    :parameters (?i - agent)',
                f'    :precondition (and (at-l{k} ?i) (not (clear-{k})) )',
                '  )',
                f'  (:action check-{k}', '    :parameters (?i - agent)',
                f'    :action-type (semi-private-sensing (e-check{k}-clear ?i) '
                f'(e-check{k}-screened ?i))',
                '    :observability-conditions (:and (?i Fully) (default Partially))', '  )', '']
        out += [f'  ;; Looking at the zone from lookout {k} requires knowing the view is clear.',
                f'  (:event e-look{k}-blocked', '    :parameters (?i - agent)',
                f'    :precondition (and (at-l{k} ?i) ([?i] (clear-{k})) (blocked-z) '
                ')', '  )',
                f'  (:event e-look{k}-free', '    :parameters (?i - agent)',
                f'    :precondition (and (at-l{k} ?i) ([?i] (clear-{k})) (not (blocked-z)) '
                ')', '  )',
                f'  (:action look-{k}', '    :parameters (?i - agent)',
                f'    :action-type (semi-private-sensing (e-look{k}-blocked ?i) (e-look{k}-free ?i))',
                '    :observability-conditions (:and (?i Fully) (default Partially))', '  )', '']
    out += [')']
    return '\n'.join(out) + '\n'


def problem(n):
    L = range(1, n + 1)
    one = '(or ' + ' '.join(f'(clear-{k})' for k in L) + ')'
    pairs = [f'      ([C. All] (not (and (clear-{a}) (clear-{b}))))'
             for a in L for b in L if a < b]
    places = ['board'] + [f'l{k}' for k in L]
    out = [f';; {n} lookouts, exactly one of them with a clear view; the robot at its start.',
           '(define (problem lookout-board-1)', '', '  (:domain lookout-board)', '',
           '  (:requirements', '    :equality :list-comprehensions',
           '    :finitary-S5-theories :modal-goals :knowing-whether',
           '    :multi-pointed-models :negative-preconditions', '  )', '', '  (:agents r1)', '',
           '  (:init', '    (:and']
    out += [f'      ([C. All] (not (at-{p} r1)))' for p in places]
    out += [f'      ([C. All] {one})'] + pairs
    out += ['      ([C. All] (<Kw. r1> (blocked-z)))']
    out += [f'      ([C. All] (<Kw. r1> (clear-{k})))' for k in L]
    out += ['    )', '  )', '', '  (:goal ([Kw. r1] (blocked-z)))', ')']
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lookouts', type=int, default=4)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'instances'), exist_ok=True)
    open(os.path.join(a.out, 'lookout-board.epddl'), 'w').write(domain(a.lookouts))
    open(os.path.join(a.out, 'instances', 'problem_1.epddl'), 'w').write(problem(a.lookouts))


if __name__ == '__main__':
    main()
