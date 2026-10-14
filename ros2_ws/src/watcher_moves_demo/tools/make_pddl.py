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
The classical side of the watcher that moves.

    make_pddl.py --out pddl/

Writes two PDDL domains and two mappings:

  watcher-vocabulary.pddl  the executor's vocabulary under the epistemic
                           policy: one action per thing the carrier does, with
                           no conditions worth the name, since what decides
                           the policy is in the EPDDL;
  watcher-classical.pddl   the carrier's model in the classical formulations.
                           It has every action the epistemic domain has, each
                           lasting the time its route takes: reading the order
                           and the duty board, the drive through a gate, the
                           drives to a window, the lamp, the back route. The
                           ally is informed by a gate or a lamp; nothing says
                           who else sees either;
  watcher-v1-mapping.json  watcher.epddl's actions -- the policy that reads the
                           duty board once -- as vocabulary actions;
  watcher-mobile-mapping.json  the same for watcher-mobile.epddl's, the
                           domain in which the watcher may change posts.

problem_read() and problem_deliver() give the carrier's two problems.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

SPEED = 0.18
TURN = 4.0
LAMP = 4.0
GATE_HOLD = 3.0

ACTIONS = ('read_order', 'read_post', 'drive_win_w', 'drive_win_e', 'drive_ga_w', 'drive_ga_e', 'look', 'signal',
           'gate_in', 'to_gate', 'back_route')
VOCABULARY = """;; THE WATCHER THAT MOVES: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain watcher)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (robot_ok ?r - robot))
""" + ''.join(f"""
(:durative-action {name}
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))
""" for name in ACTIONS) + ")\n"

CLASSICAL = """;; THE WATCHER, CLASSICAL. Written by tools/make_pddl.py.
;;
;; The carrier's model. It knows its order once it has read the board, and has
;; every way the epistemic domain has of reaching its bay and of informing the
;; ally: the gate, which the ally's booth sees, or the lamp at a window. It can
;; read the duty board too. What it cannot say is that someone else is
;; watching: a goal over facts has no place for what another robot knows.
(define (domain watcher)
(:requirements :strips :typing :durative-actions :fluents)
(:types robot bay side)
(:predicates
  (at_start ?r - robot) (at_window ?r - robot ?s - side) (in ?r - robot ?b - bay)
  (ordered ?r - robot ?b - bay) (read ?r - robot) (post_read ?r - robot)
  (informed) (gates_allowed))
(:functions (d_gate ?b - bay) (d_window ?s - side) (d_back ?s - side ?b - bay))
(:durative-action read_order
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (read ?r))))
(:durative-action read_post
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (post_read ?r))))
(:durative-action to_gate
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_gate ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)) (at start (gates_allowed)))
  :effect (and (at start (not (at_start ?r))) (at end (in ?r ?b)) (at end (informed))))
(:durative-action to_window
  :parameters (?r - robot ?s - side)
  :duration ( = ?duration (d_window ?s))
  :condition (and (at start (at_start ?r)))
  :effect (and (at start (not (at_start ?r))) (at end (at_window ?r ?s))))
(:durative-action signal
  :parameters (?r - robot ?s - side ?b - bay)
  :duration ( = ?duration 4)
  :condition (and (over all (at_window ?r ?s)) (at start (ordered ?r ?b)))
  :effect (and (at end (informed))))
(:durative-action back_route
  :parameters (?r - robot ?s - side ?b - bay)
  :duration ( = ?duration (d_back ?s ?b))
  :condition (and (at start (at_window ?r ?s)) (at start (ordered ?r ?b)) (at start (informed)))
  :effect (and (at start (not (at_window ?r ?s))) (at end (in ?r ?b))))
)
"""


def seconds(path, hold=0.0):
    return round(L.length(path) / SPEED + TURN + hold, 1)


def functions():
    out = []
    for b in ('x', 'y'):
        out.append(f'  (= (d_gate bay_{b}) {seconds(L.route_to_gate(b) + L.route_gate_in(b)[1:], GATE_HOLD)})')
    for s in ('w', 'e'):
        out.append(f'  (= (d_window {s}) {seconds(L.route_to_window(s))})')
        for b in ('x', 'y'):
            out.append(f'  (= (d_back {s} bay_{b}) {seconds(L.route_back(s, b))})')
    return out


OBJECTS = '(:objects r1 - robot bay_x bay_y - bay w e - side)'


def problem_read():
    return ('(define (problem watcher-read)\n(:domain watcher)\n' + OBJECTS +
            '\n(:init (at_start r1))\n(:goal (and (read r1)))\n)\n')


def problem_deliver(order, gates=True):
    """The carrier's delivery, its order known. gates=False is the discreet
    carrier: the modeller has ruled the gates out, so the ally is told by lamp."""
    init = ['  (at_start r1)', f'  (ordered r1 bay_{order})'] + (['  (gates_allowed)'] if gates else [])
    return '\n'.join(['(define (problem watcher-deliver)', '(:domain watcher)', OBJECTS, '(:init'] + init
                     + functions() + [')', f'(:goal (and (in r1 bay_{order}) (informed)))', ')']) + '\n'


def problem_vocabulary():
    return ('(define (problem watcher-1)\n(:domain watcher)\n(:objects r1 - robot)\n'
            '(:init (robot_ok r1))\n(:goal (and (robot_ok r1)))\n)\n')


def mapping_v1():
    m = {'read-order_r1': '(read_order r1)', 'peek_r1_r3': '(read_post r1)', 'gate_r1_r2_r3': '(to_gate r1)',
         'to-win-w_r1': '(drive_win_w r1)', 'to-win-e_r1': '(drive_win_e r1)',
         'say-w_r1_r2_r3': '(signal r1)', 'say-e_r1_r2_r3': '(signal r1)', 'back_r1': '(back_route r1)'}
    return {k: {'action': v, 'duration': 10.0} for k, v in m.items()}


def mapping_mobile():
    m = {'read-order_r1': '(read_order r1)', 'peek_r1_r3': '(read_post r1)',
         'look-w_r1_r3': '(look r1)', 'look-e_r1_r3': '(look r1)',
         'say-w_r1_r2_r3': '(signal r1)', 'say-e_r1_r2_r3': '(signal r1)',
         'gate-x_r1_r2_r3': '(gate_in r1)', 'gate-y_r1_r2_r3': '(gate_in r1)', 'back_r1': '(back_route r1)'}
    for p in ('win-w', 'win-e', 'ga-w', 'ga-e'):
        m[f'to-{p}_r1_r3'] = f'(drive_{p.replace("-", "_")} r1)'
    return {k: {'action': v, 'duration': 10.0} for k, v in m.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'watcher-vocabulary.pddl'), 'w').write(VOCABULARY)
    open(os.path.join(a.out, 'watcher-classical.pddl'), 'w').write(CLASSICAL)
    json.dump(mapping_v1(), open(os.path.join(a.out, 'watcher-v1-mapping.json'), 'w'), indent=1)
    json.dump(mapping_mobile(), open(os.path.join(a.out, 'watcher-mobile-mapping.json'), 'w'), indent=1)
    print('\n'.join(functions()))


if __name__ == '__main__':
    main()
