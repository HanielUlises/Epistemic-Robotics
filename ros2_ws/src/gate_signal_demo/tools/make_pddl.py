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
The classical side of the gate signal.

    make_pddl.py --out pddl/

Writes two PDDL domains and the mapping:

  gate-signal-vocabulary.pddl   the executor's vocabulary under the epistemic
                                policy: one action per thing a robot does,
                                with no conditions worth the name, since what
                                decides the policy is in the EPDDL;
  gate-signal-classical.pddl    the model each robot plans with on its own in
                                the classical formulations. r1 knows its order
                                and has both routes to its bay, each lasting
                                the time its route takes over the roadmap; r2
                                can watch the gates, drive to a bay's door,
                                look in, go in and unload. Neither has any
                                notion of what the other knows.
  gate-signal-mapping.json      each epistemic action as the vocabulary action
                                the executor dispatches.

problem_r1() and problem_r2() give each robot's own problem.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

SPEED = 0.18
TURN = 4.0

VOCABULARY = """;; THE GATE SIGNAL: the executor's vocabulary under the epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain gate-signal)
(:requirements :strips :typing :durative-actions)
(:types robot bay)
(:predicates (robot_ok ?r - robot))
""" + ''.join(f"""
(:durative-action {name}
    :parameters ({params})
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))
""" for name, params in (('read_order', '?r - robot'), ('to_gate', '?r - robot'), ('back_route', '?r - robot'),
                         ('enter_bay', '?r - robot'), ('watch', '?r ?o - robot'), ('go', '?r - robot ?b - bay'),
                         ('unload', '?r - robot'))) + ")\n"

CLASSICAL = """;; THE GATE SIGNAL, CLASSICAL. Written by tools/make_pddl.py.
;;
;; Each robot plans with this on its own. r1 knows its order and has two routes
;; to its bay, the back route and the route through its gate, each lasting the
;; time it takes, and reads the order board first; r2 can drive from its booth into a bay, look over a bay from
;; its side door, cross from one bay to the other, and unload. Nothing here
;; says what either robot knows of the other.
(define (domain gate-signal)
(:requirements :strips :typing :durative-actions :fluents)
(:types robot bay)
(:predicates
  (at_start ?r - robot) (at_gate ?r - robot ?b - bay) (at_station ?r - robot)
  (in ?r - robot ?b - bay) (ordered ?r - robot ?b - bay) (free ?b - bay)
  (looked ?r - robot ?b - bay) (delivered ?r - robot) (read ?r - robot))
(:functions (d_back ?b - bay) (d_gate ?b - bay) (d_enter ?b - bay) (d_go ?b - bay) (d_cross))
(:durative-action read_order
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (read ?r))))
(:durative-action back_route
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_back ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)))
  :effect (and (at start (not (at_start ?r))) (at end (in ?r ?b))))
(:durative-action to_gate
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_gate ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)))
  :effect (and (at start (not (at_start ?r))) (at end (at_gate ?r ?b))))
(:durative-action enter_bay
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_enter ?b))
  :condition (and (at start (at_gate ?r ?b)))
  :effect (and (at start (not (at_gate ?r ?b))) (at end (in ?r ?b))))
(:durative-action go
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_go ?b))
  :condition (and (at start (at_station ?r)))
  :effect (and (at start (not (at_station ?r))) (at end (in ?r ?b))))
(:durative-action look_bay
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration 3)
  :condition (and (over all (in ?r ?b)))
  :effect (and (at end (looked ?r ?b))))
(:durative-action cross
  :parameters (?r - robot ?from ?to - bay)
  :duration ( = ?duration (d_cross))
  :condition (and (at start (in ?r ?from)))
  :effect (and (at start (not (in ?r ?from))) (at end (in ?r ?to))))
(:durative-action unload
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration 2)
  :condition (and (over all (in ?r ?b)) (at start (free ?b)))
  :effect (and (at end (delivered ?r))))
)
"""


def seconds(path):
    nodes, _ = L.roadmap()
    import math
    d = sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))
    return round(d / SPEED + TURN, 1)


ROUTES = {  # per robot action, the places for bay x; bay y mirrors them
    'back_route': ['start', 's_x', 'sd_x', 'bay_x'],
    'to_gate': ['start', 'pass_x', 'gate_x'],
    'enter_bay': ['gate_x', 'side_x', 'in_x', 'bay_x'],
    'go': ['station', 'gate_x', 'side_x', 'in_x'],
    'cross': ['in_y', 'side_y', 'gate_y', 'station', 'gate_x', 'side_x', 'in_x'],  # into bay x
}


def route(action, b):
    flip = {'x': 'y', 'y': 'x'}
    return [p[:-1] + (p[-1] if b == 'x' else flip[p[-1]]) if p[-2:] in ('_x', '_y') else p
            for p in ROUTES[action]]


def functions():
    out = []
    for b in ('x', 'y'):
        out += [f'  (= (d_back bay_{b}) {seconds(route("back_route", b))})',
                f'  (= (d_gate bay_{b}) {seconds(route("to_gate", b))})',
                f'  (= (d_enter bay_{b}) {seconds(route("enter_bay", b))})',
                f'  (= (d_go bay_{b}) {seconds(route("go", b))})']
    return out + [f'  (= (d_cross) {seconds(route("cross", "x"))})']


def problem_read():
    return ('(define (problem gate-signal-r1)\n(:domain gate-signal)\n(:objects r1 - robot bay_x bay_y - bay)\n'
            '(:init (at_start r1))\n(:goal (and (read r1)))\n)\n')


def problem_r1(order):
    return '\n'.join(['(define (problem gate-signal-r1)', '(:domain gate-signal)',
                      '(:objects r1 - robot bay_x bay_y - bay)', '(:init', '  (at_start r1)',
                      f'  (ordered r1 bay_{order})', f'  (free bay_{order})'] + functions() + [')', '(:goal (and (delivered r1)))', ')']) + '\n'


def problem_r2(at='station', free=('x', 'y'), goal='(delivered r2)'):
    """r2's own problem: from its station or from inside bay `at`, with the
    bays it takes to be free."""
    init = ['  (at_station r2)'] if at == 'station' else [f'  (in r2 bay_{at})']
    return '\n'.join(['(define (problem gate-signal-r2)', '(:domain gate-signal)',
                      '(:objects r2 - robot bay_x bay_y - bay)', '(:init'] + init
                     + [f'  (free bay_{b})' for b in free] + functions()
                     + [')', f'(:goal (and {goal}))', ')']) + '\n'


def problem_vocabulary():
    return ('(define (problem gate-signal-1)\n(:domain gate-signal)\n'
            '(:objects r1 r2 - robot bay_x bay_y - bay)\n'
            '(:init (robot_ok r1) (robot_ok r2))\n(:goal (and (robot_ok r1)))\n)\n')


def mapping():
    m = {'read-order_r1': '(read_order r1)', 'to-gate_r1': '(to_gate r1)', 'back-route_r1': '(back_route r1)',
         'enter-bay_r1': '(enter_bay r1)', 'watch_r2_r1': '(watch r2 r1)', 'go-x_r2': '(go r2 bay_x)',
         'go-y_r2': '(go r2 bay_y)', 'drop-carrier_r1': '(unload r1)', 'drop-other-x_r2': '(unload r2)',
         'drop-other-y_r2': '(unload r2)'}
    return {k: {'action': v, 'duration': 10.0} for k, v in m.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'gate-signal-vocabulary.pddl'), 'w').write(VOCABULARY)
    open(os.path.join(a.out, 'gate-signal-classical.pddl'), 'w').write(CLASSICAL)
    json.dump(mapping(), open(os.path.join(a.out, 'gate-signal-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
