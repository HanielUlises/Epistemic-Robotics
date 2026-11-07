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
The executor's vocabulary and the action mapping.

    make_pddl.py --out pddl/

fleet-vocabulary.pddl   one action per kind of thing a robot does, each naming
                        its places: (drive r from to) along the hall, out of a
                        room or along the corridor to a bed, (lobby r l from to)
                        into, out of or through a lobby, (look r l view),
                        (tell r l) a door found shut, and (believe r l2 l v) a
                        belief reported to robot l2. The facts are there for
                        ePlanSys's parallel groups only: a lobby action holds
                        its lobby, so two robots are never dispatched into one
                        lobby together, and a report writes (radio), which every
                        action reads, so each goes alone. What decides the
                        policy is in the EPDDL.
fleet-mapping.json      every grounded action of the EPDDL as a vocabulary
                        action, with the duration tools/plan.py plans with.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import plan as P  # noqa: E402
import ranks as R  # noqa: E402

ROBOTS = sorted({r.name for act in R.FLEETS for r in R.FLEETS[act]})
ROBOT = {r.name: r for act in R.FLEETS for r in R.FLEETS[act]}


def durative(name, params, cond, eff):
    return (f'\n(:durative-action {name}\n  :parameters ({params})\n  :duration (= ?duration 10)\n'
            f'  :condition (and {cond})\n  :effect (and {eff}))\n')


def vocabulary():
    acts = durative('drive', '?r - robot ?a - place ?b - place', '(at start (robot_ok ?r)) (at start (radio))',
                    '(at end (robot_ok ?r))')
    acts += durative('lobby', '?r - robot ?l - lobby ?a - place ?b - place',
                     '(at start (robot_ok ?r)) (at start (radio)) (at start (lobby_free ?l))',
                     '(at end (robot_ok ?r)) (at end (lobby_free ?l))')
    acts += durative('look', '?r - robot ?l - lobby ?v - view',
                     '(at start (robot_ok ?r)) (at start (radio)) (at start (lobby_free ?l))',
                     '(at end (robot_ok ?r)) (at end (lobby_free ?l))')
    acts += durative('tell', '?r - robot ?l - lobby', '(at start (robot_ok ?r)) (at start (radio))',
                     '(at end (robot_ok ?r)) (at end (radio))')
    acts += durative('believe', '?r - robot ?h - robot ?l - lobby ?v - verdict',
                     '(at start (robot_ok ?r)) (at start (radio))', '(at end (robot_ok ?r)) (at end (radio))')
    return (';; BELIEF REVISION IN A MIXED FLEET: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain mixed-fleet)\n(:requirements :strips :typing :durative-actions)\n'
            '(:types robot place lobby view verdict)\n'
            '(:predicates (robot_ok ?r - robot) (radio) (lobby_free ?l - lobby))\n' + acts + ')\n')


def problem(robots):
    robots = list(robots)
    places = sorted(L.roadmap()[0])
    lobbies = [f'lobby_{d}' for d in L.DOORS]
    views = [f'view_{d}s' for d in L.DOORS]
    init = [f'(robot_ok {r})' for r in robots] + ['(radio)'] + [f'(lobby_free {x})' for x in lobbies]
    return (f"(define (problem mixed-fleet) (:domain mixed-fleet)\n"
            f"(:objects {' '.join(robots)} - robot {' '.join(places)} - place {' '.join(lobbies)} - lobby "
            f"{' '.join(views)} - view shut open - verdict)\n(:init {' '.join(init)})\n"
            f"(:goal (and (robot_ok {robots[0]}))))\n")


def mapping():
    out = {}
    for r in ROBOTS:
        rb = ROBOT[r]
        c = P.SOUTH
        for i in range(len(c)):
            for j in (i - 1, i + 1):
                if 0 <= j < len(c):
                    a = ('seek', i, j)
                    out[P.grounded(r, a)] = {'action': f'(drive {r} {c[i]} {c[j]})', 'duration': float(P.hop_time(i, j))}
        for d in L.DOORS:
            out[P.grounded(r, ('enter', d))] = {'action': f'(lobby {r} lobby_{d} v{d} j{d})',
                                                'duration': float(P.enter_time(d))}
            out[P.grounded(r, ('back', d))] = {'action': f'(lobby {r} lobby_{d} j{d} v{d})',
                                               'duration': float(P.back_time(d))}
            out[P.grounded(r, ('look', d))] = {'action': f'(look {r} lobby_{d} view_{d}s)', 'duration': float(P.LOOK)}
            out[P.grounded(r, ('cross', d))] = {'action': f'(lobby {r} lobby_{d} j{d} n{d})',
                                                'duration': float(P.cross_time(d))}
            out[P.grounded(r, ('deliver', d))] = {'action': f'(drive {r} n{d} bed{rb.ward})',
                                                  'duration': float(P.deliver_time(d, rb.ward))}
            out[P.grounded(r, ('tell', d))] = {'action': f'(tell {r} lobby_{d})', 'duration': float(P.TELL)}
            for h in ROBOTS:
                for v in (False, True):
                    out[P.grounded(r, ('tellb', h, d, v))] = {
                        'action': f"(believe {r} {h} lobby_{d} {'open' if v else 'shut'})", 'duration': float(P.TELL)}
        out[P.grounded(r, ('out', r))] = {'action': f'(drive {r} {rb.room} {c[P.ROOM_AT[rb.room]]})',
                                          'duration': float(P.room_time(rb.room))}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for f in os.listdir(a.out):
        os.remove(os.path.join(a.out, f))
    open(os.path.join(a.out, 'fleet-vocabulary.pddl'), 'w').write(vocabulary())
    json.dump(mapping(), open(os.path.join(a.out, 'fleet-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
