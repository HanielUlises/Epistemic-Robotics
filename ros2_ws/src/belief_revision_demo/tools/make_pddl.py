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

revision-vocabulary.pddl  one action per kind of thing a robot does, each
                          naming its places: (drive r from to) along the hall
                          or the corridor or into a room, (lobby r l from to)
                          into, out of or through a lobby, (look r l view),
                          (tell r l) and the porter's (round p). The facts are
                          there for ePlanSys's parallel groups only: a lobby
                          action holds its lobby, so two robots are never
                          dispatched into one lobby together, and a report and
                          the porter's round write (radio), which every action
                          reads, so each goes alone. What decides the policy
                          is in the EPDDL.
revision-mapping.json     every grounded action of the EPDDL as a vocabulary
                          action, with the duration tools/plan.py plans with.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import plan as P  # noqa: E402

ROBOTS = ('c', 'd')


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
    acts += durative('round', '?r - robot', '(at start (robot_ok ?r)) (at start (radio))',
                     '(at end (robot_ok ?r)) (at end (radio))')
    return (';; BELIEF REVISION IN A CHANGING WARD: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain belief-revision)\n(:requirements :strips :typing :durative-actions)\n'
            '(:types robot place lobby view)\n'
            '(:predicates (robot_ok ?r - robot) (radio) (lobby_free ?l - lobby))\n' + acts + ')\n')


def problem(robots):
    robots = list(robots)
    places = sorted(L.roadmap()[0])
    lobbies = [f'lobby_{d}' for d in L.DOORS]
    views = [f'view_{d}{s}' for d in L.DOORS for s in 'sn']
    init = [f'(robot_ok {r})' for r in robots] + ['(radio)'] + [f'(lobby_free {x})' for x in lobbies]
    return (f"(define (problem belief-revision) (:domain belief-revision)\n"
            f"(:objects {' '.join(robots)} - robot {' '.join(places)} - place {' '.join(lobbies)} - lobby "
            f"{' '.join(views)} - view)\n(:init {' '.join(init)})\n(:goal (and (robot_ok c))))\n")


def mapping():
    out = {}
    room = {r: P.ROOM[r][2] for r in ROBOTS}
    hall = {r: P.SOUTH[P.ROOM[r][1]] for r in ROBOTS}
    bed = {r: P.WARD[r][2] for r in ROBOTS}
    corridor = {r: P.NORTH[P.WARD[r][1]] for r in ROBOTS}
    for r in ROBOTS:
        for side in ('S', 'N'):
            c = P.chain(side)
            for i in range(len(c)):
                for j in (i - 1, i + 1):
                    if 0 <= j < len(c):
                        for k in ('seek', 'walk'):
                            a = (k, side, i, j)
                            out[P.grounded(r, a)] = {'action': f'(drive {r} {c[i]} {c[j]})',
                                                     'duration': float(P.duration(r, a))}
        for d in L.DOORS:
            for side, m, b, far in (('S', 'v', 'j', 'n'), ('N', 'n', 'k', 'v')):
                out[P.grounded(r, ('enter', side, d))] = {'action': f'(lobby {r} lobby_{d} {m}{d} {b}{d})',
                                                          'duration': float(P.enter_time(side, d))}
                out[P.grounded(r, ('back', side, d))] = {'action': f'(lobby {r} lobby_{d} {b}{d} {m}{d})',
                                                         'duration': float(P.back_time(side, d))}
                out[P.grounded(r, ('look', side, d))] = {
                    'action': f"(look {r} lobby_{d} view_{d}{'s' if side == 'S' else 'n'})", 'duration': float(P.LOOK)}
                out[P.grounded(r, ('cross', side, d))] = {'action': f'(lobby {r} lobby_{d} {b}{d} {far}{d})',
                                                          'duration': float(P.cross_time(side, d))}
            out[P.grounded(r, ('tell', d))] = {'action': f'(tell {r} lobby_{d})', 'duration': float(P.TELL)}
        out[P.grounded(r, ('out', r))] = {'action': f'(drive {r} {room[r]} {hall[r]})', 'duration': float(P.room_time(r))}
        out[P.grounded(r, ('home', r))] = {'action': f'(drive {r} {hall[r]} {room[r]})', 'duration': float(P.room_time(r))}
        out[P.grounded(r, ('bed', r))] = {'action': f'(drive {r} {corridor[r]} {bed[r]})', 'duration': float(P.bed_time(r))}
        out[P.grounded(r, ('leave', r))] = {'action': f'(drive {r} {bed[r]} {corridor[r]})', 'duration': float(P.bed_time(r))}
    out['round_p'] = {'action': '(round p)', 'duration': 60.0}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'revision-vocabulary.pddl'), 'w').write(vocabulary())
    json.dump(mapping(), open(os.path.join(a.out, 'revision-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
