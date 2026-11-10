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

fleet-vocabulary.pddl   one action per kind of thing a robot does: (drive r
                        from to) along the hall, out of a room or along the
                        corridor to a bed; (passage r p from to) into, out of
                        or through a passage; (backout r p from to), a
                        bed-mover backing out; (look r p view), a reading from
                        a mouth or from the mark; (say r p word), a reading
                        said on the radio (open and readshut for what a look
                        from a mouth read, shut and sure for what is known). A robot's places carry its name: it
                        keeps to its own line along the hall, so two robots'
                        drives name nothing in common and ePlanSys dispatches
                        them together. The facts are there for ePlanSys's
                        parallel groups only: an action in a passage holds it,
                        so two robots are never dispatched into one together,
                        nor one read while a robot drives in it; and a report
                        writes (radio), which every action reads, so each goes
                        alone. What decides the policy is in the EPDDL.
fleet-mapping-act<k>.json  every grounded action of an act's EPDDL as a vocabulary
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

WORDS = ['open', 'readshut'] + [f'{w}{k}' for w in ('open', 'readshut') for k in range(1, R.LOOKS + 1)] + ['shut', 'sure']
VIEWS = ['far'] + [f'far{k}' for k in range(1, R.LOOKS + 1)] + ['near']


def durative(name, params, cond, eff):
    return (f'\n(:durative-action {name}\n  :parameters ({params})\n  :duration (= ?duration 10)\n'
            f'  :condition (and {cond})\n  :effect (and {eff}))\n')


def vocabulary():
    acts = durative('drive', '?r - robot ?a - place ?b - place', '(at start (robot_ok ?r)) (at start (radio))',
                    '(at end (robot_ok ?r))')
    held = '(at start (robot_ok ?r)) (at start (radio)) (at start (passage_free ?p))'
    holds = '(at end (robot_ok ?r)) (at end (passage_free ?p))'
    acts += durative('passage', '?r - robot ?p - passage ?a - place ?b - place', held, holds)
    acts += durative('backout', '?r - robot ?p - passage ?a - place ?b - place', held, holds)
    acts += durative('look', '?r - robot ?p - passage ?v - view', held, holds)
    acts += durative('say', '?r - robot ?p - passage ?w - word', '(at start (robot_ok ?r)) (at start (radio))',
                     '(at end (robot_ok ?r)) (at end (radio))')
    return (';; A FAULTY SENSOR IN THE FLEET: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain faulty-sensor)\n(:requirements :strips :typing :durative-actions)\n'
            '(:types robot place passage view word)\n'
            '(:predicates (robot_ok ?r - robot) (radio) (passage_free ?p - passage))\n' + acts + ')\n')


def place(name, r):
    return f'{name}_{r}'


def problem(robots):
    robots = list(robots)
    places = sorted(place(n, r) for r in robots for n in L.roadmap(r)[0])
    passages = [f'pass_{d}' for d in L.DOORS]
    views = [f'{v}_{d}' for v in VIEWS for d in L.DOORS]
    init = [f'(robot_ok {r})' for r in robots] + ['(radio)'] + [f'(passage_free {x})' for x in passages]
    return (f"(define (problem faulty-sensor) (:domain faulty-sensor)\n"
            f"(:objects {' '.join(robots)} - robot {' '.join(places)} - place {' '.join(passages)} - passage "
            f"{' '.join(views)} - view {' '.join(WORDS)} - word)\n(:init {' '.join(init)})\n"
            f"(:goal (and (robot_ok {robots[0]}))))\n")


def mapping(act):
    ROBOT = {r.name: r for r in R.fleet(act)}
    out = {}

    def put(r, a, f, text):
        out[P.grounded(r, a, f)] = {'action': text, 'duration': float(P.duration(r, a, ROBOT))}

    for r in ROBOT:
        rb = ROBOT[r]
        c = P.HALL
        f0 = 'sensors'
        put(r, ('out',), f0, f'(drive {r} {place(rb.room, r)} {place(c[P.ROOM_AT[rb.room]], r)})')
        for i in range(len(c)):
            for j in (i - 1, i + 1):
                if 0 <= j < len(c):
                    put(r, ('hop', i, j), f0, f'(drive {r} {place(c[i], r)} {place(c[j], r)})')
        for d in L.DOORS:
            p, v, m, n, e = f'pass_{d}', place('v' + d, r), place('m' + d, r), place('n' + d, r), place('e' + d, r)
            put(r, ('check', d), f0, f'(look {r} {p} near_{d})')
            put(r, ('say', d, 'shut'), f0, f'(say {r} {p} shut)')
            put(r, ('say', d, 'sure'), f0, f'(say {r} {p} sure)')
            if rb.role == 'scout':
                put(r, ('look', d, 1), 'sensors', f'(look {r} {p} far_{d})')
                put(r, ('say', d, 'open', 1), 'sensors', f'(say {r} {p} open)')
                put(r, ('say', d, 'read-shut', 1), 'sensors', f'(say {r} {p} readshut)')
                for k in range(1, R.LOOKS + 1):
                    put(r, ('look', d, k), 'readings', f'(look {r} {p} far{k}_{d})')
                    put(r, ('say', d, 'open', k), 'readings', f'(say {r} {p} open{k})')
                    put(r, ('say', d, 'read-shut', k), 'readings', f'(say {r} {p} readshut{k})')
                put(r, ('advance', d), f0, f'(passage {r} {p} {v} {m})')
                put(r, ('retire', d), f0, f'(passage {r} {p} {m} {v})')
            else:
                for f in R.FORMULATIONS:
                    put(r, ('enter', d), f, f'(passage {r} {p} {v} {m})')
                put(r, ('cross', d), f0, f'(passage {r} {p} {m} {n})')
                put(r, ('reverse', d), f0, f'(backout {r} {p} {m} {e})')
                put(r, ('deliver', d), f0, f'(drive {r} {n} {place(f"bed{rb.ward}", r)})')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for f in os.listdir(a.out):
        os.remove(os.path.join(a.out, f))
    open(os.path.join(a.out, 'fleet-vocabulary.pddl'), 'w').write(vocabulary())
    for act in sorted(R.FLEETS):
        json.dump(mapping(act), open(os.path.join(a.out, f'fleet-mapping-act{act}.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
