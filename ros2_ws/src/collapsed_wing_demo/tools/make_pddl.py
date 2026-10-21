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
The classical side of the collapsed wing, and the executor's vocabulary.

    make_pddl.py --out pddl/

wing-optimistic.pddl  the classical model with every way assumed clear: the
                      medic may cross a way while (assumed_clear_<d>) holds,
                      and the mission removes the fact when the medic's own
                      laser finds rubble on it, puts the medic in that way, and
                      replans; retreat_<d> brings it back to the hall.
wing-gated.pddl       the classical model with the medic's crossing gated on a
                      fact a scout's check gives, (checked_clear_<d>): the
                      epistemic precondition written as a fact. A check is
                      optimistic, and the mission replans when it reports
                      rubble. Nothing checks the middle passage, so nothing
                      gives (checked_clear_m).
wing-vocabulary.pddl  the executor's vocabulary under the epistemic policy:
                      one action per thing a robot does, each touching only its
                      own robot's fact, so that the executor may dispatch two
                      robots' actions together. What decides the policy is in
                      the EPDDL.
wing-mapping.json     the EPDDL's grounded actions as vocabulary actions.

No condition is negative: POPF through PlanSys2 does not take them. Each drive
lasts what its route takes at layout.SPEED.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

LOOK = 5
TELL = 2
SIDE = {'e': 's1', 'w': 's2'}          # which scout reaches which doorway
WAYS = ('w', 'm', 'e')
RETREAT = {'w': ['below_w', 'wing_w', 'hd_w', 'hall'], 'e': ['below_e', 'wing_e', 'hd_e', 'hall'],
           'm': ['p3', 'p2', 'p1', 'p0', 'hall_e', 'hall']}
VOCAB = ['goto_e', 'goto_w', 'inspect_e', 'inspect_w', 'tell_w', 'tell_m', 'tell_e', 'cross_w', 'cross_m',
         'cross_e']


def retreat_length(d):
    nodes = L.roadmap()
    return sum(L.math.dist(nodes[a], nodes[b]) for a, b in zip(RETREAT[d], RETREAT[d][1:]))


def durative(name, params, d, cond, eff):
    return (f'\n(:durative-action {name}\n  :parameters ({params})\n  :duration (= ?duration {d})\n'
            f'  :condition (and {cond})\n  :effect (and {eff}))\n')


def vocabulary():
    # (radio) is what a message changes: every tell writes it and every crossing
    # reads it, so that the executor, which dispatches together the actions of
    # the policy that neither writes what the other reads, never sends the medic
    # before the message its crossing waits for. Driving and looking leave it alone.
    acts = ''
    for n in VOCAB:
        radio = n.startswith(('tell_', 'cross_'))
        cond = '(at start (robot_ok ?r))' + (' (at start (radio))' if radio else '')
        eff = '(at end (robot_ok ?r))' + (' (at end (radio))' if n.startswith('tell_') else '')
        acts += durative(n, '?r - robot', 10, cond, eff)
    return (';; THE COLLAPSED WING: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain collapsed-wing)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates (robot_ok ?r - robot) (radio))\n{acts})\n')


def classical(gated):
    gate = 'checked_clear' if gated else 'assumed_clear'
    preds = ['(at_hall ?r - robot)', '(with_victim ?r - robot)']
    preds += [f'({gate}_{d})' for d in WAYS] + [f'(in_way_{d} ?r - robot)' for d in WAYS]
    acts = ''
    if gated:
        preds += ['(at_entry_e ?r - robot)', '(at_entry_w ?r - robot)', '(at_vantage_e ?r - robot)',
                  '(at_vantage_w ?r - robot)', '(unchecked_e)', '(unchecked_w)']
        for d, s in SIDE.items():
            acts += durative(f'goto_{d}', '?r - robot', L.trip(s), f'(at start (at_entry_{d} ?r))',
                             f'(at start (not (at_entry_{d} ?r))) (at end (at_vantage_{d} ?r))')
            acts += durative(f'check_{d}', '?r - robot', LOOK,
                             f'(at start (unchecked_{d})) (over all (at_vantage_{d} ?r))',
                             f'(at start (not (unchecked_{d}))) (at end (checked_clear_{d}))')
    for d in WAYS:
        acts += durative(f'cross_{d}', '?r - robot', L.trip(f'm_{d}'),
                         f'(at start (at_hall ?r)) (over all ({gate}_{d}))',
                         f'(at start (not (at_hall ?r))) (at end (with_victim ?r))')
        acts += durative(f'retreat_{d}', '?r - robot', round(retreat_length(d) / L.SPEED),
                         f'(at start (in_way_{d} ?r))',
                         f'(at start (not (in_way_{d} ?r))) (at end (at_hall ?r))')
    kind = 'gated on a scout\'s check' if gated else 'every way assumed clear'
    plist = '\n  '.join(dict.fromkeys(preds))
    return (f';; THE COLLAPSED WING for a classical planner, {kind}. Written by tools/make_pddl.py.\n'
            '(define (domain collapsed-wing)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates\n  {plist}\n)\n{acts})\n')


def mapping():
    out = {}
    for d, s in SIDE.items():
        out[f'goto-{d}_{s}'] = {'action': f'(goto_{d} {s})', 'duration': float(L.trip(s))}
        out[f'inspect-{d}_{s}'] = {'action': f'(inspect_{d} {s})', 'duration': float(LOOK)}
    for d in WAYS:
        for s in ('s1', 's2'):
            out[f'tell-{d}_{s}'] = {'action': f'(tell_{d} {s})', 'duration': float(TELL)}
        out[f'cross-{d}_m'] = {'action': f'(cross_{d} m)', 'duration': float(L.trip(f'm_{d}'))}
    return out


def problem(facts, goal=('(with_victim m)',)):
    return (f"(define (problem collapsed-wing) (:domain collapsed-wing)\n"
            f"(:objects s1 s2 m - robot)\n(:init {' '.join(facts)})\n(:goal (and {' '.join(goal)})))\n")


def problem_start(gated):
    if gated:
        return problem(['(at_hall m)', '(at_entry_e s1)', '(at_entry_w s2)', '(unchecked_e)', '(unchecked_w)'])
    return problem(['(at_hall m)'] + [f'(assumed_clear_{d})' for d in WAYS])


def problem_vocabulary():
    return problem(['(robot_ok s1)', '(robot_ok s2)', '(robot_ok m)', '(radio)'], goal=('(robot_ok m)',))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'wing-vocabulary.pddl'), 'w').write(vocabulary())
    open(os.path.join(a.out, 'wing-optimistic.pddl'), 'w').write(classical(False))
    open(os.path.join(a.out, 'wing-gated.pddl'), 'w').write(classical(True))
    json.dump(mapping(), open(os.path.join(a.out, 'wing-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
