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
The classical side of the link grades, and the executor's vocabulary.

    make_pddl.py --out pddl/

link-classical.pddl   the robots' model in the classical formulations. What a
                      robot knows is a fact, (knows ?r): it has decided the
                      corridor, by looking from the mouth or by being told
                      over the link, which report needs: (link). Each drive
                      lasts what its route takes at layout.SPEED.
link-vocabulary.pddl  the executor's vocabulary under the epistemic policy:
                      one action per thing a robot does, each touching only
                      its robots' own fact, so that the executor may dispatch
                      two robots' actions together. What decides the policy
                      is in the EPDDL.
link-mapping.json     the EPDDL's grounded actions as vocabulary actions.

problem() gives a classical problem; problem_vocabulary() the vocabulary's.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

INSPECT = 5
REPORT = 5
# Each robot's drive to its spot, named for where it starts.
DRIVE = {'r1': 'drive_dock', 'r2': 'drive_station'}
START = {'r1': 'at_dock', 'r2': 'at_station'}

VOCABULARY = """;; THE LINK GRADES: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain link-grades)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (robot_ok ?r - robot))
{actions})
"""
VOCAB_ACTION = """
(:durative-action {name}
    :parameters ({params})
    :duration ( = ?duration 10)
    :condition (and {cond})
    :effect (and {eff}))
"""

CLASSICAL = """;; THE LINK GRADES: the corridor on a link that may fall, for a classical planner.
;; Written by tools/make_pddl.py.
;;
;; What a robot knows is recorded as a fact, (knows ?r): it has decided the
;; corridor. A robot decides it by looking from the corridor's mouth, or is
;; told by one that has, which takes the link: report needs (link). Whether
;; (link) is in the problem is the formulation's.
(define (domain link-grades)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates
  (at_dock ?r - robot)
  (at_station ?r - robot)
  (at_spot ?r - robot)
  (knows ?r - robot)
  (link)
)
{drives}
(:durative-action inspect
  :parameters (?r - robot)
  :duration (= ?duration {inspect})
  :condition (and (over all (at_spot ?r)))
  :effect (and (at end (knows ?r))))

(:durative-action report
  :parameters (?from ?to - robot)
  :duration (= ?duration {report})
  :condition (and (at start (knows ?from)) (over all (link)))
  :effect (and (at end (knows ?to))))
)
"""
DRIVE_ACTION = """
(:durative-action {name}
  :parameters (?r - robot)
  :duration (= ?duration {d})
  :condition (and (at start ({start} ?r)))
  :effect (and (at start (not ({start} ?r))) (at end (at_spot ?r))))
"""


def vocabulary():
    acts = []
    for r in ('r1', 'r2'):
        acts.append(VOCAB_ACTION.format(name=DRIVE[r], params='?r - robot', cond='(at start (robot_ok ?r))',
                                        eff='(at end (robot_ok ?r))'))
    acts.append(VOCAB_ACTION.format(name='inspect', params='?r - robot', cond='(at start (robot_ok ?r))',
                                    eff='(at end (robot_ok ?r))'))
    acts.append(VOCAB_ACTION.format(name='share', params='?a ?b - robot',
                                    cond='(at start (robot_ok ?a)) (at start (robot_ok ?b))',
                                    eff='(at end (robot_ok ?a)) (at end (robot_ok ?b))'))
    return VOCABULARY.format(actions=''.join(acts))


def classical():
    drives = ''.join(DRIVE_ACTION.format(name=DRIVE[r], d=L.trip(r), start=START[r]) for r in ('r1', 'r2'))
    return CLASSICAL.format(drives=drives, inspect=INSPECT, report=REPORT)


def mapping():
    out = {f'goto-junction_{r}': {'action': f'({DRIVE[r]} {r})', 'duration': float(L.trip(r))} for r in ('r1', 'r2')}
    out.update({f'inspect_{r}': {'action': f'(inspect {r})', 'duration': float(INSPECT)} for r in ('r1', 'r2')})
    out.update({f'share_{a}_{b}': {'action': f'(share {a} {b})', 'duration': float(REPORT)}
                for a, b in (('r1', 'r2'), ('r2', 'r1'))})
    return out


def problem(facts, goal=('(knows r1)', '(knows r2)')):
    return (f"(define (problem link-grades) (:domain link-grades)\n"
            f"(:objects r1 r2 - robot)\n(:init {' '.join(facts)})\n(:goal (and {' '.join(goal)})))\n")


def problem_start(link):
    return problem(['(at_dock r1)', '(at_station r2)'] + (['(link)'] if link else []))


def problem_vocabulary():
    return problem(['(robot_ok r1)', '(robot_ok r2)'], goal=('(robot_ok r1)',))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'link-vocabulary.pddl'), 'w').write(vocabulary())
    open(os.path.join(a.out, 'link-classical.pddl'), 'w').write(classical())
    json.dump(mapping(), open(os.path.join(a.out, 'link-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
