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
The muddy children, as robots at a muster point.

    muster.py --robots 4 --out DIR

writes the domain `muddy-robots.epddl` and two floors, `pa-n4.epddl` and
`silent-n4.epddl`, which differ in one line: whether the hall has a public
address the supervisor can speak over.

At the end of a shift N robots meet at the muster point. Each carries a status
lamp on its mast, which the fleet's diagnostics light when the robot's
calibration has drifted. A robot sees every other robot's lamp and not its
own. The supervisor knows that at least one lamp is lit and not which.

  announce   the supervisor says over the public address that at least one of
             them is faulty: a public announcement. With two or more faulty
             robots every robot already knows it, since each sees a lit lamp.
  bell       a bell rings, and every robot that knows it is faulty drives to the
             calibration bay. Everyone sees whether anyone left, and nobody
             leaving is as public as somebody leaving: a public sensing action
             with the two outcomes.

The goal is that every robot knows whether it is faulty. The worlds are the
2^N fault patterns, robot i relating two that differ at most in i's own lamp,
and the designated worlds are every pattern with a lit lamp, since that much
is true and the planner does not know which.

Without the announcement no number of bells helps: in the pattern with no lit
lamp nobody can ever leave, every robot considers a pattern one lamp from it,
and so on, and the bell's outcome "nobody leaves" is common knowledge before
it rings. With it, k faulty robots know after the k-th bell (Fagin, Halpern,
Moses and Vardi, 1995, ch. 1).
"""

import argparse
import os


def agents(n):
    return [f'r{i + 1}' for i in range(n)]


DOMAIN = """\
(define (domain muddy-robots)

  ;; N robots at a muster point, each with a status lamp every other robot can
  ;; see and it cannot. Written by tools/muster.py; the README says why each
  ;; part is as it is.

  (:requirements
    :typing :equality :partial-observability :list-comprehensions :lists
    :group-modalities :static-common-knowledge :negative-preconditions
    :modal-preconditions :knowing-whether
  )

  (:action-type-libraries intermediate)

  ;; The muster point, as an object: plank grounds an action once per
  ;; assignment of its parameters, and an action with none is never grounded.
  (:types hall)

  (:predicates
    ;; The robot's calibration has drifted, and its lamp is lit.
    (faulty ?i - agent)
    ;; The hall has a public address the supervisor can speak over.
    (pa)
  )

  ;--------------------THE ANNOUNCEMENT------------------
  ;
  ; Public: every robot hears it, and hears that every other robot does.

  (:event e-announce
    :parameters (?h - hall)
    :precondition (and (pa) (exists (?i - agent) (faulty ?i)))
  )

  (:event e-announce-none
    :parameters (?h - hall)
    :precondition (and (pa) (forall (?i - agent) (not (faulty ?i))))
  )

  (:action announce
    :parameters (?h - hall)
    :action-type (public-announcement (e-announce ?h) (e-announce-none ?h))
    :observability-conditions (default Fully)
  )

  ;--------------------THE BELL------------------
  ;
  ; Every robot that knows it is faulty leaves for the calibration bay, and
  ; every robot sees whether anyone left. Nobody leaving is an outcome too.

  (:event e-leave
    :parameters (?h - hall)
    :precondition (exists (?i - agent) ([?i] (faulty ?i)))
  )

  (:event e-stay
    :parameters (?h - hall)
    :precondition (forall (?i - agent) (not ([?i] (faulty ?i))))
  )

  (:action bell
    :parameters (?h - hall)
    :action-type (public-sensing (e-leave ?h) (e-stay ?h))
    :observability-conditions (default Fully)
  )
)
"""

PROBLEM = """\
(define (problem muddy-robots-{floor}-n{n})

  (:domain muddy-robots)

  ;; {n} robots at the muster point, {where}. Written by tools/muster.py.

  (:requirements
    :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:objects muster - hall)

  (:init
    (:and
      ([C. All] ({pa}))
      ;; Each robot sees every other robot's lamp, and not its own.
      (:forall (?i ?j - agent | (/= ?i ?j)) ([C. All] ([Kw. ?i] (faulty ?j))))
      (:forall (?i - agent) ([C. All] (<Kw. ?i> (faulty ?i))))
      ;; At least one lamp is lit. True, and not common knowledge.
      (or {some})
    )
  )

  ;; Every robot knows whether it is faulty. Where each then goes, the bay or
  ;; back to work, it decides on that knowledge; see the README.
  (:goal (forall (?i - agent) ([Kw. ?i] (faulty ?i))))
)
"""


def problem_text(n, floor):
    names = agents(n)
    return PROBLEM.format(
        floor=floor, n=n, agents=' '.join(names),
        pa='pa' if floor == 'pa' else 'not (pa)',
        where='with a public address' if floor == 'pa' else 'with no public address',
        some=' '.join(f'(faulty {a})' for a in names))


# ─── What the executor needs ────────────────────────────────────────────────

PDDL = """\
;; The classical half of the muddy robots, written by tools/muster.py. Nothing
;; here knows which lamps are lit, or that a robot can leave only when it knows
;; its own; that is in the EPDDL, and the executor checks it against the model.

(define (domain muddy-robots)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  hall
)

(:predicates
  (open ?h - hall)
  (announced ?h - hall)
  (rung ?h - hall)
  (dismissed ?h - hall)
)

(:durative-action announce
  :parameters (?h - hall)
  :duration (= ?duration 6)
  :condition (and (at start (open ?h)))
  :effect (and (at end (announced ?h)))
)

(:durative-action bell
  :parameters (?h - hall)
  :duration (= ?duration 8)
  :condition (and (at start (open ?h)))
  :effect (and (at end (rung ?h)))
)
)
"""

MAPPING = {
    'announce_muster': {'action': '(announce muster)', 'duration': 8.5},
    'bell_muster': {'action': '(bell muster)', 'duration': 8.0},
}


def write(out, n, executor=False):
    """The domain and both floors for n robots, and with @p executor the
    classical model and the action mapping."""
    import json
    os.makedirs(out, exist_ok=True)
    paths = {'domain': os.path.join(out, 'muddy-robots.epddl')}
    with open(paths['domain'], 'w') as fh:
        fh.write(DOMAIN)
    for floor in ('pa', 'silent'):
        paths[floor] = os.path.join(out, f'{floor}-n{n}.epddl')
        with open(paths[floor], 'w') as fh:
            fh.write(problem_text(n, floor))
    if executor:
        paths['pddl'] = os.path.join(out, 'muddy-robots.pddl')
        with open(paths['pddl'], 'w') as fh:
            fh.write(PDDL)
        paths['mapping'] = os.path.join(out, 'mapping.json')
        with open(paths['mapping'], 'w') as fh:
            json.dump(MAPPING, fh, indent=2)
            fh.write('\n')
    return paths


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', type=int, default=4)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    for role, path in write(args.out, args.robots, executor=True).items():
        print(f'{role:7s} {path}')


if __name__ == '__main__':
    main()
