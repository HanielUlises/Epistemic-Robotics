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
The EPDDL of belief revision in a mixed fleet: the domain and the problems.

    model.py --out epddl-workspace/mixed-fleet

writes mixed-fleet.epddl and instances/act<k>.epddl: an explicit model per act,
one world per loop the porter may have walked at one, every robot's relation
the universal one, and each world labelled, for every robot r, with r's rank
of it as a grade atom (rank0 r) .. (rank8 r). The robots rank by their makers'
rankings (tools/ranks.py); who takes whose word is a fact, (takes-word l s).

Belief is a formula over knowledge and a robot's own grades. With m the least
grade robot r has among the worlds it cannot rule out,

    B_r phi         = OR_k ( [r](no grade of r below k) AND <r>(rank_k r) AND [r]((rank_k r) -> phi) )
    not B_r not phi = OR_k ( [r](no grade of r below k) AND <r>((rank_k r) AND phi) )

so a look or a report of evidence, which removes worlds, revises every robot's
belief by Spohn's conditioning, each on its own ranking. A belief report is
the one action that changes a grade: robot s says it believes phi, which it
may only if it believes phi, does not know it, and believes that the listener
l does not believe it (a second-order guard); every robot hears it, and if l
takes s's word, l's grade rises by ALPHA in every world where phi is false,
capped at the highest grade. That is Spohn conditioning on evidence of
strength ALPHA, done by conditional effects on the grade atoms; every relation
stays an equivalence.

A robot enters a lobby only if it does not believe its door shut, seeks along
the hall only if it does not believe every door ahead shut, and crosses only a
door it knows to be open.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402

G = R.GRADES
ROOMS = ('hd', 'ht', 'hp')
PLACES = list(ROOMS) + P.SOUTH + ['j' + d for d in R.DOORS] + ['n' + d for d in R.DOORS]


def no_grade_below(k, r):
    if k == 0:
        return '(true)'
    return '(and ' + ' '.join(f'(not (rank{j} {r}))' for j in range(k)) + ')'


def believes(r, phi):
    parts = []
    for k in range(G):
        dia = f'(not ([{r}] (not (rank{k} {r}))))'
        box = f'([{r}] (imply (rank{k} {r}) {phi}))'
        parts.append(f'(and {dia} {box})' if k == 0 else f'(and ([{r}] {no_grade_below(k, r)}) {dia} {box})')
    return '(or ' + ' '.join(parts) + ')'


def entertains(r, phi):
    """not B_r not phi."""
    parts = []
    for k in range(G):
        dia = f'(not ([{r}] (not (and (rank{k} {r}) {phi}))))'
        parts.append(dia if k == 0 else f'(and ([{r}] {no_grade_below(k, r)}) {dia})')
    return '(or ' + ' '.join(parts) + ')'


def knows(r, phi):
    return f'([{r}] {phi})'


def opens(doors):
    return f'(open-{doors[0]})' if len(doors) == 1 else '(or ' + ' '.join(f'(open-{d})' for d in doors) + ')'


def literal(d, value):
    return f'(open-{d})' if value else f'(not (open-{d}))'


def ontic(name, pre, eff, params='?r - agent', args='?r', comment=''):
    c = f'  ;; {comment}\n' if comment else ''
    return f"""{c}  (:event e-{name}
    :parameters ({params})
    :precondition {pre}
    :effects {eff}
  )

  (:action {name}
    :parameters ({params})
    :action-type (public-ontic (e-{name} {args}))
    :observability-conditions (default Fully)
  )

"""


def move(a, b):
    return f'(:and (at-{b} ?r) (not (at-{a} ?r)))'


def domain():
    out = [HEADER]
    preds = ['(open-' + d + ')' for d in R.DOORS] + [f'(rank{k} ?r - agent)' for k in range(G)] + \
        [f'(at-{p} ?r - agent)' for p in PLACES] + \
        ['(robot ?r - agent)', '(searching ?r - agent)', '(delivered ?r - agent)',
         '(takes-word ?l - agent ?s - agent)']
    out.append('  (:predicates\n    ' + '\n    '.join(preds) + '\n  )\n\n  (:event nil)\n\n')

    out.append('  ;;--------------------LEAVING THE ROOM------------------\n\n')
    eff = ' '.join(f'(when (at-{h} ?r) (:and (at-{P.SOUTH[P.ROOM_AT[h]]} ?r) (not (at-{h} ?r))))' for h in ROOMS)
    out.append(ontic('set-out', '(and (robot ?r) (not (searching ?r)) (not (delivered ?r)) '
                     '(or ' + ' '.join(f'(at-{h} ?r)' for h in ROOMS) + '))',
                     f'(:and (searching ?r) {eff})'))

    out.append('  ;;--------------------SEEKING------------------\n'
               '  ;;\n  ;; A hop along the hall, only towards a door the robot does not believe shut.\n\n')
    c = P.SOUTH
    for i in range(len(c)):
        for j in (i - 1, i + 1):
            if not 0 <= j < len(c):
                continue
            doors = P.ahead(i, j)
            if not doors:
                continue
            out.append(ontic(f'seek-{c[i]}-{c[j]}',
                             f'(and (searching ?r) (at-{c[i]} ?r) {entertains("?r", opens(doors))})',
                             move(c[i], c[j])))

    out.append('  ;;--------------------THE LOBBIES------------------\n'
               '  ;;\n  ;; In to the bend from which the door is seen, only if the robot does not believe\n'
               '  ;; it shut; a look from the bend; back out to the mouth; through, knowing it open.\n\n')
    for d in R.DOORS:
        out.append(ontic(f'enter-{d}-s', f'(and (searching ?r) (at-v{d} ?r) {entertains("?r", f"(open-{d})")})',
                         move('v' + d, 'j' + d)))
        out.append(f"""  (:event e-look-{d}-s-open
    :parameters (?r - agent)
    :precondition (and (at-j{d} ?r) (open-{d}))
  )

  (:event e-look-{d}-s-shut
    :parameters (?r - agent)
    :precondition (and (at-j{d} ?r) (not (open-{d})))
  )

  (:action look-{d}-s
    :parameters (?r - agent)
    :action-type (semi-private-sensing (e-look-{d}-s-open ?r) (e-look-{d}-s-shut ?r))
    :observability-conditions (:and (?r Fully) (default Partially))
  )

""")
        out.append(ontic(f'back-{d}-s', f'(and (robot ?r) (at-j{d} ?r))', move('j' + d, 'v' + d)))
        out.append(ontic(f'cross-{d}-n', f'(and (searching ?r) (at-j{d} ?r) (open-{d}) ([?r] (open-{d})))',
                         f'(:and (at-n{d} ?r) (not (at-j{d} ?r)) (not (searching ?r)))'))
        out.append(ontic(f'deliver-{d}', f'(and (robot ?r) (at-n{d} ?r) (not (delivered ?r)))',
                         f'(:and (delivered ?r) (not (at-n{d} ?r)))'))

    out.append('  ;;--------------------REPORTS OF EVIDENCE------------------\n'
               '  ;;\n  ;; A robot reports a door it knows to be shut. Every robot hears it.\n\n')
    for d in R.DOORS:
        out.append(f"""  (:event e-tell-{d}
    :parameters (?r - agent)
    :precondition (and (robot ?r) (not (open-{d})) ([?r] (not (open-{d}))))
  )

  (:event e-tell-{d}-else
    :parameters (?r - agent)
    :precondition (not (and (robot ?r) (not (open-{d})) ([?r] (not (open-{d})))))
  )

  (:action tell-{d}
    :parameters (?r - agent)
    :action-type (public-announcement (e-tell-{d} ?r) (e-tell-{d}-else ?r))
    :observability-conditions (default Fully)
  )

""")

    out.append('  ;;--------------------REPORTS OF BELIEF------------------\n'
               '  ;;\n'
               '  ;; Robot ?s says it believes a door shut (or open): it believes so, does not know it,\n'
               '  ;; and believes that ?l, which takes its word and is still searching, does not\n'
               '  ;; believe it. Every robot hears it; ?l\'s grade rises by the trust in every world\n'
               '  ;; the report rules out.\n\n')
    for d in R.DOORS:
        for value in (False, True):
            phi = literal(d, value)
            pre = (f'(and (robot ?s) (takes-word ?l ?s) (searching ?l) {believes("?s", phi)} '
                   f'(not {knows("?s", phi)}) {believes("?s", "(not " + believes("?l", phi) + ")")})')
            # one assignment per grade atom (iff): plank conjoins several conditional
            # effects on one atom, which loses an atom one adds and another deletes
            keep, move_ = phi, literal(d, not value)
            shifts = []
            for t in range(G):
                src = [k for k in range(G) if min(k + R.ALPHA, G - 1) == t and k != t]
                stay = f'(and {keep} (rank{t} ?l))'
                if t == G - 1:
                    stay = f'(rank{t} ?l)' if not src else stay
                    srcs = src + [t]
                else:
                    srcs = src
                if not srcs:
                    cond = stay
                else:
                    into = ' '.join(f'(rank{k} ?l)' for k in srcs)
                    into = into if len(srcs) == 1 else f'(or {into})'
                    cond = f'(or {stay} (and {move_} {into}))'
                shifts.append(f'(iff {cond} (rank{t} ?l))')
            name = f"believe-{d}-{'open' if value else 'shut'}"
            out.append(ontic(name, pre, '(:and ' + ' '.join(shifts) + ')', params='?s - agent ?l - agent',
                             args='?s ?l'))
    out.append(')\n')
    return ''.join(out)


HEADER = """;; BELIEF REVISION IN A MIXED FLEET. Written by tools/model.py in
;; mixed_fleet_demo; do not edit by hand.
;;
;; A hospital floor at night. Four fire doors, A to D from west to east, close
;; four lobbies through a band of linen rooms between the hall and the corridor;
;; each lobby turns once, so its door is seen only from a bend inside it. At one
;; the night porter walks a loop, north through one door and south through
;; another, shutting both. Nobody sees him.
;;
;; The fleet is mixed: robots of different makers, each ranking the six loops its
;; own way. Every world carries, for every robot, that robot's Spohn rank of it as
;; a grade atom (rank0 r .. rank8 r). Belief is a formula over knowledge and a
;; robot's own grades: a look or a report of evidence removes worlds, and each
;; robot's least grade among those left says what it believes. A report of
;; belief changes grades: the listener, if it takes the speaker's word, raises
;; its grade of every world the report rules out. Every relation stays an
;; equivalence.

(define (domain mixed-fleet)

  (:requirements
    :equality :partial-observability :ontic-actions :negative-preconditions
    :modal-preconditions :conditional-effects :universal-preconditions :disjunctive-preconditions
  )

  (:action-type-libraries intermediate)

"""


def problem(act):
    worlds = R.worlds()
    names = [f'w{i}' for i in range(len(worlds))]
    fleet = R.fleet(act)
    agents = [r.name for r in fleet]
    lines = [f';; BELIEF REVISION IN A MIXED FLEET, act {act}. Written by tools/model.py.',
             ';; Upper case is a door the porter shut at one; each world\'s ranks by robot.',
             ';; ' + ', '.join(f'{r.name} ranks by {r.ranking} and starts in {r.room} for ward {r.ward}' for r in fleet),
             ';; takes the word of: ' + (', '.join(f'{l} of {s}' for l, s in sorted(R.trust(act))) or 'nobody')]
    for n, w in zip(names, worlds):
        lines.append(f';;   {n}: {R.name(w)}  ' + ' '.join(f'{r.name}:{R.rank(r.ranking, w)}' for r in fleet))
    rel = []
    for a in agents:
        pairs = ' '.join(f'({u} {v})' for u in names for v in names)
        rel.append(f'      {a} (:and {pairs})')
    labels = []
    for n, w in zip(names, worlds):
        facts = [f'(open-{d})' for d in R.DOORS if R.open_at(w, d)]
        for r in fleet:
            facts += [f'(rank{R.rank(r.ranking, w)} {r.name})', f'(robot {r.name})', f'(at-{r.room} {r.name})']
        facts += [f'(takes-word {l} {s})' for l, s in sorted(R.trust(act))]
        labels.append(f"      {n} (:and {' '.join(facts)})")
    goal = ' '.join(f'(delivered {r})' for r in agents)
    return '\n'.join(lines) + f"""

(define (problem act{act})
  (:domain mixed-fleet)
  (:requirements :general-frames :multi-pointed-models :modal-goals :negative-preconditions)
  (:agents {' '.join(agents)})
  (:init
    :worlds ({' '.join(names)})
    :relations (
{chr(10).join(rel)}
    )
    :labels (
{chr(10).join(labels)}
    )
    :designated ({' '.join(names)})
  )
  (:goal (and {goal}))
)
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'instances'), exist_ok=True)
    open(os.path.join(a.out, 'mixed-fleet.epddl'), 'w').write(domain())
    for act in sorted(R.FLEETS):
        open(os.path.join(a.out, 'instances', f'act{act}.epddl'), 'w').write(problem(act))


if __name__ == '__main__':
    main()
