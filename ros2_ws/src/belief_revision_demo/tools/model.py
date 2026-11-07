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
The EPDDL of belief revision in a changing ward: the domain and twelve problems.

    model.py --out epddl-workspace/belief-revision

writes belief-revision.epddl and instances/act<k>-<formulation>.epddl, an
explicit model per act and formulation (tools/ranks.py): one world per thing
that may have happened tonight, every courier's relation the universal one,
and each world labelled with its rank under the formulation as a grade atom
g0..g6. The grades are the only difference between the four problems of an
act. Nothing in the domain changes a grade.

Belief is a formula over knowledge and the grades. With m the least grade of
the worlds robot r cannot rule out,

    B_r phi         = OR_k ( [r](no grade below k) AND <r>g_k AND [r](g_k -> phi) )
    not B_r not phi = OR_k ( [r](no grade below k) AND <r>(g_k AND phi) )

so a look or a report, which removes worlds, revises a belief exactly as
Spohn's conditioning does, and nothing else is needed for revision. The
domain uses the second formula as a guard: a robot enters a lobby only if it
does not believe the door shut, and seeks along the hall or the corridor only
if it does not believe every door ahead shut. It crosses only a door it knows
to be open.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402

GRADES = 7
PLACES = (P.SOUTH + P.NORTH + ['j' + d for d in R.DOORS] + ['k' + d for d in R.DOORS]
          + ['hc', 'ht', 'rc', 'rd'])


def no_grade_below(k):
    if k == 0:
        return '(true)'
    return '(and ' + ' '.join(f'(not (g{j}))' for j in range(k)) + ')'


KD45 = False


def entertains(agent, phi):
    """not B_agent not phi, as EPDDL. In the KD45 control the agent's relation is its belief."""
    if KD45:
        return f'(not ([{agent}] (not {phi})))'
    parts = []
    for k in range(GRADES):
        dia = f'(not ([{agent}] (not (and (g{k}) {phi}))))'
        parts.append(dia if k == 0 else f'(and ([{agent}] {no_grade_below(k)}) {dia})')
    return '(or ' + ' '.join(parts) + ')'


def opens(doors):
    return f'(open-{doors[0]})' if len(doors) == 1 else '(or ' + ' '.join(f'(open-{d})' for d in doors) + ')'


def ontic(name, pre, eff, comment=''):
    c = f'  ;; {comment}\n' if comment else ''
    return f"""{c}  (:event e-{name}
    :parameters (?r - agent)
    :precondition {pre}
    :effects {eff}
  )

  (:action {name}
    :parameters (?r - agent)
    :action-type (public-ontic (e-{name} ?r))
    :observability-conditions (default Fully)
  )

"""


def move(a, b):
    return f'(:and (at-{b} ?r) (not (at-{a} ?r)))'


def domain():
    out = [HEADER]
    preds = ['(open-' + d + ')' for d in R.DOORS] + [f'(g{k})' for k in range(GRADES)] + \
        [f'(at-{p} ?r - agent)' for p in PLACES] + \
        ['(courier ?r - agent)', '(porter ?r - agent)', '(out ?r - agent)', '(back ?r - agent)',
         '(delivered ?r - agent)', '(home ?r - agent)', '(lives-hc ?r - agent)', '(lives-ht ?r - agent)',
         '(bed-rc ?r - agent)', '(bed-rd ?r - agent)', '(round-done)', '(reopens)']
    out.append('  (:predicates\n    ' + '\n    '.join(preds) + '\n  )\n\n  (:event nil)\n\n')

    out.append('  ;;--------------------LEAVING THE ROOM------------------\n\n')
    out.append(ontic('set-out', '(and (courier ?r) (out ?r) (or (at-hc ?r) (at-ht ?r)))',
                     '(:and (when (at-hc ?r) (:and (at-sp ?r) (not (at-hc ?r)))) '
                     '(when (at-ht ?r) (:and (at-st ?r) (not (at-ht ?r)))))'))

    out.append('  ;;--------------------SEEKING------------------\n'
               '  ;;\n  ;; A hop along the hall on the way out, or along the corridor on the way back,\n'
               '  ;; only towards a door the robot does not believe shut.\n\n')
    for side, leg in (('S', 'out'), ('N', 'back')):
        c = P.chain(side)
        for i in range(len(c)):
            for j in (i - 1, i + 1):
                if not 0 <= j < len(c):
                    continue
                doors = P.ahead(side, i, j)
                if not doors:
                    continue
                out.append(ontic(f'seek-{c[i]}-{c[j]}',
                                 f'(and (courier ?r) ({leg} ?r) (at-{c[i]} ?r) {entertains("?r", opens(doors))})',
                                 move(c[i], c[j])))

    out.append('  ;;--------------------THE LOBBIES------------------\n'
               '  ;;\n  ;; In to the bend from which the door is seen, only if the robot does not believe\n'
               '  ;; it shut; a look from the bend; back out to the mouth; through, knowing it open.\n\n')
    for d in R.DOORS:
        for side, leg, m, b, far in (('s', 'out', 'v', 'j', 'n'), ('n', 'back', 'n', 'k', 's')):
            out.append(ontic(f'enter-{d}-{side}',
                             f'(and (courier ?r) ({leg} ?r) (at-{m}{d} ?r) {entertains("?r", f"(open-{d})")})',
                             move(m + d, b + d)))
            out.append(f"""  (:event e-look-{d}-{side}-open
    :parameters (?r - agent)
    :precondition (and (at-{b}{d} ?r) (open-{d}))
  )

  (:event e-look-{d}-{side}-shut
    :parameters (?r - agent)
    :precondition (and (at-{b}{d} ?r) (not (open-{d})))
  )

  (:action look-{d}-{side}
    :parameters (?r - agent)
    :action-type (semi-private-sensing (e-look-{d}-{side}-open ?r) (e-look-{d}-{side}-shut ?r))
    :observability-conditions (:and (?r Fully) (default Partially))
  )

""")
            out.append(ontic(f'back-{d}-{side}', f'(and (courier ?r) (at-{b}{d} ?r))', move(b + d, m + d)))
        out.append(ontic(f'cross-{d}-n', f'(and (courier ?r) (out ?r) (at-j{d} ?r) (open-{d}) ([?r] (open-{d})))',
                         move('j' + d, 'n' + d)))
        out.append(ontic(f'cross-{d}-s', f'(and (courier ?r) (back ?r) (at-k{d} ?r) (open-{d}) ([?r] (open-{d})))',
                         move('k' + d, 'v' + d)))

    out.append('  ;;--------------------WALKING ON------------------\n\n')
    for side, leg in (('N', 'out'), ('S', 'back')):
        c = P.chain(side)
        for i in range(len(c)):
            for j in (i - 1, i + 1):
                if 0 <= j < len(c):
                    out.append(ontic(f'walk-{c[i]}-{c[j]}', f'(and (courier ?r) ({leg} ?r) (at-{c[i]} ?r))',
                                     move(c[i], c[j])))
    out.append(ontic('deliver', '(and (courier ?r) (out ?r) (or (and (bed-rc ?r) (at-nw ?r)) (and (bed-rd ?r) (at-nb ?r))))',
                     '(:and (delivered ?r) (when (at-nw ?r) (:and (at-rc ?r) (not (at-nw ?r)))) '
                     '(when (at-nb ?r) (:and (at-rd ?r) (not (at-nb ?r)))))'))
    out.append(ontic('leave', '(and (courier ?r) (out ?r) (delivered ?r) (round-done))',
                     '(:and (back ?r) (not (out ?r)) (when (at-rc ?r) (:and (at-nw ?r) (not (at-rc ?r)))) '
                     '(when (at-rd ?r) (:and (at-nb ?r) (not (at-rd ?r)))))'))
    out.append(ontic('arrive', '(and (courier ?r) (back ?r) (or (and (lives-hc ?r) (at-sp ?r)) (and (lives-ht ?r) (at-st ?r))))',
                     '(:and (home ?r) (when (at-sp ?r) (:and (at-hc ?r) (not (at-sp ?r)))) '
                     '(when (at-st ?r) (:and (at-ht ?r) (not (at-st ?r)))))'))

    out.append('  ;;--------------------THE RADIO------------------\n'
               '  ;;\n  ;; A robot reports a door it knows to be shut. Every robot hears it.\n\n')
    for d in R.DOORS:
        out.append(f"""  (:event e-tell-{d}
    :parameters (?r - agent)
    :precondition (and (courier ?r) (not (open-{d})) ([?r] (not (open-{d}))))
  )

  (:event e-tell-{d}-else
    :parameters (?r - agent)
    :precondition (not (and (courier ?r) (not (open-{d})) ([?r] (not (open-{d})))))
  )

  (:action tell-{d}
    :parameters (?r - agent)
    :action-type (public-announcement (e-tell-{d} ?r) (e-tell-{d}-else ?r))
    :observability-conditions (default Fully)
  )

""")
    out.append('  ;;--------------------THE PORTER AT TWO------------------\n'
               '  ;;\n  ;; Every robot knows he walks; none sees whether he opens what he shut at one.\n\n')
    out.append(ontic('round', '(and (porter ?r) (not (round-done)) (forall (?i - agent) (imply (courier ?i) (delivered ?i))))',
                     '(:and (round-done) (when (reopens) (:and (open-a) (open-b) (open-c) (open-d))))'))
    out.append(')\n')
    return ''.join(out)


HEADER = """;; BELIEF REVISION IN A CHANGING WARD. Written by tools/model.py in
;; belief_revision_demo; do not edit by hand.
;;
;; A hospital floor at night. Four fire doors, A to D from west to east, close
;; four lobbies through a band of linen rooms between the hall and the corridor;
;; each lobby turns once, so its door is seen only from a bend inside it. At one
;; the night porter walks a loop, north through one door and south through
;; another, shutting both. Nobody sees him. At two he walks it back and opens
;; them again, unless he is held up.
;;
;; The couriers know every loop is possible and how plausible each is: every
;; world carries its Spohn rank as a grade atom (g0..g6), fixed when the
;; problem is written and never changed by an action. Belief is a formula over
;; knowledge and the grades (tools/model.py), and revision is what a look or a
;; report does to knowledge: it removes worlds, and the least grade among the
;; worlds left says what is believed. Every relation stays an equivalence.
;;
;; A courier enters a lobby only if it does not believe its door shut, seeks
;; along the hall or the corridor only if it does not believe every door ahead
;; shut, and crosses only a door it knows to be open.

(define (domain belief-revision)

  (:requirements
    :equality :partial-observability :ontic-actions :negative-preconditions
    :modal-preconditions :conditional-effects :universal-preconditions :disjunctive-preconditions
  )

  (:action-type-libraries intermediate)

"""


def problem(act, formulation):
    worlds = R.worlds(act)
    names = [f'w{i}' for i in range(len(worlds))]
    agents = P.ROBOTS[act] + (['p'] if act == 3 else [])
    lines = [f';; BELIEF REVISION IN A CHANGING WARD, act {act}, ranked by {formulation}. Written by tools/model.py.',
             ';; Upper case is a door the porter shut at one; + he reopens them at two, - he is held up.']
    for n, w in zip(names, worlds):
        lines.append(f';;   {n}: {R.name(w):4s} rank {R.rank(formulation, w)}')
    rel = []
    for a in agents:
        if a == 'p':
            pairs = ' '.join(f'({n} {n})' for n in names)
        else:
            pairs = ' '.join(f'({u} {v})' for u in names for v in names)
        rel.append(f'      {a} (:and {pairs})')
    labels = []
    for n, w in zip(names, worlds):
        facts = [f'(open-{d})' for d in R.DOORS if R.open_at(w, d, False)]
        facts.append(f'(g{R.rank(formulation, w)})')
        if w[1] == 'reopen':
            facts.append('(reopens)')
        for r in P.ROBOTS[act]:
            facts += [f'(courier {r})', f'(out {r})', f'(at-{P.ROOM[r][2]} {r})', f'(lives-{P.ROOM[r][2]} {r})',
                      f'(bed-{P.WARD[r][2]} {r})']
        if act == 3:
            facts.append('(porter p)')
        labels.append(f"      {n} (:and {' '.join(facts)})")
    goal = ' '.join(f'(delivered {r})' if act < 3 else f'(home {r})' for r in P.ROBOTS[act])
    return '\n'.join(lines) + f"""

(define (problem act{act}-{formulation})
  (:domain belief-revision)
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


def kd45_problem():
    """Act 1 without grades: the courier's relation points at the rostered night only,
    a KD45 belief, and nothing ranks the rest."""
    text = problem(1, 'explanation')
    worlds = R.worlds(1)
    names = [f'w{i}' for i in range(len(worlds))]
    top = names[worlds.index((R.ROSTERED, None))]
    pairs = ' '.join(f'({u} {top})' for u in names)
    import re
    text = re.sub(r'      c \(:and [^\n]*\)', f'      c (:and {pairs})', text)
    text = text.replace('(define (problem act1-explanation)', '(define (problem act1-kd45)')
    text = text.replace(':requirements :general-frames', ':requirements :KD45-frames :general-frames')
    text = re.sub(r' \(g\d\)', '', text)
    return text.replace(';; BELIEF REVISION IN A CHANGING WARD, act 1, ranked by explanation.',
                        ';; BELIEF REVISION IN A CHANGING WARD, act 1, the KD45 control: the courier believes the\n'
                        ';; rostered night and nothing ranks the others.')


def main():
    global KD45
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'kd45'), exist_ok=True)
    KD45 = True
    open(os.path.join(a.out, 'kd45', 'belief-revision-kd45.epddl'), 'w').write(
        domain().replace('(define (domain belief-revision)', '(define (domain belief-revision-kd45)'))
    open(os.path.join(a.out, 'kd45', 'act1-kd45.epddl'), 'w').write(
        kd45_problem().replace('(:domain belief-revision)', '(:domain belief-revision-kd45)'))
    KD45 = False
    os.makedirs(os.path.join(a.out, 'instances'), exist_ok=True)
    open(os.path.join(a.out, 'belief-revision.epddl'), 'w').write(domain())
    for act in (1, 2, 3):
        for f in R.FORMULATIONS:
            open(os.path.join(a.out, 'instances', f'act{act}-{f}.epddl'), 'w').write(problem(act, f))


if __name__ == '__main__':
    main()
