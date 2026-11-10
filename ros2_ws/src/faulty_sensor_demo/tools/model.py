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
The EPDDL of a faulty sensor in the fleet: two domains and their problems.

    model.py --out epddl-workspace/faulty-sensor

writes faulty-sensor.epddl (an error belongs to a sensor), faulty-readings.epddl
(an error belongs to a reading) and instances/act<k>-<sensors|readings>.epddl:
an explicit model per act, one world per world of the model of error
(tools/ranks.py), every robot's relation the universal one, and each world
labelled with its rank as a grade atom (rank0) .. (rank5).

Belief is a formula over knowledge and the grades. With m the least grade
among the worlds robot r cannot rule out,

    B^k_r phi = OR_m ( [r](no grade below m) AND <r>(rank_m)
                       AND [r]((rank_m OR .. OR rank_(m+k-1)) -> phi) )

so B^1 is belief and B^2 firm belief of degree 2. A reading removes worlds,
for its reader when it is taken and for every robot when it is said, and the
least grade among those left says what is believed, and how firmly.

A look from a mouth is a sensing action with two events, "shut" where the
door is shut and the reading does not err, "open" otherwise: where the door
is open, or where the reader is short-sighted (sensors) or this look errs
(readings). A reading at the mark has the door's state for its precondition.
A bed-mover drives into a passage by one of three actions, which differ in
their guard alone: believing the door open, believing it firmly, knowing it.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402

G = R.GRADES
ROOMS = ('hd', 'ld', 'ht', 'hp')
PLACES = list(ROOMS) + P.HALL + ['m' + d for d in R.DOORS] + ['n' + d for d in R.DOORS]
DOMAIN = {'sensors': 'faulty-sensor', 'readings': 'faulty-readings'}


def no_grade_below(k):
    if k == 0:
        return '(true)'
    return '(and ' + ' '.join(f'(not (rank{j}))' for j in range(k)) + ')'


def believes(r, phi, k=1):
    """B^k_r phi."""
    parts = []
    for m in range(G):
        band = [f'(rank{j})' for j in range(m, min(m + k, G))]
        band = band[0] if len(band) == 1 else '(or ' + ' '.join(band) + ')'
        dia = f'(not ([{r}] (not (rank{m}))))'
        box = f'([{r}] (imply {band} {phi}))'
        parts.append(f'(and {dia} {box})' if m == 0 else f'(and ([{r}] {no_grade_below(m)}) {dia} {box})')
    return '(or ' + ' '.join(parts) + ')'


def knows(r, phi):
    return f'([{r}] {phi})'


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


def sensing(name, where, open_pre, shut_pre):
    return f"""  (:event e-{name}-open
    :parameters (?r - agent)
    :precondition (and {where} {open_pre})
  )

  (:event e-{name}-shut
    :parameters (?r - agent)
    :precondition (and {where} {shut_pre})
  )

  (:action {name}
    :parameters (?r - agent)
    :action-type (semi-private-sensing (e-{name}-open ?r) (e-{name}-shut ?r))
    :observability-conditions (:and (?r Fully) (default Partially))
  )

"""


def announce(name, pre):
    return f"""  (:event e-{name}
    :parameters (?r - agent)
    :precondition {pre}
  )

  (:event e-{name}-else
    :parameters (?r - agent)
    :precondition (not {pre})
  )

  (:action {name}
    :parameters (?r - agent)
    :action-type (public-announcement (e-{name} ?r) (e-{name}-else ?r))
    :observability-conditions (default Fully)
  )

"""


def move(a, b):
    return f'(:and (at-{b} ?r) (not (at-{a} ?r)))'


def domain(kind):
    out = [HEADER[kind]]
    errors = ['(short ?r - agent)'] if kind == 'sensors' else \
        [f'(err-{d}-{k})' for d in R.DOORS for k in range(1, R.LOOKS + 1)]
    preds = ['(open-' + d + ')' for d in R.DOORS] + errors + [f'(rank{k})' for k in range(G)] + \
        [f'(at-{p} ?r - agent)' for p in PLACES] + \
        ['(scout ?r - agent)', '(mover ?r - agent)', '(delivered ?r - agent)']
    out.append('  (:predicates\n    ' + '\n    '.join(preds) + '\n  )\n\n  (:event nil)\n\n')

    out.append('  ;;--------------------THE HALL------------------\n'
               '  ;;\n  ;; Out of a room to the hall, and a hop along it. Every robot sees every drive.\n\n')
    eff = ' '.join(f'(when (at-{h} ?r) (:and (at-{P.HALL[P.ROOM_AT[h]]} ?r) (not (at-{h} ?r))))' for h in ROOMS)
    out.append(ontic('set-out', '(or ' + ' '.join(f'(at-{h} ?r)' for h in ROOMS) + ')', f'(:and {eff})'))
    c = P.HALL
    for i in range(len(c)):
        for j in (i - 1, i + 1):
            if 0 <= j < len(c):
                out.append(ontic(f'hop-{c[i]}-{c[j]}', f'(at-{c[i]} ?r)', move(c[i], c[j])))

    out.append('  ;;--------------------READINGS FROM A MOUTH------------------\n'
               '  ;;\n  ;; A scout looks down a passage from its mouth. It reads the door shut only where\n'
               '  ;; the door is shut and the reading does not err. The scout sees what it read; the\n'
               '  ;; others see that it looked.\n\n')
    for d in R.DOORS:
        where = f'(scout ?r) (at-v{d} ?r)'
        if kind == 'sensors':
            out.append(sensing(f'look-{d}', where, f'(or (open-{d}) (short ?r))',
                               f'(not (open-{d})) (not (short ?r))'))
        else:
            for k in range(1, R.LOOKS + 1):
                out.append(sensing(f'look{k}-{d}', where, f'(or (open-{d}) (err-{d}-{k}))',
                                   f'(not (open-{d})) (not (err-{d}-{k}))'))

    out.append('  ;;--------------------READINGS AT THE MARK------------------\n'
               '  ;;\n  ;; A scout drives in to the mark before the door, where every laser reads right,\n'
               '  ;; and out again. Any robot at a mark reads the door there.\n\n')
    for d in R.DOORS:
        out.append(ontic(f'advance-{d}', f'(and (scout ?r) (at-v{d} ?r))', move('v' + d, 'm' + d)))
        out.append(sensing(f'check-{d}', f'(at-m{d} ?r)', f'(open-{d})', f'(not (open-{d}))'))
        out.append(ontic(f'retire-{d}', f'(and (scout ?r) (at-m{d} ?r))', move('m' + d, 'v' + d)))

    out.append('  ;;--------------------WHAT IS SAID------------------\n'
               '  ;;\n  ;; A robot says what it read, and every robot hears it: a door read open from a\n'
               '  ;; mouth (which is the door open, or the reading in error), a door read shut from\n'
               '  ;; a mouth (the door shut and the reading sound, which is more than the door shut:\n'
               '  ;; said of a door already known shut, it is news of the laser), a door known shut,\n'
               '  ;; a door known open.\n\n')
    for d in R.DOORS:
        if kind == 'sensors':
            phi = f'(or (open-{d}) (short ?r))'
            out.append(announce(f'say-open-{d}', f'(and (scout ?r) {phi} {knows("?r", phi)})'))
            psi = f'(and (not (open-{d})) (not (short ?r)))'
            out.append(announce(f'say-read-shut-{d}', f'(and (scout ?r) {psi} {knows("?r", psi)})'))
        else:
            for k in range(1, R.LOOKS + 1):
                phi = f'(or (open-{d}) (err-{d}-{k}))'
                out.append(announce(f'say-open{k}-{d}', f'(and (scout ?r) {phi} {knows("?r", phi)})'))
                psi = f'(and (not (open-{d})) (not (err-{d}-{k})))'
                out.append(announce(f'say-read-shut{k}-{d}', f'(and (scout ?r) {psi} {knows("?r", psi)})'))
        out.append(announce(f'say-shut-{d}', f'(and (not (open-{d})) {knows("?r", f"(not (open-{d}))")})'))
        out.append(announce(f'say-sure-{d}', f'(and (open-{d}) {knows("?r", f"(open-{d})")})'))

    out.append('  ;;--------------------THE BED-MOVERS------------------\n'
               '  ;;\n  ;; A bed-mover drives into a passage only when it is sure enough that the door is\n'
               '  ;; open: believing it, believing it firmly (to degree 2), or knowing it. It cannot\n'
               '  ;; turn round inside: it crosses a door it knows open, or backs out.\n\n')
    for d in R.DOORS:
        phi = f'(open-{d})'
        for word, guard in (('believing', believes('?r', phi, 1)), ('firmly', believes('?r', phi, 2)),
                            ('knowing', knows('?r', phi))):
            out.append(ontic(f'enter-{word}-{d}', f'(and (mover ?r) (at-v{d} ?r) {guard})', move('v' + d, 'm' + d)))
        out.append(ontic(f'cross-{d}', f'(and (mover ?r) (at-m{d} ?r) (open-{d}) {knows("?r", phi)})',
                         move('m' + d, 'n' + d)))
        out.append(ontic(f'reverse-{d}', f'(and (mover ?r) (at-m{d} ?r))', move('m' + d, 'v' + d)))
        out.append(ontic(f'deliver-{d}', f'(and (mover ?r) (at-n{d} ?r) (not (delivered ?r)))',
                         f'(:and (delivered ?r) (not (at-n{d} ?r)))'))
    out.append(')\n')
    return ''.join(out)


_COMMON = """;;
;; A hospital floor at night. Four fire doors, A to D from west to east, close
;; four passages through a band of linen rooms between the hall and the corridor.
;; On his round the night porter shut two of them. A scout reads a door from its
;; passage's mouth, more than five metres off, or from the mark before it. A
;; short-sighted laser gets no return from a leaf that far away, and reads every
;; door open from a mouth; at the mark every laser reads right. A bed-mover
;; cannot turn round in a passage, and drives in only when it is sure enough
;; that the door is open.
;;
;; Every world carries its Spohn rank as a grade atom (rank0 .. rank5). Belief
;; is a formula over knowledge and the grades: a reading removes worlds, and the
;; least grade among those left says what is believed, and how firmly.
"""

HEADER = {
    'sensors': """;; A FAULTY SENSOR IN THE FLEET: an error belongs to a sensor. Written by
;; tools/model.py in faulty_sensor_demo; do not edit by hand.
""" + _COMMON + """;;
;; Here a world is the porter's round and the set of short-sighted scouts, and
;; its rank the sum of their grades. A scout that has read one door shut is
;; known sound, and what it reads open is then known open.

(define (domain faulty-sensor)

  (:requirements
    :equality :partial-observability :ontic-actions :negative-preconditions
    :modal-preconditions :conditional-effects :universal-preconditions :disjunctive-preconditions
  )

  (:action-type-libraries intermediate)

""",
    'readings': """;; A FAULTY SENSOR IN THE FLEET: an error belongs to a reading. Written by
;; tools/model.py in faulty_sensor_demo; do not edit by hand.
""" + _COMMON + """;;
;; Here a world is the porter's round and, for each shut door, which of the first
;; and the second look at it from its mouth errs; its rank is the number of looks
;; that err. No reading says anything of another, so two looks at one door by one
;; scout make a firm belief.

(define (domain faulty-readings)

  (:requirements
    :equality :partial-observability :ontic-actions :negative-preconditions
    :modal-preconditions :conditional-effects :universal-preconditions :disjunctive-preconditions
  )

  (:action-type-libraries intermediate)

""",
}


def problem(act, kind):
    m = R.Model(act, kind)
    names = [f'w{i}' for i in range(m.n)]
    fleet = R.fleet(act)
    agents = [r.name for r in fleet]
    lines = [f';; A FAULTY SENSOR IN THE FLEET, act {act}, an error belonging to a '
             f'{"sensor" if kind == "sensors" else "reading"}. Written by tools/model.py.',
             ';; ' + '; '.join(f'{r.name}: {r.what}, starts in {r.room}' + (f' for ward {r.ward}' if r.ward else
                                                                              f', grade {r.grade}') for r in fleet),
             f';; {m.n} worlds: the two doors shut, then what errs; then the rank.']
    for i in range(m.n):
        lines.append(f';;   {names[i]}: {m.name(i)}  {m.rank[i]}')
    pairs = ' '.join(f'({u} {v})' for u in names for v in names)
    rel = [f'      {a} (:and {pairs})' for a in agents]
    common = []
    for r in fleet:
        common += [f'({r.role} {r.name})', f'(at-{r.room} {r.name})']
    labels = []
    for i in range(m.n):
        facts = [f'(open-{d})' for d in R.DOORS if d not in m.worlds[i][0]] + m.atoms(i) + [f'(rank{m.rank[i]})']
        labels.append(f"      {names[i]} (:and {' '.join(facts + common)})")
    goal = ' '.join(f'(delivered {r.name})' for r in fleet if r.role == 'mover')
    return '\n'.join(lines) + f"""

(define (problem act{act}-{kind})
  (:domain {DOMAIN[kind]})
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
    for kind in ('sensors', 'readings'):
        open(os.path.join(a.out, f'{DOMAIN[kind]}.epddl'), 'w').write(domain(kind))
        for act in sorted(R.FLEETS):
            open(os.path.join(a.out, 'instances', f'act{act}-{kind}.epddl'), 'w').write(problem(act, kind))


if __name__ == '__main__':
    main()
