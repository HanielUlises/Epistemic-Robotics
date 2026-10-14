#!/usr/bin/env python3
"""The watcher that moves, for a watcher with K changes of post.

    make_domain.py --moves K --out DIR [--no-gates]

Writes DIR/watcher-mobile.epddl and DIR/instances/problem_K.epddl.

The hall of watcher.epddl. The carrier stands at one of five places -- its
start, the two windows, the two gate approaches -- and drives between them.
Every drive is an action with one event per thing the watcher may do while it
lasts: stay, or go to the other post if it has a change of post left; only
the watcher tells them apart. The carrier can look down the hall it stands
in, from a window or from a gate approach. From a gate approach of the bay its
order names it can go through the gate. Observability follows the sightlines:
nobody sees the boards or the back route; the watcher sees a window, a gate
and the drives of its own hall only from its own side, and knows where it went.

With --no-gates the gate approaches and the gates are left out: the carrier
tells the ally only by the lamp at a window. Its visible choices then depend on
where the watcher is and not on the order, which is what keeps the order from a
watcher that knows the policy (tools/opacity_sightlines.py).

The watcher's changes of post left are counted by atoms m1 .. mK, mi meaning
at least i left; a change of post consumes the highest.
"""
import argparse
import os

PLACES_ALL = ['win-w', 'win-e', 'ga-w', 'ga-e']
PLACES = PLACES_ALL
SIDE = {'win-w': 'w', 'win-e': 'e', 'ga-w': 'w', 'ga-e': 'e'}


def watch_cond(side):
    return '(west ?w)' if side == 'w' else '(not (west ?w))'


def where(s):
    return f'(or (at-win-{s} ?i) (at-ga-{s} ?i))' if 'ga-w' in PLACES else f'(at-win-{s} ?i)'


def domain(k):
    out = [f''';; THE WATCHER THAT MOVES, with {k} change{"s" if k != 1 else ""} of post. Written by make_domain.py.
;; See the docstring there for the hall, the actions and what each agent observes.

(define (domain watcher-mobile)

  (:requirements
    :equality :partial-observability :list-comprehensions :lists
    :ontic-actions :negative-preconditions :modal-preconditions :knowing-whether
    :negative-obs-conditions
  )

  (:action-type-libraries intermediate sightlines-mobile)

  (:predicates
    (dest-x)
    (west ?w - agent)
''' + ''.join(f'    (m{i} ?w - agent)\n' for i in range(1, k + 1)) + '''    (carrier ?i - agent)
    (ally ?a - agent)
    (watcher ?w - agent)
    (at-start ?i - agent)
''' + ''.join(f'    (at-{p} ?i - agent)\n' for p in PLACES) + '''    (in-x ?i - agent)
    (in-y ?i - agent)
  )

  (:event nil)

  ;;--------------------THE BOARDS: THE OTHERS KNOW THEY WERE READ, NOT WHAT------------------

  (:event e-order-x
    :parameters (?i - agent)
    :precondition (and (carrier ?i) (at-start ?i) (dest-x)))
  (:event e-order-y
    :parameters (?i - agent)
    :precondition (and (carrier ?i) (at-start ?i) (not (dest-x))))
  (:action read-order
    :parameters (?i - agent)
    :action-type (semi-private-sensing (e-order-x ?i) (e-order-y ?i))
    :observability-conditions (:and (?i Fully) (default Partially)))

  (:event e-peek-west
    :parameters (?i ?w - agent)
    :precondition (and (carrier ?i) (watcher ?w) (at-start ?i) (west ?w)))
  (:event e-peek-east
    :parameters (?i ?w - agent)
    :precondition (and (carrier ?i) (watcher ?w) (at-start ?i) (not (west ?w))))
  (:action peek
    :parameters (?i ?w - agent | (/= ?i ?w))
    :action-type (semi-private-sensing (e-peek-west ?i ?w) (e-peek-east ?i ?w))
    :observability-conditions (:and (?i Fully) (default Partially)))
''']
    # drives: to each place, from the start or any other place
    for p in PLACES:
        frm = ' '.join(f'(at-{q} ?i)' for q in ['start'] + PLACES if q != p)
        clear = ' '.join(f'(not (at-{q} ?i))' for q in ['start'] + PLACES if q != p)
        events = [('stay', '', '')]
        for i in range(k, 0, -1):
            higher = f' (not (m{i + 1} ?w))' if i < k else ''
            events.append((f'go-w-{i}', f' (m{i} ?w){higher} (not (west ?w))', f' (west ?w) (not (m{i} ?w))'))
            events.append((f'go-e-{i}', f' (m{i} ?w){higher} (west ?w)', f' (not (west ?w)) (not (m{i} ?w))'))
        out.append(f'\n  ;;--------------------DRIVE TO {p.upper()}------------------\n')
        for name, pre, eff in events:
            out.append(f'''  (:event e-to-{p}-{name}
    :parameters (?i ?w - agent)
    :precondition (and (carrier ?i) (watcher ?w) (or {frm}){pre})
    :effects (:and (at-{p} ?i) {clear}{eff}))
''')
        names = ' '.join(f'(e-to-{p}-{n} ?i ?w)' for n, _, _ in events)
        atype = f'drive-while-watched-{len(events)}'
        out.append(f'''  (:action to-{p}
    :parameters (?i ?w - agent | (/= ?i ?w))
    :action-type ({atype} {names})
    :observability-conditions (:and (?w Fully) (default Partially)))
''')
    # looks
    for s in ('w', 'e'):
        here = watch_cond(s)
        gone = '(not (west ?w))' if s == 'w' else '(west ?w)'
        out.append(f'''
  ;;--------------------LOOKING DOWN THE {"WEST" if s == "w" else "EAST"} HALL------------------

  (:event e-look-{s}-here
    :parameters (?i ?w - agent)
    :precondition (and (carrier ?i) (watcher ?w) {where(s)} {here}))
  (:event e-look-{s}-clear
    :parameters (?i ?w - agent)
    :precondition (and (carrier ?i) (watcher ?w) {where(s)} {gone}))
  (:action look-{s}
    :parameters (?i ?w - agent | (/= ?i ?w))
    :action-type (private-sensing (e-look-{s}-here ?i ?w) (e-look-{s}-clear ?i ?w) (nil))
    :observability-conditions (:and (?i Fully) (default Oblivious)))
''')
    # the lamp
    for s in ('w', 'e'):
        out.append(f'''
  (:event e-say-x-{s}
    :parameters (?i ?a ?w - agent)
    :precondition (and (carrier ?i) (ally ?a) (watcher ?w) (at-win-{s} ?i) (dest-x) ([Kw. ?i] (dest-x))))
  (:event e-say-y-{s}
    :parameters (?i ?a ?w - agent)
    :precondition (and (carrier ?i) (ally ?a) (watcher ?w) (at-win-{s} ?i) (not (dest-x)) ([Kw. ?i] (dest-x))))
  (:action say-{s}
    :parameters (?i ?a ?w - agent | (and (/= ?i ?a) (/= ?i ?w) (/= ?a ?w)))
    :action-type (quasi-private-sensing (e-say-x-{s} ?i ?a ?w) (e-say-y-{s} ?i ?a ?w) (nil))
    :observability-conditions
      (:and (?i Fully) (?a Fully) (?w (if {watch_cond(s)} Fully else Oblivious)) (default Oblivious)))
''')
    # the gates: bay X's is west, bay Y's east
    for s, b, fact in [] if 'ga-w' not in PLACES else (('w', 'x', '(dest-x)'), ('e', 'y', '(not (dest-x))')):
        out.append(f'''
  (:event e-gate-{b}
    :parameters (?i ?a ?w - agent)
    :precondition (and (carrier ?i) (ally ?a) (watcher ?w) (at-ga-{s} ?i) {fact} ([Kw. ?i] (dest-x)))
    :effects (:and (in-{b} ?i) (not (at-ga-{s} ?i))))
  (:action gate-{b}
    :parameters (?i ?a ?w - agent | (and (/= ?i ?a) (/= ?i ?w) (/= ?a ?w)))
    :action-type (private-ontic (e-gate-{b} ?i ?a ?w) (nil))
    :observability-conditions
      (:and (?i Fully) (?a Fully) (?w (if {watch_cond(s)} Fully else Oblivious)) (default Oblivious)))
''')
    out.append('''
  ;;--------------------THE BACK ROUTE, SEEN BY NOBODY------------------

  (:event e-back-x
    :parameters (?i - agent)
    :precondition (and (carrier ?i) (or (at-win-w ?i) (at-win-e ?i)) (dest-x) ([Kw. ?i] (dest-x)))
    :effects (:and (in-x ?i) (not (at-win-w ?i)) (not (at-win-e ?i))))
  (:event e-back-y
    :parameters (?i - agent)
    :precondition (and (carrier ?i) (or (at-win-w ?i) (at-win-e ?i)) (not (dest-x)) ([Kw. ?i] (dest-x)))
    :effects (:and (in-y ?i) (not (at-win-w ?i)) (not (at-win-e ?i))))
  (:action back
    :parameters (?i - agent)
    :action-type (quasi-private-ontic (e-back-x ?i) (e-back-y ?i) (nil))
    :observability-conditions (:and (?i Fully) (default Oblivious)))
)
''')
    return ''.join(out)


def library(k):
    n = 1 + 2 * k
    ev = ' '.join(f'?e{i}' for i in range(n))
    cond = '\n                 '.join(f'?e{i} (:non-trivial-postconditions)' for i in range(n))
    return f'''(define (action-type-library sightlines-mobile)

  ;; The types of watcher.epddl's sightlines library, and a drive with {n}
  ;; designated events, one per thing the watcher may do while it lasts: the
  ;; mover cannot tell them apart (Partially), the watcher can. Written by
  ;; make_domain.py; plank substitutes relation variables as text, so they are
  ;; ?x and ?y.

  (:requirements
    :lists :list-comprehensions :equality
    :partial-observability :multi-pointed-models :events-conditions
    :negative-list-formulas
  )

  (:action-type quasi-private-ontic
    :events     (?pos ?neg ?nil)
    :observability-types (Fully Partially Oblivious)
    :relations  (Fully     (:forall (?x - event) (?x ?x))
                 Partially (:forall (?x ?y - event |
                    (imply
                        (/= ?x ?y)
                        (and (/= ?x ?nil) (/= ?y ?nil)) ))
                    (?x ?y) )
                 Oblivious (:forall (?x - event) (?x ?nil)) )
    :designated (?pos ?neg)
    :conditions (?pos (:non-trivial-postconditions)
                 ?neg (:non-trivial-postconditions)
                 ?nil (:trivial-event) )
  )

  (:action-type drive-while-watched-{n}
    :events     ({ev})
    :observability-types (Fully Partially)
    :relations  (Fully     (:forall (?x - event) (?x ?x))
                 Partially (:forall (?x ?y - event) (?x ?y)) )
    :designated ({ev})
    :conditions ({cond})
  )
)
'''


def problem(k):
    lines = []
    roles = {'carrier': 'r1', 'ally': 'r2', 'watcher': 'r3'}
    for r, ag in roles.items():
        for x in ('r1', 'r2', 'r3'):
            lines.append(f'      ([C. All] {"(" + r + " " + x + ")" if x == ag else "(not (" + r + " " + x + "))"})')
    lines.append('      ([C. All] (at-start r1))')
    for x in ('r2', 'r3'):
        lines.append(f'      ([C. All] (not (at-start {x})))')
    for p in PLACES + ['in-x', 'in-y']:
        pred = p if p.startswith('in-') else f'at-{p}'
        for x in ('r1', 'r2', 'r3'):
            lines.append(f'      ([C. All] (not ({pred} {x})))')
    for x in ('r1', 'r2'):
        lines.append(f'      ([C. All] (not (west {x})))')
    for i in range(1, k + 1):
        lines.append(f'      ([C. All] (m{i} r3))')
        for x in ('r1', 'r2'):
            lines.append(f'      ([C. All] (not (m{i} {x})))')
    lines += ['      (:forall (?i - agent) ([C. All] (<Kw. ?i> (dest-x))))',
              '      ([C. All] (<Kw. r1> (west r3)))', '      ([C. All] (<Kw. r2> (west r3)))',
              '      ([C. All] ([Kw. r3] (west r3)))']
    return f'''(define (problem watcher-mobile-{k})

  ;; Four worlds, one per order and post. The watcher has {k} change{"s" if k != 1 else ""} of post,
  ;; which is common knowledge; where it is, only it knows.

  (:domain watcher-mobile)

  (:requirements
    :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions :negative-goals
  )

  (:agents r1 r2 r3)

  (:init
    (:and
{chr(10).join(lines)}
    )
  )

  (:goal
    (and
      (or (in-x r1) (in-y r1))
      ([Kw. r2] (dest-x))
      (not ([Kw. r3] (dest-x)))
    )
  )
)
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--moves', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-gates', action='store_true')
    a = ap.parse_args()
    global PLACES
    PLACES = ['win-w', 'win-e'] if a.no_gates else PLACES_ALL
    os.makedirs(os.path.join(a.out, 'instances'), exist_ok=True)
    open(os.path.join(a.out, 'watcher-mobile.epddl'), 'w').write(domain(a.moves))
    open(os.path.join(a.out, 'sightlines-mobile.epddl'), 'w').write(library(a.moves))
    open(os.path.join(a.out, 'instances', f'problem_{a.moves}.epddl'), 'w').write(problem(a.moves))


if __name__ == '__main__':
    main()
