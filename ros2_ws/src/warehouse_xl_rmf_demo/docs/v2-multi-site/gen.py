#!/usr/bin/env python3
"""Generates the k-site survey problem, for measuring how the search scales."""
import sys
k = int(sys.argv[1])
sites = [f'a{i:02d}' for i in range(1, k + 1)]

def exactly_one():
    out = []
    for s in sites:
        conj = ' '.join(f'(contaminated {t})' if t == s else f'(not (contaminated {t}))'
                        for t in sites)
        out.append(f'(and {conj})')
    return '(or ' + ' '.join(out) + ')' if k > 1 else out[0]

goal = '\n'.join(
    [f'      ([Kw. scout] (contaminated {s}))' for s in sites] +
    [f'      ([Kw. relay] (contaminated {s}))' for s in sites] +
    [f'      (<Kw. observer> (contaminated {s}))' for s in sites])

print(f"""(define (problem site-survey-{k})
  (:domain survey-sites)
  (:requirements
    :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions
  )
  (:agents scout relay observer)
  (:objects {' '.join(sites)} - site)
  (:init
    (:and
      (:forall (?i - agent)
        (:forall (?s - site)
          ([C. All] (not (on-site ?i ?s)))))
      ([C. All] {exactly_one()})
      (:forall (?i - agent)
        (:forall (?s - site)
          ([C. All] (<Kw. ?i> (contaminated ?s)))))
    )
  )
  (:goal
    (and
{goal}
    )
  )
)""")
