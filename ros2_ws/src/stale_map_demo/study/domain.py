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
EPDDL for one instance of the communication study.

The four stale regimes share one domain, written per instance: one action per
forklift slot, so that a bay can change twice and each change has its own
observers. A change sets the bay's state without requiring the old one, since
a robot that sees a load set down sees a load whatever it believed was there
before, and a robot that saw only the second of two changes would otherwise
see an event its world rules out, and be left believing everything.

The elimination regime is an S5 domain of its own: exactly one bay is open,
robots know whether the bays they see are open, and a report carries what
its sender knows, which may be what it inferred.

Library variables are ?x and ?y, never ?e or ?f, and no observability uses
else-if: see plank-grounding-quirks.
"""

STALE_DOMAIN = """\
(define (domain study-{name})

  (:requirements
    :KD45-frames :typing :equality :partial-observability :list-comprehensions
    :lists :ontic-actions :negative-preconditions :modal-preconditions
  )

  (:action-type-libraries intermediate maps)

  (:types bay)

  (:predicates
    (blocked ?t - bay)
    (hauls ?i - agent)
    (solo ?i - agent)
    (pair ?i ?j - agent)
    (delivered ?i - agent)
    (link ?i ?j - agent)
    (radio)
    (settled)
{slot_predicates}
  )

  (:event nil)

{changes}
  (:event e-says-open
    :parameters (?i ?j - agent ?t - bay)
    :precondition (and (radio) (settled) (link ?i ?j)
                       ([?i] (not (blocked ?t))) ([?i] ([?j] (blocked ?t))))
  )

  (:event e-opened
    :parameters (?t - bay)
    :precondition (blocked ?t)
    :effects (:and (not (blocked ?t)))
  )

  (:action send-open
    :parameters (?i ?j - agent ?t - bay | (/= ?i ?j))
    :action-type (map-report (e-says-open ?i ?j ?t) (e-opened ?t) (nil))
    :observability-conditions (:and (?i Fully) (?j Recipient) (default Oblivious))
  )

  (:event e-says-blocked
    :parameters (?i ?j - agent ?t - bay)
    :precondition (and (radio) (settled) (link ?i ?j)
                       ([?i] (blocked ?t)) ([?i] ([?j] (not (blocked ?t)))))
  )

  (:event e-closed
    :parameters (?t - bay)
    :precondition (not (blocked ?t))
    :effects (:and (blocked ?t))
  )

  (:action send-blocked
    :parameters (?i ?j - agent ?t - bay | (/= ?i ?j))
    :action-type (map-report (e-says-blocked ?i ?j ?t) (e-closed ?t) (nil))
    :observability-conditions (:and (?i Fully) (?j Recipient) (default Oblivious))
  )

  (:event e-cross
    :parameters (?h - agent ?t - bay)
    :precondition (and (solo ?h) (settled) (not (delivered ?h))
                       (not (blocked ?t)) ([?h] (not (blocked ?t))))
    :effects (:and (delivered ?h))
  )

  (:action cross
    :parameters (?h - agent ?t - bay)
    :action-type (private-ontic (e-cross ?h ?t) (nil))
    :observability-conditions (:and (?h Fully) (default Oblivious))
  )

  ;; Two haulers take one load through one bay together. Each must believe
  ;; the bay open and believe the other does, or one of them is not there.
  (:event e-cross-joint
    :parameters (?h ?k - agent ?t - bay)
    :precondition (and (pair ?h ?k) (settled) (not (delivered ?h)) (not (delivered ?k))
                       (not (blocked ?t))
                       ([?h] (not (blocked ?t))) ([?k] (not (blocked ?t)))
                       ([?h] ([?k] (not (blocked ?t)))) ([?k] ([?h] (not (blocked ?t)))))
    :effects (:and (delivered ?h) (delivered ?k))
  )

  (:action cross-joint
    :parameters (?h ?k - agent ?t - bay | (/= ?h ?k))
    :action-type (private-ontic (e-cross-joint ?h ?k ?t) (nil))
    :observability-conditions (:and (?h Fully) (?k Fully) (default Oblivious))
  )
)
"""

CHANGE = """\
  (:event e-change{k}
    :parameters (?t - bay)
    :precondition (and (due{k}) (bay{k} ?t))
    :effects (:and {effect} (not (due{k})) {after})
  )

  (:event e-slot{k}
    :parameters (?t - bay)
    :precondition (and (due{k}) (bay{k} ?t))
    :effects (:and (not (due{k})) {after})
  )

  (:action change{k}
    :parameters (?t - bay)
    :action-type (change (e-change{k} ?t) (e-slot{k} ?t))
    :observability-conditions
      (:forall (?j - agent) (?j (if (saw{k} ?j) Fully else Oblivious)))
  )

"""

STALE_PROBLEM = """\
(define (problem task-{name})

  (:domain study-{name})

  (:requirements
    :KD45-frames :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:objects {bays} - bay)

  (:init
    (:and
{facts}
    )
  )

  (:goal (and {goal}))
)
"""


def ck(text):
    return f'      ([C. All] {text})'


def stale_task(inst, name):
    """The domain and problem text of a stale-regime instance."""
    k_max = len(inst.changes)
    preds = []
    changes = []
    for k, c in enumerate(inst.changes, start=1):
        preds += [f'    (due{k})', f'    (bay{k} ?t - bay)', f'    (saw{k} ?i - agent)']
        effect = f'(blocked ?t)' if c.kind == 'stage' else f'(not (blocked ?t))'
        after = f'(due{k + 1})' if k < k_max else '(settled)'
        changes.append(CHANGE.format(k=k, effect=effect, after=after))
    domain = STALE_DOMAIN.format(name=name, slot_predicates='\n'.join(preds),
                                 changes=''.join(changes))

    facts = [ck('(radio)'), ck('(not (settled))')]
    for t in inst.bays:
        facts.append(ck(f'(blocked {t})' if t in inst.shift_blocked else f'(not (blocked {t}))'))
    for k, c in enumerate(inst.changes, start=1):
        facts.append(ck(f'(due{k})' if k == 1 else f'(not (due{k}))'))
        for t in inst.bays:
            facts.append(ck(f'(bay{k} {t})' if t == c.bay else f'(not (bay{k} {t}))'))
        for a in inst.agents:
            facts.append(ck(f'(saw{k} {a})' if a in c.observers else f'(not (saw{k} {a}))'))
    solo = set(inst.solo_haulers())
    paired = {(h, k) for h, k in inst.pairs} | {(k, h) for h, k in inst.pairs}
    for a in inst.agents:
        facts.append(ck(f'(hauls {a})' if a in inst.haulers else f'(not (hauls {a}))'))
        facts.append(ck(f'(solo {a})' if a in solo else f'(not (solo {a}))'))
        facts.append(ck(f'(not (delivered {a}))'))
        for b in inst.agents:
            facts.append(ck(f'(link {a} {b})' if a != b and inst.linked(a, b)
                            else f'(not (link {a} {b}))'))
            facts.append(ck(f'(pair {a} {b})' if (a, b) in paired else f'(not (pair {a} {b}))'))

    goal = [f'(delivered {h})' for h in inst.haulers]
    for c in sorted(inst.contractors):
        for t in sorted(inst.secret_bays):
            goal.append(f'(not ([{c}] (blocked {t})))')
    problem = STALE_PROBLEM.format(name=name, agents=' '.join(inst.agents),
                                   bays=' '.join(inst.bays), facts='\n'.join(facts),
                                   goal=' '.join(goal))
    return domain, problem


ELIM_DOMAIN = """\
(define (domain study-{name})

  (:requirements
    :KD45-frames :typing :equality :partial-observability :list-comprehensions :lists
    :ontic-actions :negative-preconditions :modal-preconditions :knowing-whether
  )

  (:action-type-libraries intermediate)

  (:types bay)

  (:predicates
    (open ?t - bay)
    (hauls ?i - agent)
    (delivered ?i - agent)
    (link ?i ?j - agent)
  )

  (:event nil)

  ;; A report of what the sender believes about a bay, which may be what it
  ;; inferred. Private, as a map sent is: the others are oblivious to it, so
  ;; that the epistemic fleet learns from messages exactly what the sharing
  ;; protocols learn from theirs.
  (:event e-says-open
    :parameters (?i ?j - agent ?t - bay)
    :precondition (and (link ?i ?j) ([?i] (open ?t)))
  )

  (:action tell-open
    :parameters (?i ?j - agent ?t - bay | (/= ?i ?j))
    :action-type (private-announcement (e-says-open ?i ?j ?t) (nil))
    :observability-conditions (:and (?i Fully) (?j Fully) (default Oblivious))
  )

  (:event e-says-shut
    :parameters (?i ?j - agent ?t - bay)
    :precondition (and (link ?i ?j) ([?i] (not (open ?t))))
  )

  (:action tell-shut
    :parameters (?i ?j - agent ?t - bay | (/= ?i ?j))
    :action-type (private-announcement (e-says-shut ?i ?j ?t) (nil))
    :observability-conditions (:and (?i Fully) (?j Fully) (default Oblivious))
  )

  ;; Private, for the same reason: a hauler that watched another cross would
  ;; learn the bay is open, and the protocols' haulers do not watch.
  (:event e-cross
    :parameters (?h - agent ?t - bay)
    :precondition (and (hauls ?h) (not (delivered ?h)) (open ?t) ([?h] (open ?t)))
    :effects (:and (delivered ?h))
  )

  (:action cross
    :parameters (?h - agent ?t - bay)
    :action-type (private-ontic (e-cross ?h ?t) (nil))
    :observability-conditions (:and (?h Fully) (default Oblivious))
  )
)
"""

ELIM_PROBLEM = """\
(define (problem task-{name})

  (:domain study-{name})

  (:requirements
    :KD45-frames :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:objects {bays} - bay)

  (:init
    (:and
{facts}
    )
  )

  (:goal (and {goal}))
)
"""


def elimination_task(inst, name):
    facts = []
    one = ' '.join(
        '(and ' + ' '.join(f'(open {u})' if u == t else f'(not (open {u}))' for u in inst.bays) + ')'
        for t in inst.bays)
    facts.append(ck(f'(or {one})'))
    for a in inst.agents:
        facts.append(ck(f'(hauls {a})' if a in inst.haulers else f'(not (hauls {a}))'))
        facts.append(ck(f'(not (delivered {a}))'))
        for b in inst.agents:
            facts.append(ck(f'(link {a} {b})' if a != b and inst.linked(a, b)
                            else f'(not (link {a} {b}))'))
        # What the robot knows whether, stated for every bay: plank's S5
        # initial state takes what is not stated as known. A robot knows
        # whether a bay is open when it sees the bay, when it sees the open
        # bay, or when it sees every other bay.
        seen = inst.sees_at_start[a]
        for t in inst.bays:
            knows = t in seen or inst.open_bay in seen or set(inst.bays) - seen == {t}
            facts.append(ck(f'([Kw. {a}] (open {t}))' if knows else f'(<Kw. {a}> (open {t}))'))
    # The open bay itself: true, and not common knowledge.
    facts.append(f'      (open {inst.open_bay})')
    goal = [f'(delivered {h})' for h in inst.haulers]
    domain = ELIM_DOMAIN.format(name=name)
    problem = ELIM_PROBLEM.format(name=name, agents=' '.join(inst.agents),
                                  bays=' '.join(inst.bays), facts='\n'.join(facts),
                                  goal=' '.join(goal))
    return domain, problem


def task_text(inst, name):
    if inst.regime == 'elimination':
        return elimination_task(inst, name)
    return stale_task(inst, name)
