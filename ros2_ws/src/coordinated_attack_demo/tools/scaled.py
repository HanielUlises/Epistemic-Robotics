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
The coordinated attack for n robots and m message levels.

    scaled.py --robots 4 --messages 4 --out DIR

writes into DIR the domain `coordinated-attack-m4.epddl`, the two floors
`beacon-n4-m4.epddl` and `radio-n4-m4.epddl`, and, for the executor, the
action mapping and the classical model the two floors are run with.

The domain is coordinated-attack.epddl with two things made general.

  robots     the load needs every robot under it, so lift requires
             C_All job(s), common knowledge over all n. The agents are
             south, north, south2, north2, ...; south reads the order.
  messages   level l, for l = 1..m, from i to j, says K_i E^(l-1) job(s):
             the sender knows that everyone knows, l-1 times over, where
             the load is. Level 1 is tell, level 2 ack, level 3 ack2, and so
             on. Each (level, sender, receiver) goes out at most once, which
             the sender's log enforces, and so the space stays finite.

With two robots the content is the published one: K_i E^(l-1) job(s) and
K_i K_j K_i ... job(s), l operators deep, are equivalent in S5 when there
are two agents, since E is then K_i and K_j together and the conjuncts
that repeat an agent collapse by introspection. What differs is the log:
the published domain lets each level go out once in all, and this one once
per sender and receiver, so at n = 2 it admits the four messages and their
mirror images. The beacon floor's result is the same and the radio floor's
is still no policy; the radio space is larger.

With more robots, what a receiver learns is more than the content: a robot
that knows where the load is can only have learnt it from one that knew
first, and the model records who could have told whom. How many messages
each level costs is a property of this family of messages, and is what
`tools/ladder.py` measures.
"""

import argparse
import json
import os


def agents(n):
    """south, north, south2, north2, ...: alternately on the storage and the
    dispatch floor, numbered from the second on each."""
    names = []
    for k in range(n):
        side = 'south' if k % 2 == 0 else 'north'
        names.append(side if k < 2 else f'{side}{k // 2 + 1}')
    return names


STANDS = ('s1', 's2')
READER = 'south'


def kind(level):
    """tell, ack, ack2, ack3, ... for level 1, 2, 3, 4, ..."""
    if level == 1:
        return 'tell'
    if level == 2:
        return 'ack'
    return f'ack{level - 1}'


def kinds(m):
    return [kind(level) for level in range(1, m + 1)]


def everyone(k, atom):
    """E^k atom, in EPDDL."""
    for _ in range(k):
        atom = f'([All] {atom})'
    return atom


def content(level, var='?i', atom='(job ?s)'):
    """K_i E^(level-1) job(s), in EPDDL."""
    return f'([{var}] {everyone(level - 1, atom)})'


# ─── The domain ─────────────────────────────────────────────────────────────

DOMAIN_HEAD = """\
(define (domain coordinated-attack)

  ;; The coordinated attack for any number of robots, with {m} message levels.
  ;; Written by tools/scaled.py; coordinated-attack.epddl is the published
  ;; two-robot domain, and the comments there say why each part is as it is.
  ;;
  ;; Every robot takes a share of the load, so lift requires common knowledge
  ;; over all of them. A radio message at level l says that its sender knows
  ;; that everyone knows, l-1 times over, which stand the order names; each
  ;; level goes from each sender to each receiver at most once.

  (:requirements
    :typing :equality :partial-observability :list-comprehensions :lists
    :ontic-actions :group-modalities :static-common-knowledge
    :negative-preconditions :modal-preconditions :knowing-whether
  )

  (:action-type-libraries intermediate lossy)

  (:types stand)

  (:predicates
    (job ?s - stand)
    (reads-order ?i - agent)
    (beacon)
    (at-view ?i - agent)
    (lifted)
{logs}
  )

  (:event nil)

  (:event e-here
    :parameters (?i - agent ?s - stand)
    :precondition (and (reads-order ?i) (not (at-view ?i)) (job ?s) (<Kw. ?i> (job ?s)))
  )

  (:event e-elsewhere
    :parameters (?i - agent ?s - stand)
    :precondition (and (reads-order ?i) (not (at-view ?i)) (not (job ?s)) (<Kw. ?i> (job ?s)))
  )

  (:action read-order
    :parameters (?i - agent ?s - stand)
    :action-type (semi-private-sensing (e-here ?i ?s) (e-elsewhere ?i ?s))
    :observability-conditions (:and (?i Fully) (default Partially))
  )
"""

MESSAGE = """
  ; Level {level}: K_i{es} job(s).

  (:event e-{kind}
    :parameters (?i ?j - agent ?s - stand)
    :precondition (and (not (sent-{kind} ?i ?j)) {content})
    :effects (sent-{kind} ?i ?j)
  )

  (:event e-{kind}-lost
    :parameters (?i ?j - agent ?s - stand)
    :precondition (and (not (sent-{kind} ?i ?j)) {content})
    :effects (sent-{kind} ?i ?j)
  )

  (:action {kind}
    :parameters (?i ?j - agent ?s - stand | (/= ?i ?j))
    :action-type (lossy-message (e-{kind} ?i ?j ?s) (e-{kind}-lost ?i ?j ?s) (nil))
    :observability-conditions (:and (?i Sender) (?j Receiver) (default Bystander))
  )
"""

DOMAIN_TAIL = """
  (:event e-go-view
    :parameters (?i - agent)
    :precondition (and (beacon) (not (at-view ?i)))
    :effects (at-view ?i)
  )

  (:action go-view
    :parameters (?i - agent)
    :action-type (public-ontic (e-go-view ?i))
    :observability-conditions (default Fully)
  )

  (:event e-signal
    :parameters (?i - agent ?s - stand)
    :precondition (and (at-view ?i) ([?i] (job ?s)))
  )

  (:action signal
    :parameters (?i - agent ?s - stand)
    :action-type (private-announcement (e-signal ?i ?s) (nil))
    :observability-conditions
      (:forall (?j - agent)
        (?j (if (at-view ?j) Fully else Oblivious)))
  )

  (:event e-lift
    :parameters (?s - stand)
    :precondition (and (job ?s) ([C. All] (job ?s)))
    :effects (lifted)
  )

  (:action lift
    :parameters (?s - stand)
    :action-type (public-ontic (e-lift ?s))
    :observability-conditions (default Fully)
  )
)
"""


def domain_text(m):
    logs = '\n'.join(f'    (sent-{k} ?i ?j - agent)' for k in kinds(m))
    out = DOMAIN_HEAD.format(m=m, logs=logs)
    for level in range(1, m + 1):
        out += MESSAGE.format(level=level, kind=kind(level), content=content(level),
                              es=' E' * (level - 1))
    return out + DOMAIN_TAIL


# ─── The two floors ─────────────────────────────────────────────────────────

PROBLEM = """\
(define (problem coordinated-attack-{floor}-n{n}-m{m})

  (:domain coordinated-attack)

  ;; {robots} on the pass-through floor, {where}. Written by tools/scaled.py.

  (:requirements
    :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :knowing-whether
    :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:objects s1 s2 - stand)

  (:init
    (:and
      ([C. All] ({beacon}))
      ([C. All] (not (lifted)))
{logs}
      ([C. All] (reads-order {reader}))
{others}
      (:forall (?i - agent) ([C. All] (not (at-view ?i))))
      ([C. All]
        (or
          (and (job s1) (not (job s2)))
          (and (not (job s1)) (job s2))))
      (:forall (?i - agent)
        (:forall (?s - stand)
          ([C. All] (<Kw. ?i> (job ?s)))))
    )
  )

  (:goal {goal})
)
"""


def problem_text(n, m, floor, depth=None):
    """The floor's problem. With @p depth, the goal is E^depth job(s) for the
    stand the order names, in place of lifted: what tools/ladder.py asks the
    planner, whose iterative deepening then finds the fewest messages."""
    names = agents(n)
    logs = '\n'.join(f'      (:forall (?i ?j - agent) ([C. All] (not (sent-{k} ?i ?j))))'
                     for k in kinds(m))
    others = '\n'.join(f'      ([C. All] (not (reads-order {a})))' for a in names if a != READER)
    if depth is None:
        goal = '(lifted)'
    else:
        goal = '(or ' + ' '.join(everyone(depth, f'(job {s})') for s in STANDS) + ')'
    return PROBLEM.format(
        floor=floor if depth is None else f'{floor}-e{depth}', n=n, m=m,
        agents=' '.join(names), reader=READER, logs=logs, others=others, goal=goal,
        robots=f'{n} robots', beacon='beacon' if floor == 'beacon' else 'not (beacon)',
        where='with the beacon in t2' if floor == 'beacon' else 'with no beacon')


def domain_name(m):
    return f'coordinated-attack-m{m}'


def problem_name(n, m, floor, depth=None):
    return f'{floor}-n{n}-m{m}' + ('' if depth is None else f'-e{depth}')


# ─── What the executor needs ────────────────────────────────────────────────

DURATION = {'read-order': 60.0, 'radio': 4.0, 'go-view': 60.0, 'signal': 6.0, 'lift': 90.0}


def mapping(n, m):
    """Ground action name -> classical action, as make_mapping.py writes it
    for the published floors: lift is one joint action for every robot."""
    names = agents(n)
    table = {}
    for i in names:
        table[f'go-view_{i}'] = {'action': f'(go_view {i})', 'duration': DURATION['go-view']}
        for s in STANDS:
            table[f'read-order_{i}_{s}'] = {'action': f'(read_order {i} {s})',
                                            'duration': DURATION['read-order']}
            table[f'signal_{i}_{s}'] = {'action': f'(signal {i} {s})',
                                        'duration': DURATION['signal']}
            for j in names:
                if i != j:
                    for k in kinds(m):
                        table[f'{k}_{i}_{j}_{s}'] = {'action': f'(radio_{k} {i} {j} {s})',
                                                     'duration': DURATION['radio']}
    for s in STANDS:
        table[f'lift_{s}'] = {'action': f'(lift {" ".join(names)} {s})',
                              'duration': DURATION['lift']}
    return dict(sorted(table.items()))


PDDL = """\
;; The classical half of the coordinated attack with {n} robots and {m} message
;; levels, written by tools/scaled.py. As in coordinated-attack.pddl, nothing
;; here knows which stand the order names or that a lift needs common
;; knowledge of it; the executor checks that against the model.

(define (domain coordinated-attack)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  stand
)

(:predicates
  (ready ?r - robot)
  (read ?r - robot ?s - stand)
  (said ?from ?to - robot ?s - stand)
  (at-view ?r - robot)
  (signalled ?s - stand)
  (lifted)
)

(:durative-action read_order
  :parameters (?r - robot ?s - stand)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (read ?r ?s)))
)
{radio}
(:durative-action go_view
  :parameters (?r - robot)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (at-view ?r)))
)

(:durative-action signal
  :parameters (?r - robot ?s - stand)
  :duration (= ?duration 6)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (signalled ?s)))
)

;; Joint: one action for every robot, since no share of the load may be
;; raised alone.
(:durative-action lift
  :parameters ({robots} - robot ?s - stand)
  :duration (= ?duration 90)
  :condition (and {ready})
  :effect (and (at end (lifted)))
)
)
"""

RADIO = """
(:durative-action radio_{kind}
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (said ?from ?to ?s)))
)
"""


def pddl_text(n, m):
    robots = [f'?r{k + 1}' for k in range(n)]
    return PDDL.format(
        n=n, m=m, radio=''.join(RADIO.format(kind=k) for k in kinds(m)),
        robots=' '.join(robots), ready=' '.join(f'(at start (ready {r}))' for r in robots))


def write(out, n, m, executor=True):
    """Writes the domain and both floors for (n, m), and with @p executor the
    mapping and the classical model. Returns the paths by role."""
    os.makedirs(out, exist_ok=True)
    paths = {'domain': os.path.join(out, domain_name(m) + '.epddl')}
    with open(paths['domain'], 'w') as fh:
        fh.write(domain_text(m))
    for floor in ('beacon', 'radio'):
        paths[floor] = os.path.join(out, problem_name(n, m, floor) + '.epddl')
        with open(paths[floor], 'w') as fh:
            fh.write(problem_text(n, m, floor))
    if executor:
        paths['mapping'] = os.path.join(out, f'mapping-n{n}-m{m}.json')
        with open(paths['mapping'], 'w') as fh:
            json.dump(mapping(n, m), fh, indent=2)
            fh.write('\n')
        paths['pddl'] = os.path.join(out, f'coordinated-attack-n{n}-m{m}.pddl')
        with open(paths['pddl'], 'w') as fh:
            fh.write(pddl_text(n, m))
    return paths


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', type=int, default=2)
    ap.add_argument('--messages', type=int, default=4)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if args.robots < 2 or args.messages < 1:
        raise SystemExit('needs at least two robots and one message level')
    for role, path in write(args.out, args.robots, args.messages).items():
        print(f'{role:8s} {path}')


if __name__ == '__main__':
    main()
