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
A leak in the hotel that no robot can place alone.

    hotel_domain.py --out DIR                       # the hotel: 2 floors, 2 columns
    hotel_domain.py --floors 3 --columns 3 --out DIR

writes into DIR the action-type library `earshot.epddl`, the domain
`hotel-distributed.epddl`, one problem per fleet, and, for the executor, the
action mapping and the classical model.

Water is coming through from one guest room. Which one is the product of two
facts held by two different robots of two different fleets:

  cleaner    found water coming down one of the risers on its night round on
             the ground floor. A riser serves one column of rooms, so the
             cleaner knows the column and not the floor. Its fleet's graph has
             no upper floor: it cannot ride a lift.
  concierge  runs the front desk, where the building's flow alarm reports
             which floor's branch is running. It knows the floor and not the
             column. It works on the ground floor.
  porter     carries the valve key. It can ride the lifts and knows neither.
  guest      is in the lobby, has no robot, and must not learn where the leak
             is.

So the room is distributed knowledge of the cleaner and the concierge, and
the only robot that can act on it knows nothing. Fleets of different vendors
share no network: a robot can tell another robot only by being where it is,
and then everyone standing there hears it, the guest included if it is the
lobby. The building's public address reaches everyone.

Every observability type in the library relates an absent agent's view of
every event to every other: an agent that is not there knows where the robots
are, because Open-RMF publishes that to everyone, and not whether anything was
said or done. Every relation is an equivalence, so the models stay S5 and
everything an agent holds is knowledge.

The four fleets differ in the channels they use and the goal they plan for:

  epistemic  every channel; the goal is the whole one: the leak contained,
             every responder knowing that every responder knows it, and the
             guest unable to rule out any room.
  broadcast  every channel; the guest is not modelled as someone to keep in
             the dark, so the goal drops that conjunct.
  filter     a rule keeps any message that names a floor or a column off the
             public address; the all-clear names neither and is allowed. The
             goal is the broadcast fleet's: the rule is meant to do the
             guest's part.
  siloed     no channel between fleets, which is Open-RMF as deployed: fleets
             share the traffic schedule and nothing they observe. It plans for
             the leak contained.
  pa-only    the public address and nothing else, with the whole goal. There
             is no policy, and the planner cannot show it; tools/reach.py does.

What each fleet's policy does to the whole goal is what tools/trace.py
measures and tools/reach.py settles where the planner cannot.
"""

import argparse
import json
import os

AGENTS = ('cleaner', 'concierge', 'porter', 'guest')
RESPONDERS = ('cleaner', 'concierge', 'porter')

# The hotel of rmf_demos has two guest floors, L2 and L3, each with room1 at
# the north end, the master suite at the south-west and room15 at the
# south-east. The demonstration uses the two rooms, which make two columns.
HOTEL_FLOORS = ('L2', 'L3')
HOTEL_COLUMNS = ('room1', 'room15')
EXTRA_COLUMNS = ('master_suite',)

FLEETS = ('epistemic', 'broadcast', 'filter', 'siloed', 'pa-only')
CHANNELS = {
    'epistemic': ('local-talk', 'pa-location', 'pa-all-clear'),
    'broadcast': ('local-talk', 'pa-location', 'pa-all-clear'),
    'filter': ('local-talk', 'pa-all-clear'),
    'siloed': (),
    'pa-only': ('pa-location', 'pa-all-clear'),
}


def floors_of(n):
    return HOTEL_FLOORS[:n] if n <= len(HOTEL_FLOORS) else \
        tuple(f'L{k + 2}' for k in range(n))


def columns_of(n):
    named = HOTEL_COLUMNS + EXTRA_COLUMNS
    return named[:n] if n <= len(named) else \
        named + tuple(f'room{k + 1}x' for k in range(len(named), n))


def room(floor, column):
    """The room's zone name, which is also its waypoint in the hotel map."""
    return f'{floor}_{column}'


# ─── The library ────────────────────────────────────────────────────────────

LIBRARY_HEAD = """\
(define (action-type-library earshot)

  ;; Actions heard or seen only by whoever is in the same place.
  ;;
  ;; Two observability types, and every type below uses them the same way.
  ;; Fully: the agent is there and tells each event from every other. Absent:
  ;; the agent is elsewhere, knows where everyone is, and cannot tell whether
  ;; anything happened or what, so it relates every event to every other,
  ;; ?nil (nothing happened) included. Both relations are equivalences, and
  ;; the product of an S5 model with any of these is S5.
  ;;
  ;; The intermediate library's Oblivious type would serve an absent agent
  ;; badly here. It maps every event onto ?nil, so an oblivious agent believes
  ;; nothing happened even when something did, and an agent that saw the
  ;; porter ride to a room would believe the porter did nothing there. The
  ;; frame would leave S5 for no reason the hotel gives.
  ;;
  ;; The relation variables are ?x and ?y: plank substitutes a library's
  ;; variables textually, and ?e and ?f are action parameters elsewhere.

  (:requirements
    :lists :list-comprehensions :equality
    :partial-observability :multi-pointed-models :events-conditions
  )

  (:action-type local-ontic
    :events (?pos ?nil)
    :observability-types (Fully Absent)
    :relations (Fully  (:forall (?x - event) (?x ?x))
                Absent (:forall (?x ?y - event) (?x ?y)))
    :designated (?pos)
    :conditions (?pos (:non-trivial-postconditions) ?nil (:trivial-event))
  )

  (:action-type local-sensing
    :events (?pos ?neg ?nil)
    :observability-types (Fully Absent)
    :relations (Fully  (:forall (?x - event) (?x ?x))
                Absent (:forall (?x ?y - event) (?x ?y)))
    :designated (?pos ?neg)
    :conditions (?pos (:trivial-postconditions) ?neg (:trivial-postconditions)
                 ?nil (:trivial-event))
  )
"""

LIBRARY_TELL = """
  ;; A robot says which of {n} values holds, to whoever is in the same place.

  (:action-type local-tell-{n}
    :events ({vs} ?nil)
    :observability-types (Fully Absent)
    :relations (Fully  (:forall (?x - event) (?x ?x))
                Absent (:forall (?x ?y - event) (?x ?y)))
    :designated ({vs})
    :conditions ({triv} ?nil (:trivial-event))
  )

  ;; The same over the public address: everyone hears which.

  (:action-type public-tell-{n}
    :events ({vs})
    :observability-types (Fully)
    :relations (Fully (:forall (?x - event) (?x ?x)))
    :designated ({vs})
    :conditions ({triv})
  )
"""


def library_text(arities):
    out = LIBRARY_HEAD
    for n in sorted(set(arities)):
        vs = ' '.join(f'?v{k + 1}' for k in range(n))
        triv = ' '.join(f'?v{k + 1} (:trivial-postconditions)' for k in range(n))
        out += LIBRARY_TELL.format(n=n, vs=vs, triv=triv)
    return out + ')\n'


# ─── The domain ─────────────────────────────────────────────────────────────

DOMAIN_HEAD = """\
(define (domain hotel-distributed)

  ;; A leak no robot can place alone. Written by tools/hotel_domain.py for
  ;; {nf} floors and {nk} columns; the docstring there says what each agent
  ;; knows and why.
  ;;
  ;; The room is not an atom anyone observes. floor-f and col-k are, and
  ;; source(z) holds exactly when the floor and the column of z do, which the
  ;; problem makes common knowledge. Each value has its own atom and its own
  ;; event so that the domain names no object: plank numbers domain constants
  ;; ahead of the agents, and an observability condition that names an agent
  ;; then indexes past the end of the agents.

  (:requirements
    :typing :facts :equality :partial-observability :ontic-actions
    :lists :list-comprehensions :negative-preconditions :modal-preconditions
    :existential-preconditions :knowing-whether :group-modalities
    :multi-pointed-models
  )

  (:action-type-libraries intermediate earshot)

  (:types zone)

  (:predicates
    (at-ag ?i - agent ?z - zone)
    (source ?z - zone)
    {values}
    (safe)
    ;; Rigid, from the hotel's map and the fleets' graphs.
    (:fact link ?i - agent ?a ?b - zone)
    (:fact room ?z - zone)
    (:fact meeting ?z - zone)
    (:fact kit ?i - agent)
    (:fact robot ?i - agent)
    ;; Which channels this fleet has. Rigid, and the only difference between
    ;; the fleets' problems apart from the goal.
    (:fact local-talk)
    (:fact pa-location)
    (:fact pa-all-clear)
  )

  (:event nil)

  ;--------------------MOVING------------------
  ;
  ; Public: Open-RMF publishes every robot's position to every fleet, and the
  ; guest is taken to see it too. A secret that holds only while the guest is
  ; not looking is not kept, so the model gives the guest the robots'
  ; positions, and the mission has to keep the secret anyway.
  ;
  ; A move between floors is one action here and a lift call, a ride and a
  ; traffic negotiation on Open-RMF's side of the bridge.

  (:event e-go
    :parameters (?i - agent ?a ?b - zone)
    :precondition (and (at-ag ?i ?a) (link ?i ?a ?b))
    :effects (:and (not (at-ag ?i ?a)) (at-ag ?i ?b))
  )

  (:action go
    :parameters (?i - agent ?a ?b - zone | (link ?i ?a ?b))
    :action-type (public-ontic (e-go ?i ?a ?b))
    :observability-conditions (default Fully)
  )

  ;--------------------CONTAINING------------------
  ;
  ; Shutting the room's valve. Only a robot with the key, standing in the
  ; room, that knows the leak is there: without the modal precondition a
  ; porter sent to the right room by luck would fix a leak it has no reason
  ; to believe in. Whoever is in the room sees it; everyone else relates it
  ; to nothing having happened.

  (:event e-contain
    :parameters (?i - agent ?z - zone)
    :precondition (and (at-ag ?i ?z) (kit ?i) (room ?z) (not (safe))
                       (source ?z) ([?i] (source ?z)))
    :effects (safe)
  )

  (:action contain
    :parameters (?i - agent ?z - zone | (and (kit ?i) (room ?z)))
    :action-type (local-ontic (e-contain ?i ?z) (nil))
    :observability-conditions
      (:forall (?j - agent) (?j (if (at-ag ?j ?z) Fully else Absent)))
  )

  ;--------------------INSPECTING------------------
  ;
  ; Opening a room and looking. One room per inspection, and only one whose
  ; state the inspector does not already know.

  (:event e-wet
    :parameters (?i - agent ?z - zone)
    :precondition (and (at-ag ?i ?z) (kit ?i) (room ?z)
                       (source ?z) (<Kw. ?i> (source ?z)))
  )

  (:event e-dry
    :parameters (?i - agent ?z - zone)
    :precondition (and (at-ag ?i ?z) (kit ?i) (room ?z)
                       (not (source ?z)) (<Kw. ?i> (source ?z)))
  )

  (:action inspect
    :parameters (?i - agent ?z - zone | (and (kit ?i) (room ?z)))
    :action-type (local-sensing (e-wet ?i ?z) (e-dry ?i ?z) (nil))
    :observability-conditions
      (:forall (?j - agent) (?j (if (at-ag ?j ?z) Fully else Absent)))
  )

  ;--------------------TELLING------------------
  ;
  ; A robot says what it knows to whoever is where it is. Truthful, because
  ; the speaker has to know it and knowledge is factive, and said only when
  ; somebody there does not know it yet: a message nobody learns from is not
  ; an action, it is a delay.
"""

# The event for one value v of a kind: v holds, the speaker knows which value
# holds, and somebody who hears it does not. Given that exactly one value holds
# and that this is common knowledge, "v and the speaker knows which" is "the
# speaker knows v". It is written the long way because the executor checks the
# modal part of every designated event's precondition as a requirement of the
# action before dispatching it: written as "the speaker knows v", the events of
# one telling would require knowing different values, and no state satisfies
# all of them. Written this way every event of a telling requires the same.
TELL_EVENT = """
  (:event e-{kind}-{value}
    :parameters (?i - agent ?z - zone)
    :precondition (and (robot ?i) (at-ag ?i ?z) ({atom}) {speaker_knows}
                       (exists (?j - agent) (and (at-ag ?j ?z) (not {hearer_knows}))))
  )
"""

PAGE_EVENT = """
  (:event e-page-{kind}-{value}
    :parameters (?i - agent)
    :precondition (and (robot ?i) ({atom}) {speaker_knows}
                       (exists (?j - agent) (not {hearer_knows})))
  )
"""


def knows_which(agent, atoms):
    """The agent knows which of @p atoms holds: it knows whether each does."""
    return '(and ' + ' '.join(f'([Kw. {agent}] ({a}))' for a in atoms) + ')'

TELL_ACTIONS = """
  (:action tell-floor
    :parameters (?i - agent ?z - zone | (and (local-talk) (robot ?i) (meeting ?z)))
    :action-type (local-tell-{nf} {floor_events} (nil))
    :observability-conditions {earshot}
  )

  (:action tell-column
    :parameters (?i - agent ?z - zone | (and (local-talk) (robot ?i) (meeting ?z)))
    :action-type (local-tell-{nk} {column_events} (nil))
    :observability-conditions {earshot}
  )

  (:action tell-safe
    :parameters (?i - agent ?z - zone | (and (local-talk) (robot ?i) (meeting ?z)))
    :action-type (local-tell-1 (e-safe ?i ?z) (nil))
    :observability-conditions {earshot}
  )

  ;--------------------PAGING------------------
  ;
  ; The public address. Everyone hears it, the guest included. Nothing marks
  ; it as dangerous: whether a page is admissible depends on what its hearers
  ; can infer from it, and that is the goal's to decide.

  (:action page-floor
    :parameters (?i - agent | (and (pa-location) (robot ?i)))
    :action-type (public-tell-{nf} {page_floor_events})
    :observability-conditions (default Fully)
  )

  (:action page-column
    :parameters (?i - agent | (and (pa-location) (robot ?i)))
    :action-type (public-tell-{nk} {page_column_events})
    :observability-conditions (default Fully)
  )

  (:action page-safe
    :parameters (?i - agent | (and (pa-all-clear) (robot ?i)))
    :action-type (public-tell-1 (e-page-safe ?i))
    :observability-conditions (default Fully)
  )
)
"""

EARSHOT = '(:forall (?j - agent) (?j (if (at-ag ?j ?z) Fully else Absent)))'


def domain_text(floors, columns):
    values = ' '.join(f'(floor-{f})' for f in floors) + ' ' + \
        ' '.join(f'(col-{k})' for k in columns)
    out = DOMAIN_HEAD.format(nf=len(floors), nk=len(columns), values=values)
    kinds = [('floor', floors, [f'floor-{f}' for f in floors]),
             ('col', columns, [f'col-{k}' for k in columns]),
             ('safe', [''], ['safe'])]
    for template in (TELL_EVENT, PAGE_EVENT):
        for kind, vals, atoms in kinds:
            for v, atom in zip(vals, atoms):
                out += template.format(
                    kind=kind, value=v, atom=atom,
                    speaker_knows=knows_which('?i', atoms),
                    hearer_knows=knows_which('?j', atoms)).replace(f'e-{kind}-\n', f'e-{kind}\n') \
                    .replace(f'e-page-{kind}-\n', f'e-page-{kind}\n')
    return out + TELL_ACTIONS.format(
        nf=len(floors), nk=len(columns), earshot=EARSHOT,
        floor_events=' '.join(f'(e-floor-{f} ?i ?z)' for f in floors),
        column_events=' '.join(f'(e-col-{k} ?i ?z)' for k in columns),
        page_floor_events=' '.join(f'(e-page-floor-{f} ?i)' for f in floors),
        page_column_events=' '.join(f'(e-page-col-{k} ?i)' for k in columns))


# ─── The problems ───────────────────────────────────────────────────────────

# Where each robot can go, from the hotel's three navigation graphs. The
# cleaners' graph is the ground floor alone. The delivery robot's has no lobby
# vertex on the ground floor, so the restaurant is the one place all three
# fleets can reach. The concierge keeps to the ground floor, where the desk is.
GROUND = ('lobby', 'restaurant')

START = {'cleaner': 'restaurant', 'concierge': 'lobby',
         'porter': 'restaurant', 'guest': 'lobby'}


def links(floors, columns):
    rooms = [room(f, k) for f in floors for k in columns]
    out = []

    def both(i, a, b):
        out.append((i, a, b))
        out.append((i, b, a))
    both('concierge', 'lobby', 'restaurant')
    both('cleaner', 'lobby', 'restaurant')
    for z in rooms:
        both('porter', 'restaurant', z)
    for x in range(len(rooms)):
        for y in range(x + 1, len(rooms)):
            both('porter', rooms[x], rooms[y])
    return out


def exactly_one(atoms):
    return '(or ' + ' '.join(
        '(and ' + ' '.join(a if a == b else f'(not {a})' for a in atoms) + ')'
        for b in atoms) + ')'


def everyone(depth, formula, group=RESPONDERS):
    g = ' '.join(group)
    for _ in range(depth):
        formula = f'([({g})] {formula})'
    return formula


def goal_text(fleet, floors, columns, which=None):
    """The goal a fleet plans for, or with @p which one conjunct of the whole
    goal: 'safe', 'stand-down' or 'secret'."""
    rooms = [room(f, k) for f in floors for k in columns]
    conjuncts = {
        'safe': '(safe)',
        'stand-down': everyone(2, '(safe)'),
        'secret': '(and ' + ' '.join(f'(<guest> (source {z}))' for z in rooms) + ')',
    }
    if which:
        return conjuncts[which]
    if fleet == 'siloed':
        return conjuncts['safe']
    if fleet in ('broadcast', 'filter'):
        return f'(and {conjuncts["safe"]} {conjuncts["stand-down"]})'
    return f'(and {conjuncts["safe"]} {conjuncts["stand-down"]} {conjuncts["secret"]})'


PROBLEM = """\
(define (problem hotel-distributed-{fleet})

  ;; {what}
  ;; Written by tools/hotel_domain.py: {nf} floors, {nk} columns.

  (:domain hotel-distributed)

  (:requirements
    :typing :facts :equality :finitary-S5-theories :multi-pointed-models
    :modal-goals :negative-goals :negative-preconditions :knowing-whether
    :list-comprehensions :group-modalities
  )

  (:agents cleaner concierge porter guest)

  (:objects {zones} - zone)

  (:facts-init
    {facts}
  )

  (:init
    (:and
      ;; Where everyone is, and that everyone knows it.
      {at}
      ([C. All] (and {at}))

      ;; The leak is not yet contained, and everyone knows that too.
      (not (safe)) ([C. All] (not (safe)))

      ;; One floor, one column, and the room is the pair.
      ([C. All] {one_floor})
      ([C. All] {one_column})
      ([C. All] (and {sources}))

      ;; The concierge knows the floor, the cleaner the column, and nobody
      ;; else knows either. Which floor and which column is not stated: the
      ;; planner does not know, and every pair is a designated world.
      {knows}
    )
  )

  (:goal {goal})
)
"""

WHAT = {
    'epistemic': 'Every channel, and the whole goal: the leak contained, every responder\n'
                 '  ;; knowing that every responder knows it, and the guest unable to rule out\n'
                 '  ;; any room.',
    'broadcast': 'Every channel, and the guest is not someone to keep in the dark: the goal\n'
                 '  ;; is the leak contained and every responder knowing that every responder\n'
                 '  ;; knows it.',
    'filter': 'No floor or column on the public address, by rule; the all-clear names\n'
              '  ;; neither and is allowed. The goal is the broadcast fleet\'s, the rule\n'
              '  ;; being meant to do the guest\'s part.',
    'siloed': 'No channel between fleets: each knows what it observes and nothing else.\n'
              '  ;; The goal is the leak contained.',
    'pa-only': 'The public address and nothing else, and the whole goal. No policy\n'
               '  ;; exists; tools/reach.py shows it.',
}


def problem_text(fleet, floors, columns, goal=None):
    rooms = [room(f, k) for f in floors for k in columns]
    zones = list(GROUND) + rooms
    facts = [f'(link {i} {a} {b})' for i, a, b in links(floors, columns)]
    facts += [f'(room {z})' for z in rooms] + ['(meeting lobby)', '(meeting restaurant)']
    facts += ['(kit porter)'] + [f'(robot {i})' for i in RESPONDERS]
    facts += [f'({c})' for c in CHANNELS[fleet]]
    at = ' '.join(f'(at-ag {i} {START[i]})' for i in AGENTS) + ' ' + ' '.join(
        f'(not (at-ag {i} {z}))' for i in AGENTS for z in zones if z != START[i])
    sources = ' '.join(
        f'(or (and (source {room(f, k)}) (floor-{f}) (col-{k})) '
        f'(and (not (source {room(f, k)})) (not (and (floor-{f}) (col-{k})))))'
        for f in floors for k in columns)
    sources += ' ' + ' '.join(f'(not (source {z}))' for z in GROUND)
    knows = []
    for f in floors:
        knows.append(f'([C. All] ([Kw. concierge] (floor-{f})))')
        knows += [f'([C. All] (<Kw. {i}> (floor-{f})))' for i in AGENTS if i != 'concierge']
    for k in columns:
        knows.append(f'([C. All] ([Kw. cleaner] (col-{k})))')
        knows += [f'([C. All] (<Kw. {i}> (col-{k})))' for i in AGENTS if i != 'cleaner']
    return PROBLEM.format(
        fleet=fleet, what=WHAT[fleet], nf=len(floors), nk=len(columns),
        zones=' '.join(zones), facts='\n    '.join(facts), at=at,
        one_floor=exactly_one([f'(floor-{f})' for f in floors]),
        one_column=exactly_one([f'(col-{k})' for k in columns]),
        sources=sources, knows='\n      '.join(knows),
        goal=goal or goal_text(fleet, floors, columns))


# ─── What the executor needs ────────────────────────────────────────────────

# Where each robot stands for a zone: the hotel's waypoints. The concierge and
# the porter cannot both stand on the restaurant's waypoint, so the porter
# waits at the kitchen's, six metres away through the same room, and the
# cleaner at its own cleaning dock there.
WAYPOINT = {
    ('concierge', 'lobby'): 'lobby',
    ('concierge', 'restaurant'): 'restaurant',
    ('cleaner', 'lobby'): 'clean_lobby',
    ('cleaner', 'restaurant'): 'clean_restaurant',
    ('porter', 'restaurant'): 'kitchen',
}

DURATION = {'goto_zone': 240.0, 'look_into': 10.0, 'shut_valve': 15.0,
            'say': 6.0, 'page': 7.0}


def spot(agent, zone):
    """The classical zone, which the bridge's task map turns into a waypoint.
    In lower case: the classical side is PDDL, whose names PlanSys2 does not
    keep in mixed case, and the task map gives each its hotel waypoint."""
    return WAYPOINT.get((agent, zone), zone.lower())


def mapping(floors, columns):
    """Ground action name -> the classical action it is executed as."""
    rooms = [room(f, k) for f in floors for k in columns]
    table = {}
    for i, a, b in links(floors, columns):
        table[f'go_{i}_{a}_{b}'] = {
            'action': f'(goto_zone {i} {spot(i, a)} {spot(i, b)})',
            'duration': DURATION['goto_zone']}
    for z in rooms:
        table[f'contain_porter_{z}'] = {'action': f'(shut_valve porter {z.lower()})',
                                        'duration': DURATION['shut_valve']}
        table[f'inspect_porter_{z}'] = {'action': f'(look_into porter {z.lower()})',
                                        'duration': DURATION['look_into']}
    for i in RESPONDERS:
        for z in GROUND:
            for kind in ('floor', 'column', 'safe'):
                table[f'tell-{kind}_{i}_{z}'] = {
                    'action': f'(say_{kind} {i} {spot(i, z)})', 'duration': DURATION['say']}
        for kind in ('floor', 'column', 'safe'):
            table[f'page-{kind}_{i}'] = {'action': f'(page_{kind} {i})',
                                         'duration': DURATION['page']}
    return dict(sorted(table.items()))


PDDL = """\
;; The classical half of the hotel's distributed leak, written by
;; tools/hotel_domain.py. Nothing here knows where the leak is, who knows the
;; floor and who the column, or that the guest is listening. That is in the
;; EPDDL, and the executor checks it against the model.

(define (domain hotel-distributed)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  zone
)

(:predicates
  (robot_at ?r - robot ?z - zone)
  (looked ?r - robot ?z - zone)
  (shut_off ?r - robot ?z - zone)
  (said ?r - robot ?z - zone)
  (paged ?r - robot)
)

;; Long, because a ride between floors is a lift call and a wait as well.
;; Open-RMF decides how long it takes and the bridge waits for it.
(:durative-action goto_zone
  :parameters (?r - robot ?from ?to - zone)
  :duration (= ?duration 240)
  :condition (and (at start (robot_at ?r ?from)))
  :effect (and (at start (not (robot_at ?r ?from))) (at end (robot_at ?r ?to)))
)

(:durative-action look_into
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 10)
  :condition (and (over all (robot_at ?r ?z)))
  :effect (and (at end (looked ?r ?z)))
)

(:durative-action shut_valve
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 15)
  :condition (and (over all (robot_at ?r ?z)))
  :effect (and (at end (shut_off ?r ?z)))
)
{say}
{page}
)
"""

SAY = """
(:durative-action say_{kind}
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 6)
  :condition (and (at start (robot_at ?r ?z)))
  :effect (and (at end (said ?r ?z)))
)
"""

PAGE = """
(:durative-action page_{kind}
  :parameters (?r - robot)
  :duration (= ?duration 7)
  :condition (and)
  :effect (and (at end (paged ?r)))
)
"""


def pddl_text():
    return PDDL.format(say=''.join(SAY.format(kind=k) for k in ('floor', 'column', 'safe')),
                       page=''.join(PAGE.format(kind=k) for k in ('floor', 'column', 'safe')))


def write(out, nf=2, nk=2, executor=False, fleets=FLEETS):
    """The library, the domain and a problem per fleet; with @p executor the
    mapping and the classical model too. Returns the paths by role."""
    floors, columns = floors_of(nf), columns_of(nk)
    os.makedirs(out, exist_ok=True)
    paths = {'library': os.path.join(out, 'earshot.epddl'),
             'domain': os.path.join(out, 'hotel-distributed.epddl')}
    with open(paths['library'], 'w') as fh:
        fh.write(library_text([1, nf, nk]))
    with open(paths['domain'], 'w') as fh:
        fh.write(domain_text(floors, columns))
    for fleet in fleets:
        paths[fleet] = os.path.join(out, f'{fleet}.epddl')
        with open(paths[fleet], 'w') as fh:
            fh.write(problem_text(fleet, floors, columns))
    if executor:
        paths['mapping'] = os.path.join(out, 'mapping.json')
        with open(paths['mapping'], 'w') as fh:
            json.dump(mapping(floors, columns), fh, indent=2)
            fh.write('\n')
        paths['pddl'] = os.path.join(out, 'hotel-distributed.pddl')
        with open(paths['pddl'], 'w') as fh:
            fh.write(pddl_text())
    return paths


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--floors', type=int, default=2)
    ap.add_argument('--columns', type=int, default=2)
    ap.add_argument('--out', required=True)
    ap.add_argument('--executor', action='store_true',
                    help='also write the action mapping and the classical model')
    args = ap.parse_args()
    for role, path in write(args.out, args.floors, args.columns, args.executor).items():
        print(f'{role:9s} {path}')


if __name__ == '__main__':
    main()
