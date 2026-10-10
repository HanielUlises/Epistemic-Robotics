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
Maps that go stale, for a fleet of any size.

    stale_maps.py --out DIR                      the floor of tools/layout.py
    stale_maps.py --out DIR --robots 12 --seed 3 a random fleet, for scaling

writes the domain `stale-maps.epddl` and four floors. Three differ in two
lines, whether the robots have a radio and whether they know their maps go
stale; the fourth differs from the first in its goal:

  radio    a radio, and robots that take their map to be the floor
  silent   no radio, and the same robots
  doubt    no radio, and robots that know the floor may have changed
  resync   a radio, and the goal that every robot's map be current

At the start of the shift every robot holds the same map of the floor: the
shift map, in which bay t1 is open and t2 and t3 hold staged loads. During
the shift a forklift stages a load in t1 and takes the load out of t3. A
robot sees a change when its station has a sight line into the bay, which
tools/layout.py decides by casting rays over the floor plan; every other
robot's map still shows the bay as it was. Which robot is stationed where is
common knowledge, since the fleet's positions are, and so is the forklift's
schedule: what a robot does not know is whether the forklift used its slot.

The haulers then take loads through the racking. A hauler crosses a bay only
if the bay is open and the hauler believes it is open, and the goal is that
every hauler has crossed. After the forklift only t3 is open, and the shift
map says only t1 is.
"""

import argparse
import json
import os
import random

DOMAIN = """\
(define (domain stale-maps)

  ;; A fleet whose maps go stale. Written by tools/stale_maps.py; the README
  ;; says why each part is as it is.

  (:requirements
    :KD45-frames :typing :equality :partial-observability :list-comprehensions
    :lists :ontic-actions :negative-preconditions :modal-preconditions
  )

  (:action-type-libraries intermediate maps)

  (:types bay)

  (:predicates
    ;; A load stands in the bay, and nothing can be taken through it.
    (blocked ?t - bay)
    ;; The robot's station has a sight line into the bay. Static, and common
    ;; knowledge: the fleet's positions are.
    (sees ?i - agent ?t - bay)
    (hauls ?i - agent)
    (delivered ?i - agent)
    ;; The forklift's slots, as the schedule has them. Common knowledge;
    ;; whether a slot was used is not.
    (stage-due ?t - bay)
    (clear-due ?t - bay)
    ;; The floor: a radio between the robots, and robots that know their
    ;; maps may go stale. Static, and common knowledge.
    (radio)
    (doubts)
  )

  (:event nil)

  ;--------------------THE FORKLIFT------------------
  ;
  ; A robot that sees the bay sees the change. One that does not either takes
  ; the slot to have passed with nothing moved, which leaves its map stale and
  ; its belief false, or, on the doubting floor, does not know which.
  ;
  ; Each change is two actions, one per floor, and not one action with an
  ; else-if over the floor: plank grounds an else-if branch without its own
  ; condition, and puts that condition, and its negation, into the else.

  (:event e-stage-sure
    :parameters (?t - bay)
    :precondition (and (not (doubts)) (stage-due ?t) (not (blocked ?t)))
    :effects (:and (blocked ?t) (not (stage-due ?t)))
  )

  (:event e-stage-slot-sure
    :parameters (?t - bay)
    :precondition (and (not (doubts)) (stage-due ?t))
    :effects (:and (not (stage-due ?t)))
  )

  (:action stage
    :parameters (?t - bay)
    :action-type (change (e-stage-sure ?t) (e-stage-slot-sure ?t))
    :observability-conditions
      (:forall (?j - agent) (?j (if (sees ?j ?t) Fully else Oblivious)))
  )

  (:event e-stage-doubted
    :parameters (?t - bay)
    :precondition (and (doubts) (stage-due ?t) (not (blocked ?t)))
    :effects (:and (blocked ?t) (not (stage-due ?t)))
  )

  (:event e-stage-slot-doubted
    :parameters (?t - bay)
    :precondition (and (doubts) (stage-due ?t))
    :effects (:and (not (stage-due ?t)))
  )

  (:action stage-doubted
    :parameters (?t - bay)
    :action-type (change (e-stage-doubted ?t) (e-stage-slot-doubted ?t))
    :observability-conditions
      (:forall (?j - agent) (?j (if (sees ?j ?t) Fully else Doubting)))
  )

  (:event e-clear-sure
    :parameters (?t - bay)
    :precondition (and (not (doubts)) (clear-due ?t) (blocked ?t))
    :effects (:and (not (blocked ?t)) (not (clear-due ?t)))
  )

  (:event e-clear-slot-sure
    :parameters (?t - bay)
    :precondition (and (not (doubts)) (clear-due ?t))
    :effects (:and (not (clear-due ?t)))
  )

  (:action clear
    :parameters (?t - bay)
    :action-type (change (e-clear-sure ?t) (e-clear-slot-sure ?t))
    :observability-conditions
      (:forall (?j - agent) (?j (if (sees ?j ?t) Fully else Oblivious)))
  )

  (:event e-clear-doubted
    :parameters (?t - bay)
    :precondition (and (doubts) (clear-due ?t) (blocked ?t))
    :effects (:and (not (blocked ?t)) (not (clear-due ?t)))
  )

  (:event e-clear-slot-doubted
    :parameters (?t - bay)
    :precondition (and (doubts) (clear-due ?t))
    :effects (:and (not (clear-due ?t)))
  )

  (:action clear-doubted
    :parameters (?t - bay)
    :action-type (change (e-clear-doubted ?t) (e-clear-slot-doubted ?t))
    :observability-conditions
      (:forall (?j - agent) (?j (if (sees ?j ?t) Fully else Doubting)))
  )

  ;--------------------SENDING A MAP------------------
  ;
  ; A robot sends its map of a bay to another. It sends what its map says,
  ; which is what it believes, and it sends only to a robot it believes holds
  ; the opposite: the precondition is a belief about a belief, and it is what
  ; keeps a stale map from being sent over a fresh one. The recipient takes a
  ; reading that contradicts its own map as news that the bay changed.

  (:event e-says-open
    :parameters (?i ?j - agent ?t - bay)
    :precondition (and (radio) (settled)
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
    :precondition (and (radio) (settled)
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

  ;--------------------LOOKING------------------
  ;
  ; A robot drives to the bay's mouth and reads it with its own laser. Only
  ; on the doubting floor: a robot sure of its map has no reason to look, and
  ; one that looked and found its map wrong would be left with no world.

  (:event e-sees-open
    :parameters (?i - agent ?t - bay)
    :precondition (and (doubts) (settled) (not (blocked ?t)) (<Kw. ?i> (blocked ?t)))
  )

  (:event e-sees-blocked
    :parameters (?i - agent ?t - bay)
    :precondition (and (doubts) (settled) (blocked ?t) (<Kw. ?i> (blocked ?t)))
  )

  (:action look
    :parameters (?i - agent ?t - bay)
    :action-type (private-sensing (e-sees-open ?i ?t) (e-sees-blocked ?i ?t) (nil))
    :observability-conditions (:and (?i Fully) (default Oblivious))
  )

  ;--------------------CROSSING------------------
  ;
  ; The knowledge precondition. The other robots are at their own work and do
  ; not watch the crossing.

  (:event e-cross
    :parameters (?h - agent ?t - bay)
    :precondition (and (hauls ?h) (settled) (not (delivered ?h))
                       (not (blocked ?t)) ([?h] (not (blocked ?t))))
    :effects (:and (delivered ?h))
  )

  (:action cross
    :parameters (?h - agent ?t - bay)
    :action-type (private-ontic (e-cross ?h ?t) (nil))
    :observability-conditions (:and (?h Fully) (default Oblivious))
  )
)
"""

PROBLEM = """\
(define (problem stale-maps-{floor}-n{n})

  (:domain stale-maps)

  ;; {n} robots, {haulers} of them haulers, {where}. Written by
  ;; tools/stale_maps.py.

  (:requirements
    :KD45-frames :typing :equality :list-comprehensions
    :finitary-S5-theories :modal-goals :multi-pointed-models :negative-preconditions
  )

  (:agents {agents})

  (:objects {bays} - bay)

  (:init
    (:and
      ([C. All] ({radio}))
      ([C. All] ({doubts}))
      ;; The shift map, which every robot holds.
{shift}
      ;; The forklift's schedule.
{schedule}
      ;; The stations' sight lines into the bays.
{sees}
{hauls}
      (:forall (?i - agent) ([C. All] (not (delivered ?i))))
    )
  )

  (:goal (and {goal}))
)
"""


def settled_macro(bays):
    """plank has no derived predicates; `(settled)` is written out."""
    return '(and ' + ' '.join(
        f'(not (stage-due {t})) (not (clear-due {t}))' for t in bays) + ')'


def domain_text(bays):
    return DOMAIN.replace('(settled)', settled_macro(bays))


class Fleet:
    """One instance: who sees which bay, who hauls, what changes."""

    def __init__(self, agents, bays, shift_blocked, changes, sees, haulers):
        self.agents = list(agents)
        self.bays = list(bays)
        self.shift_blocked = set(shift_blocked)
        self.changes = dict(changes)            # bay -> 'stage' | 'clear'
        self.sees = {t: set(sees.get(t, ())) for t in bays}
        self.haulers = list(haulers)

    def actual_blocked(self):
        out = set(self.shift_blocked)
        for t, kind in self.changes.items():
            (out.add if kind == 'stage' else out.discard)(t)
        return out

    def map_of(self, agent):
        """The bays an agent's map shows blocked, under the static-world
        assumption: the shift map with the changes it saw applied."""
        out = set(self.shift_blocked)
        for t, kind in self.changes.items():
            if agent in self.sees[t]:
                (out.add if kind == 'stage' else out.discard)(t)
        return out

    def stale(self):
        """(agent, bay) for every bay an agent's map has wrong."""
        actual = self.actual_blocked()
        return [(a, t) for a in self.agents for t in self.bays
                if (t in self.map_of(a)) != (t in actual)]

    def to_json(self):
        return {'agents': self.agents, 'bays': self.bays,
                'shift_blocked': sorted(self.shift_blocked), 'changes': self.changes,
                'sees': {t: sorted(s) for t, s in self.sees.items()},
                'haulers': self.haulers}

    @staticmethod
    def from_json(d):
        return Fleet(d['agents'], d['bays'], d['shift_blocked'], d['changes'],
                     d['sees'], d['haulers'])

    def problem_text(self, floor):
        doubts = floor == 'doubt'
        ck = '      ([C. All] {})'.format
        shift = '\n'.join(ck(f'(blocked {t})' if t in self.shift_blocked
                             else f'(not (blocked {t}))') for t in self.bays)
        schedule = []
        for t in self.bays:
            kind = self.changes.get(t)
            schedule.append(ck(f'(stage-due {t})' if kind == 'stage' else f'(not (stage-due {t}))'))
            schedule.append(ck(f'(clear-due {t})' if kind == 'clear' else f'(not (clear-due {t}))'))
        sees = '\n'.join(ck(f'(sees {a} {t})' if a in self.sees[t] else f'(not (sees {a} {t}))')
                         for t in self.bays for a in self.agents)
        hauls = '\n'.join(ck(f'(hauls {a})' if a in self.haulers else f'(not (hauls {a}))')
                          for a in self.agents)
        radio = floor in ('radio', 'resync')
        where = {'radio': 'with a radio, and maps taken to be the floor',
                 'resync': 'with a radio, and every map to be brought up to date',
                 'silent': 'with no radio, and maps taken to be the floor',
                 'doubt': 'with no radio, and robots that know maps go stale'}[floor]
        return PROBLEM.format(
            floor=floor, n=len(self.agents), haulers=len(self.haulers), where=where,
            agents=' '.join(self.agents), bays=' '.join(self.bays),
            radio='radio' if radio else 'not (radio)',
            doubts='doubts' if doubts else 'not (doubts)',
            shift=shift, schedule='\n'.join(schedule), sees=sees, hauls=hauls,
            goal=self.goal_text(floor))

    def goal_text(self, floor):
        if floor != 'resync':
            return ' '.join(f'(delivered {h})' for h in self.haulers)
        actual = self.actual_blocked()
        return '\n         '.join(
            ' '.join(f'([{a}] (blocked {t}))' if t in actual else f'([{a}] (not (blocked {t})))'
                     for t in self.bays)
            for a in self.agents)


FLOORS = ('radio', 'silent', 'doubt', 'resync')


def random_fleet(n, haulers, seed, p_see=0.3):
    """A fleet of @p n robots on the three bays, each robot seeing each bay
    with probability @p p_see, for the scaling table. The changes are the
    floor's: t1 staged, t3 cleared."""
    rng = random.Random(seed)
    agents = [f'r{i + 1}' for i in range(n)]
    bays = ['t1', 't2', 't3']
    sees = {t: [a for a in agents if rng.random() < p_see] for t in ('t1', 't3')}
    return Fleet(agents, bays, ['t2', 't3'], {'t1': 'stage', 't3': 'clear'}, sees,
                 rng.sample(agents, haulers))


def write(out, fleet, floors=FLOORS):
    os.makedirs(out, exist_ok=True)
    paths = {'domain': os.path.join(out, 'stale-maps.epddl')}
    with open(paths['domain'], 'w') as fh:
        fh.write(domain_text(fleet.bays))
    for floor in floors:
        paths[floor] = os.path.join(out, f'{floor}.epddl')
        with open(paths[floor], 'w') as fh:
            fh.write(fleet.problem_text(floor))
    paths['fleet'] = os.path.join(out, 'fleet.json')
    with open(paths['fleet'], 'w') as fh:
        json.dump(fleet.to_json(), fh, indent=2)
        fh.write('\n')
    return paths


def floor_fleet():
    """The fleet tools/layout.py stations, with its sight lines."""
    import layout  # noqa: E402  (the floor is optional for a random fleet)
    return layout.fleet()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--robots', type=int, help='a random fleet of this size')
    ap.add_argument('--haulers', type=int, default=3)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--fleet', help='a fleet.json to write the floors for')
    args = ap.parse_args()
    if args.fleet:
        fleet = Fleet.from_json(json.load(open(args.fleet)))
    elif args.robots:
        fleet = random_fleet(args.robots, args.haulers, args.seed)
    else:
        fleet = floor_fleet()
    for role, path in write(args.out, fleet).items():
        print(f'{role:7s} {path}')
    print('stale  ' + ' '.join(f'{a}:{t}' for a, t in fleet.stale()))


if __name__ == '__main__':
    main()
