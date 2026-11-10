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
The nights of the ward, the fleet, and the two ways a fleet may hold that a
reading can be wrong.

A night is the two doors the porter shut on his round and the set of scouts
whose laser is short-sighted. A sound scout reads a door from its passage's
mouth as it is. A short-sighted one gets no return from a leaf that far away
and reads every door open; read from the mark before the door, 1.2 m off,
every laser is right. So a reading "shut" is never wrong, and a reading
"open" from a mouth is as good as the sensor that took it.

Nobody knows which doors are shut, nor which lasers are short-sighted, the
scout itself included. What the fleet takes to be possible, and how plausible
(a Spohn rank, 0 for what it believes), is its model of error:

    sensors     an error belongs to a sensor. A world is the porter's round and
                the set of short-sighted scouts; its rank is the sum of their
                grades, 1 for a laser of the old series and 2 for the new.
    readings    an error belongs to a reading. A world is the porter's round
                and, for each shut door, which of the first and the second look
                at it from its mouth errs; its rank is the number of looks that
                err. No reading says anything of another.

A bed-mover drives into a passage only when it is sure enough that the door
is open. The formulations differ in the model and in what sure enough is:

    credulous   sensors; belief: the door is open in the most plausible worlds.
                A fleet that takes a reading for the fact acts so.
    readings    readings; firm belief of degree 2: open in every world within
                one rank of the most plausible.
    sensors     sensors; firm belief of degree 2.
    contact     sensors; knowledge, and no scout looks from a mouth: a door is
                read from its mark only.

The frequencies the expectations are taken under: every laser of the old
series is short-sighted one night in nine (odds EPS to 1) and one of the new
series one night in sixty-five (odds EPS squared to 1), independently, and
the porter's six rounds are equally frequent.
"""

import itertools

DOORS = ('a', 'b', 'c', 'd')
LOOPS = [('c', 'd'), ('a', 'b'), ('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')]
EPS = 0.125
GRADES = 6                   # grade atoms rank0..rank5
LOOKS = 2                    # the readings model tells the first look at a door from the second
FORMULATIONS = ('credulous', 'readings', 'sensors', 'contact')
MODEL = {'credulous': 'sensors', 'readings': 'readings', 'sensors': 'sensors', 'contact': 'sensors'}
FIRM = {'credulous': 1, 'readings': 2, 'sensors': 2, 'contact': None}       # None: knowledge
# what a formulation weighs a world of rank k by when it plans: the credulous fleet
# takes its readings for facts, and plans as if no laser were short-sighted
PLAN_EPS = {'credulous': 1e-4, 'readings': EPS, 'sensors': EPS, 'contact': EPS}


class Robot:
    """A scout (it reads doors; grade is the rank of its laser being short-sighted) or
    a bed-mover (it carries a bed from its room to a ward, and reads a door only from
    the mark)."""

    def __init__(self, name, role, room, ward=None, grade=None, what=''):
        self.name, self.role, self.room, self.ward, self.grade, self.what = name, role, room, ward, grade, what

    def __repr__(self):
        return f'{self.name}({self.role}, {self.room}' + (f'->ward{self.ward})' if self.ward else ')')


def scout(name, room, grade, what):
    return Robot(name, 'scout', room, grade=grade, what=what)


def mover(name, room, ward, what):
    return Robot(name, 'mover', room, ward=ward, what=what)


# Rooms: 'hd' the day room (west), 'ld' the porters' lodge, 'ht' the treatment room
# (middle), 'hp' the pharmacy (east). Wards 1 to 4 from west to east.
FLEETS = {
    1: [mover('b', 'ht', 3, 'a bed-mover'), scout('s', 'ld', 1, 'a scout of the old series')],
    2: [mover('b', 'ht', 2, 'a bed-mover'), scout('s', 'ld', 1, 'a scout of the old series'),
        scout('t', 'hd', 1, 'a scout of the old series')],
    3: [mover('b', 'ht', 2, 'a bed-mover'), mover('g', 'hd', 4, 'a second bed-mover'),
        scout('s', 'ld', 1, 'a scout of the old series'), scout('u', 'hp', 2, 'a scout of the new series')],
}


def fleet(act):
    return FLEETS[act]


def robots(act):
    return [r.name for r in FLEETS[act]]


def scouts(act):
    return [r for r in FLEETS[act] if r.role == 'scout']


def movers(act):
    return [r for r in FLEETS[act] if r.role == 'mover']


# -- nights --------------------------------------------------------------------------------

def nights(act):
    """Every night of an act: (round, the short-sighted scouts)."""
    names = [r.name for r in scouts(act)]
    out = []
    for loop in LOOPS:
        for k in range(len(names) + 1):
            for f in itertools.combinations(names, k):
                out.append((loop, frozenset(f)))
    return out


def tag(night):
    loop, faulty = night
    return ''.join(loop).upper() + ('-' + ''.join(sorted(faulty)) if faulty else '')


def night_of(act, text):
    for n in nights(act):
        if tag(n) == text:
            return n
    raise ValueError(text)


def frequency(act, night):
    loop, faulty = night
    p = 1.0 / len(LOOPS)
    for r in scouts(act):
        odds = EPS ** r.grade
        q = odds / (1 + odds)
        p *= q if r.name in faulty else 1 - q
    return p


def open_at(loop, door):
    return door not in loop


def reads_shut(night, scout_name, door):
    """What a look from the mouth reads on a night: shut only for a sound laser at a shut door."""
    loop, faulty = night
    return door in loop and scout_name not in faulty


# -- the models ----------------------------------------------------------------------------

class Model:
    """A model of error for an act: its worlds, their ranks, and what a look reads in each."""

    def __init__(self, act, kind):
        self.act, self.kind = act, kind
        self.scouts = [r.name for r in scouts(act)]
        grade = {r.name: r.grade for r in scouts(act)}
        self.worlds = []
        if kind == 'sensors':
            for loop, faulty in nights(act):
                self.worlds.append((loop, faulty))
            self.rank = [sum(grade[r] for r in w[1]) for w in self.worlds]
        else:
            for loop in LOOPS:
                slots = [(d, k) for d in loop for k in range(1, LOOKS + 1)]
                for n in range(len(slots) + 1):
                    for errs in itertools.combinations(slots, n):
                        self.worlds.append((loop, frozenset(errs)))
            self.rank = [len(w[1]) for w in self.worlds]
        self.n = len(self.worlds)
        self.all = (1 << self.n) - 1
        self.open = {d: self.mask(lambda w, d=d: d not in w[0]) for d in DOORS}

    def mask(self, f):
        m = 0
        for i, w in enumerate(self.worlds):
            if f(w):
                m |= 1 << i
        return m

    def shut_reading(self, scout_name, door, k):
        """The worlds where the k-th look at the door from its mouth, by this scout, reads shut."""
        if self.kind == 'sensors':
            return self.mask(lambda w: door in w[0] and scout_name not in w[1])
        return self.mask(lambda w: door in w[0] and (door, k) not in w[1])

    def name(self, i):
        loop, extra = self.worlds[i]
        s = ''.join(loop).upper()
        if self.kind == 'sensors':
            return s + ('-' + ''.join(sorted(extra)) if extra else '')
        return s + ''.join(f'-{d}{k}' for d, k in sorted(extra))

    def atoms(self, i):
        """The facts of a world besides the doors, as EPDDL atoms (tools/model.py)."""
        loop, extra = self.worlds[i]
        if self.kind == 'sensors':
            return [f'(short {r})' for r in sorted(extra)]
        return [f'(err-{d}-{k})' for d, k in sorted(extra)]


_MODELS = {}


def model(act, formulation):
    key = (act, MODEL[formulation])
    if key not in _MODELS:
        _MODELS[key] = Model(act, MODEL[formulation])
    return _MODELS[key]


if __name__ == '__main__':
    for act in sorted(FLEETS):
        print(f'act {act}: {FLEETS[act]}')
        for kind in ('sensors', 'readings'):
            m = Model(act, kind)
            hist = {}
            for k in m.rank:
                hist[k] = hist.get(k, 0) + 1
            print(f'  {kind}: {m.n} worlds, by rank {dict(sorted(hist.items()))}')
        print('  nights:', ' '.join(f'{tag(n)}:{frequency(act, n):.4f}' for n in nights(act)[:6]), '...')
