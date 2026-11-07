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
The worlds of the ward, the robots of the fleet, and how each ranks the worlds.

A world is the loop the night porter walked at one: north through one fire
door and south through another, shutting both. Nobody sees him. Every robot
knows the six loops are possible; how plausible it finds each is its own
Spohn rank (an ordinal conditional function), 0 for what it believes and
higher for what would surprise it more. The fleet is mixed: robots from
different makers, each with the ranking its maker ships.

    explanation    the hospital's own courier, c, ranks by the porter's habits:
                   the rostered loop C-D rank 0, the west loop A-B rank 1, the
                   four others rank 2.
    door-by-door   the disinfection robot, d, keeps a map of independent cells:
                   a loop's rank is the number of doors whose state differs from
                   the rostered picture (Dalal's distance).
    knowledge      the linen robot, e, ships with no ranking: every loop rank 0,
                   so it believes only what it knows, and routes counting every
                   loop it cannot rule out alike.

A robot may also take another's word. When a speaker reports a belief, a
listener that takes its word adds its trust, ALPHA, to the rank of every world
the belief rules out: Spohn's conditioning on evidence of strength ALPHA, the
same as conditioning on the report in a model where a wrong report has rank
ALPHA. A listener that does not take the speaker's word is not moved. Who takes
whose word is the fleet's TRUST, and every robot knows it, as it knows every
robot's ranking.

The frequencies the expectations are taken under are the porter's habits,
EPS ** rank of the explanation ranking, normalised (as in the ward's first
demonstration).
"""

DOORS = ('a', 'b', 'c', 'd')
LOOPS = [('c', 'd'), ('a', 'b'), ('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')]
ROSTERED = ('c', 'd')
RANKINGS = ('explanation', 'door-by-door', 'full-meet', 'knowledge')
EPS = 0.125
ALPHA = 3                    # a listener's trust: the rank of a speaker's belief being wrong
GRADES = 10                  # grade atoms 0..9 per robot; tools/policy_check.py checks no cap binds

_Z = 1 + EPS + 4 * EPS ** 2
FREQ = {('c', 'd'): 1 / _Z, ('a', 'b'): EPS / _Z}           # 0.842 and 0.105
FREQ_OTHER = EPS ** 2 / _Z                                 # 0.013 each


class Robot:
    """A robot of the fleet: its ranking, the room it starts in and the ward it delivers to."""

    def __init__(self, name, ranking, room, ward, maker):
        self.name, self.ranking, self.room, self.ward, self.maker = name, ranking, room, ward, maker

    def __repr__(self):
        return f'{self.name}({self.ranking}, {self.room}->ward{self.ward})'


# The acts. Rooms: 'hd' the day room (west), 'ht' the treatment room (middle),
# 'hp' the pharmacy (east). Wards 1 to 4 from west to east.
COURIER = Robot('c', 'explanation', 'hd', 4, 'the hospital')
DISINFECTOR = Robot('d', 'door-by-door', 'hp', 2, 'a disinfection contractor')
LINEN = Robot('e', 'knowledge', 'ht', 1, 'a linen contractor')
FLEETS = {1: [COURIER, DISINFECTOR], 2: [COURIER, LINEN], 3: [COURIER, DISINFECTOR, LINEN]}
# (listener, speaker): the listener takes the speaker's word with trust ALPHA.
TRUST = {1: {('d', 'c')}, 2: {('e', 'c')}, 3: {('d', 'c'), ('e', 'c')}}

FORMULATIONS = ('egocentric', 'relevance', 'perspective')
ALSO = ('alone',)              # the 'alone' policy, checked identical to 'egocentric' by validate.sh


def worlds():
    return list(LOOPS)


def name(w):
    return ''.join(w).upper()


def open_at(w, door):
    return door not in w


def valuation(w):
    return tuple(d not in w for d in DOORS)


def hamming(u, v):
    return sum(a != b for a, b in zip(u, v))


def rank(ranking, w):
    if ranking == 'knowledge':
        return 0
    if ranking == 'explanation':
        return 0 if w == ROSTERED else 1 if w == ('a', 'b') else 2
    if ranking == 'door-by-door':
        return hamming(valuation(w), valuation(ROSTERED))
    if ranking == 'full-meet':
        return 0 if w == ROSTERED else 1
    raise ValueError(ranking)


def frequency(w):
    return FREQ.get(w, FREQ_OTHER)


def fleet(act, formulation=None):
    """The act's robots, whatever the formulation: the formulations differ only in what
    the courier says."""
    return FLEETS[act]


def robots(act):
    return [r.name for r in FLEETS[act]]


def trust(act):
    return TRUST[act]


def describe(act, formulation=None):
    return {r.name: {name(w): rank(r.ranking, w) for w in worlds()} for r in fleet(act, formulation)}


if __name__ == '__main__':
    for act in (1, 2, 3):
        print(f'act {act}: trust {sorted(TRUST[act])}')
        for r, ranks in describe(act).items():
            print(f'  {r}', ' '.join(f'{k}:{v}' for k, v in ranks.items()))
