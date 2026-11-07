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
The worlds of the ward, and four ways of ranking them.

A world is what happened tonight: the loop the porter walked at one, and in
the third act whether at two he walked it back and opened its doors ("reopen")
or was held up ("skip"). Nobody sees him. Every robot knows the six loops and
the two endings are possible; what differs between the formulations is only how
plausible each robot finds each world, a Spohn rank (an ordinal conditional
function): 0 for what it believes, higher for what would surprise it more.

    explanation    the porter's own habits. The rostered loop C-D is rank 0,
                   the west loop A-B rank 1, the four others rank 2; at two he
                   reopens (0) more often than he is held up (1). The rank of
                   a night is the sum: the two rounds are independent.
    door-by-door   minimal change (Dalal): the number of doors whose state
                   differs from what the robot expects, the rostered loop at
                   one and nothing changing at two. This is what a map with
                   independent cells does.
    full-meet      the rostered picture is believed, anything else is rank 1:
                   a KD45 belief that, contradicted, falls back on what is
                   known.
    knowledge      every world rank 0: no belief beyond knowledge.

Belief is what holds at the least-ranked worlds a robot cannot rule out, and
conditioning on an observation is restricting to the worlds that agree with
it. That is all the revision there is: the ranks never change.

The frequencies are what the expectations on the page are taken under: the
porter's habits as frequencies EPS ** rank, normalised, so that the
explanation's ranks are their orders of magnitude. tools/sensitivity.py
varies them.
"""

import itertools

DOORS = ('a', 'b', 'c', 'd')
LOOPS = [('c', 'd'), ('a', 'b'), ('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')]
ROSTERED = ('c', 'd')
PLANS = ('reopen', 'skip')
FORMULATIONS = ('explanation', 'door-by-door', 'full-meet', 'knowledge')
EPS = 0.125
_Z = 1 + EPS + 4 * EPS ** 2
FREQ_LOOP = {('c', 'd'): 1 / _Z, ('a', 'b'): EPS / _Z}     # 0.842 and 0.105
FREQ_OTHER = EPS ** 2 / _Z                                # 0.013 each
FREQ_PLAN = {'reopen': 1 / (1 + EPS), 'skip': EPS / (1 + EPS)}   # 0.889 and 0.111


def worlds(act):
    """The worlds of an act: loops, and for act 3 the loop and what happened at two."""
    if act == 3:
        return [(lp, p) for lp in LOOPS for p in PLANS]
    return [(lp, None) for lp in LOOPS]


def loop_name(lp):
    return ''.join(lp).upper()


def name(w):
    lp, p = w
    return loop_name(lp) + ('' if p is None else ('+' if p == 'reopen' else '-'))


def open_at(w, door, after_round2):
    """Whether a door is open in world w, before or after the porter's round at two."""
    lp, p = w
    if after_round2 and p == 'reopen':
        return True
    return door not in lp


def valuation(lp):
    return tuple(d not in lp for d in DOORS)


def hamming(u, v):
    return sum(a != b for a, b in zip(u, v))


def rank(formulation, w):
    lp, p = w
    if formulation == 'knowledge':
        return 0
    if formulation == 'explanation':
        r = 0 if lp == ROSTERED else 1 if lp == ('a', 'b') else 2
        return r + (0 if p in (None, 'reopen') else 1)
    if formulation == 'door-by-door':
        r = hamming(valuation(lp), valuation(ROSTERED))
        changed = len(lp) if p == 'reopen' else 0
        return r + changed
    if formulation == 'full-meet':
        return 0 if lp == ROSTERED and p in (None, 'skip') else 1
    raise ValueError(formulation)


def frequency(w):
    lp, p = w
    f = FREQ_LOOP.get(lp, FREQ_OTHER)
    return f * (1.0 if p is None else FREQ_PLAN[p])


def believed(formulation, cell):
    """The worlds of a cell a robot believes possible: those of least rank."""
    m = min(rank(formulation, w) for w in cell)
    return [w for w in cell if rank(formulation, w) == m]


def believes(formulation, cell, prop):
    return all(prop(w) for w in believed(formulation, cell))


def entertains(formulation, cell, prop):
    """Not believing not-prop: prop holds at some believed world."""
    return any(prop(w) for w in believed(formulation, cell))


def knows(cell, prop):
    return all(prop(w) for w in cell)


def max_rank(formulation, act):
    return max(rank(formulation, w) for w in worlds(act))


def describe(formulation, act):
    return {name(w): rank(formulation, w) for w in worlds(act)}


if __name__ == '__main__':
    for act in (1, 3):
        print(f'act {act}')
        for f in FORMULATIONS:
            print(f'  {f:13s}', ' '.join(f'{k}:{v}' for k, v in describe(f, act).items()))
    # the beliefs after the first surprise, B found shut
    for f in FORMULATIONS:
        cell = [w for w in worlds(1) if not open_at(w, 'b', False)]
        b = believed(f, cell)
        print(f'B shut, {f:13s} believes', [name(w) for w in b], 'open:',
              [d for d in DOORS if believes(f, cell, lambda w, d=d: open_at(w, d, False))])
    _ = itertools
