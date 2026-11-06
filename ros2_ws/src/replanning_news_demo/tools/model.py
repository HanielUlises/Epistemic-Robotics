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
The three problems of replanning on news, their Kripke models, and the news.

    model.py --out DIR        writes act1.epddl .. act3.epddl, and the same
                              problems restricted by each act's news,
                              act1-news.epddl .. act3-news.epddl

Each problem's initial state is given explicitly. A world is a valuation of
the three doors, at least one of them open, and in the third act also whether
the cleaner saw the east door on its round. That last bit is no atom of the
domain: it is a difference in what the cleaner knows and in nothing else, so
two worlds can share a valuation and differ only in the cleaner's relation.

The news of each act:

  act 1  the courier alone       the east door is shut         a literal
  act 2  courier and scout       the middle and east doors     a relation
                                 are on one circuit: both
                                 open or both shut
  act 3  and the cleaner         the cleaner has checked the   knowledge,
                                 east door (its message is     with no fact
                                 cut off before the answer)

A public announcement keeps the worlds where the news holds; restrict() does
the same here, so that the policies planned after the news can be checked
without a simulator.
"""

import argparse
import itertools
import os

DOORS = ('w', 'm', 'e')
ROBOTS = {1: ('c',), 2: ('c', 's'), 3: ('c', 's', 'k')}
ROLE = {'c': 'courier', 's': 'scout', 'k': 'cleaner'}
START = {'c': 'st', 's': 'ks'}            # the cleaner has no place in the domain: it does not drive
NEWS_TEXT = {1: 'The door panel: the east fire door is shut.',
             2: 'Maintenance: the middle and east fire doors hang on one circuit, both open or both shut.',
             3: 'Cleaner k: I have checked the east door, it is ... [the message breaks off]'}
NEWS_KIND = {1: 'a literal', 2: 'a relation', 3: 'knowledge'}
# What the executor's epistemic state is told, in its own syntax: grounded atoms.
ANNOUNCE = {1: '(not open-e)', 2: '(and (or open-m (not open-e)) (or open-e (not open-m)))',
            3: '(Kw k open-e)'}


def valuations():
    """The seven valuations with at least one door open, as dicts door -> open."""
    out = []
    for bits in itertools.product((True, False), repeat=3):
        if any(bits):
            out.append(dict(zip(DOORS, bits)))
    return out


def tag(v):
    """A valuation as WME, with an open door upper case: wMe has only the middle open."""
    return ''.join(d.upper() if v[d] else d for d in DOORS)


def worlds(act):
    """(valuation, saw) for every world of the act's initial model."""
    if act == 3:
        return [(v, saw) for saw in (True, False) for v in valuations()]
    return [(v, None) for v in valuations()]


def related(act, robot, a, b):
    """Whether robot cannot tell world a from world b."""
    (va, sa), (vb, sb) = a, b
    if robot == 'k':
        # The cleaner knows whether it saw the east door, and if it saw it,
        # whether it is open.
        return sa == sb and (not sa or va['e'] == vb['e'])
    return True


def holds(act, news, world):
    v, saw = world
    if news == 1:
        return not v['e']
    if news == 2:
        return v['m'] == v['e']
    if news == 3:
        return bool(saw)
    raise ValueError(news)


def problem(act, restricted=False, name=None):
    ws = [w for w in worlds(act) if not restricted or holds(act, act, w)]
    names = [f'w{i}' for i in range(len(ws))]
    robots = ROBOTS[act]
    labels = []
    for n, (v, saw) in zip(names, ws):
        atoms = [f'(open-{d})' for d in DOORS if v[d]]
        atoms += [f'({ROLE[r]} {r})' for r in robots]
        atoms += [f'(at-{START[r]} {r})' for r in robots if r in START]
        labels.append(f'      {n} (:and {" ".join(atoms)})')
    rel = []
    for r in robots:
        pairs = [f'({names[i]} {names[j]})' for i in range(len(ws)) for j in range(len(ws))
                 if related(act, r, ws[i], ws[j])]
        rel.append(f'      {r} (:and {" ".join(pairs)})')
    comment = [f';;   {n}: {tag(v)}' + ('' if saw is None else (' cleaner saw the east door' if saw else ''))
               for n, (v, saw) in zip(names, ws)]
    title = name or (f'act{act}-news' if restricted else f'act{act}')
    head = (f';; REPLANNING ON NEWS, act {act}: {", ".join(robots)}. Written by tools/model.py.\n'
            + (f';; Restricted by the news: {NEWS_TEXT[act]}\n' if restricted else
               ';; Before the news: at least one door open, nobody knows which.\n')
            + ';; Upper case is an open door.\n' + '\n'.join(comment) + '\n')
    return (f"""{head}
(define (problem {title})
  (:domain replanning-news)
  (:requirements :general-frames :multi-pointed-models :modal-goals :negative-preconditions)
  (:agents {' '.join(robots)})
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
  (:goal (at-r c))
)
""")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for act in (1, 2, 3):
        open(os.path.join(a.out, f'act{act}.epddl'), 'w').write(problem(act))
        open(os.path.join(a.out, f'act{act}-news.epddl'), 'w').write(problem(act, restricted=True))


if __name__ == '__main__':
    main()
