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
The reference semantics: an epistemic model with a ranking per robot.

A state is a set of worlds, for each robot a partition of them (what it
cannot tell apart: every relation an equivalence), for each robot a rank of
every world, and the designated worlds, those the planner cannot rule out.
Robot r believes phi at w when phi holds at the least-ranked worlds of r's
cell at w; it knows phi when phi holds throughout the cell. Formulas are
Python functions of a world, so they nest: B(c, w, lambda v: not B(d, v, phi))
is "c believes that d does not believe phi".

Three updates are all the actions need, and each is the product update of an
event model the EPDDL domain writes (tools/model.py), as tools/policy_check.py
checks at every node of every policy:

    restrict    a public event with a precondition: the worlds where it fails go
    refine      a semi-private look: the looker's cells split by what it saw;
                nobody else's change
    shift       a belief report taken on trust: the listener's rank of every
                world where the report is false rises by the trust, capped at
                the highest grade
"""

import ranks as R


class State:

    def __init__(self, worlds, cells, ranks, designated):
        self.worlds = tuple(worlds)                     # world indices into R.worlds()
        self.cells = cells                              # robot -> {world: frozenset}
        self.ranks = ranks                              # robot -> {world: int}
        self.designated = frozenset(designated)
        self.capped = False                             # whether a shift hit the highest grade

    @staticmethod
    def initial(fleet):
        ws = list(range(len(R.worlds())))
        everything = frozenset(ws)
        cells = {r.name: {w: everything for w in ws} for r in fleet}
        ranks = {r.name: {w: R.rank(r.ranking, R.worlds()[w]) for w in ws} for r in fleet}
        return State(ws, cells, ranks, ws)

    def key(self):
        return (self.worlds, tuple(sorted((r, tuple(sorted((w, tuple(sorted(c))) for w, c in cs.items())))
                                          for r, cs in self.cells.items())),
                tuple(sorted((r, tuple(sorted(rk.items()))) for r, rk in self.ranks.items())),
                self.designated)

    def copy(self):
        s = State(self.worlds, {r: dict(c) for r, c in self.cells.items()},
                  {r: dict(k) for r, k in self.ranks.items()}, self.designated)
        s.capped = self.capped
        return s

    # -- the modalities ------------------------------------------------------------------

    def cell(self, r, w):
        return self.cells[r][w]

    def least(self, r, w):
        c = self.cells[r][w]
        m = min(self.ranks[r][v] for v in c)
        return [v for v in c if self.ranks[r][v] == m]

    def K(self, r, w, f):
        return all(f(v) for v in self.cells[r][w])

    def B(self, r, w, f):
        return all(f(v) for v in self.least(r, w))

    def E(self, r, w, f):
        """r entertains f at w: it does not believe not-f."""
        return any(f(v) for v in self.least(r, w))

    def everywhere(self, f):
        """f at every designated world: what the planner may count on."""
        return all(f(w) for w in self.designated)

    # -- the updates ---------------------------------------------------------------------

    def restrict(self, pre):
        keep = [w for w in self.worlds if pre(w)]
        ks = set(keep)
        cells = {r: {w: frozenset(v for v in c[w] if v in ks) for w in keep} for r, c in self.cells.items()}
        ranks = {r: {w: k[w] for w in keep} for r, k in self.ranks.items()}
        s = State(keep, cells, ranks, [w for w in self.designated if w in ks])
        s.capped = self.capped
        return s

    def refine(self, r, f):
        s = self.copy()
        s.cells[r] = {w: frozenset(v for v in c if f(v) == f(w)) for w, c in self.cells[r].items()}
        return s

    def shift(self, listener, f, alpha):
        s = self.copy()
        for w in self.worlds:
            if not f(w):
                k = s.ranks[listener][w] + alpha
                if k > R.GRADES - 1:
                    s.capped = True
                    k = R.GRADES - 1
                s.ranks[listener][w] = k
        return s

    def projected(self, onto):
        """The same state with every other robot ranking as `onto` does: what a robot
        that takes the others to revise as it does supposes they believe."""
        s = self.copy()
        for r in s.ranks:
            if r != onto:
                s.ranks[r] = dict(s.ranks[onto])
        return s

    def believed_worlds(self, r):
        """r's least-ranked worlds at the designated worlds (the same at all of them while
        every branch of the policy splits on r's own looks)."""
        w = next(iter(self.designated))
        return self.least(r, w)
