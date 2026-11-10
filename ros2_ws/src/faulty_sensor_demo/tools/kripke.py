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
The reference semantics: an epistemic model with one ranking of its worlds.

A state is a set of worlds of a model of error (tools/ranks.py), for each
robot a partition of them (what it cannot tell apart: every relation an
equivalence), the rank of every world, the same for every robot, and the
designated worlds, those the planner cannot rule out. With m the least rank
in robot r's cell at w,

    K_r phi      phi holds throughout the cell
    B_r phi      phi holds at the cell's worlds of rank m
    B^k_r phi    phi holds at the cell's worlds of rank below m + k: r believes
                 phi firmly, to degree k. B^1 is B, and B^k for every k is K.

The firmness of a belief is the largest such k: how many ranks of surprise it
would take for phi to be false. Two updates are all the actions need, and
each is the product update of an event model the EPDDL domain writes
(tools/model.py), as tools/policy_check.py checks at every node of every
policy:

    refine      a reading: the reader's cells split by what it read; nobody
                else's change
    restrict    something said on the radio, or an action every robot sees:
                the worlds where its precondition fails go
"""

import math


class State:

    def __init__(self, model, worlds, cells, designated):
        self.model = model                              # ranks.Model
        self.worlds = tuple(worlds)                     # indices into model.worlds
        self.cells = cells                              # robot -> {world: frozenset}
        self.designated = frozenset(designated)

    @staticmethod
    def initial(model, robots):
        ws = list(range(model.n))
        everything = frozenset(ws)
        return State(model, ws, {r: {w: everything for w in ws} for r in robots}, ws)

    def copy(self):
        return State(self.model, self.worlds, {r: dict(c) for r, c in self.cells.items()}, self.designated)

    def rank(self, w):
        return self.model.rank[w]

    # -- the modalities ------------------------------------------------------------------

    def cell(self, r, w):
        return self.cells[r][w]

    def within(self, r, w, k):
        """The worlds of r's cell at w whose rank is below the least plus k."""
        c = self.cells[r][w]
        m = min(self.rank(v) for v in c)
        return [v for v in c if self.rank(v) < m + k]

    def K(self, r, w, f):
        return all(f(v) for v in self.cells[r][w])

    def B(self, r, w, f, k=1):
        return all(f(v) for v in self.within(r, w, k))

    def firmness(self, r, w, f):
        c = self.cells[r][w]
        against = [self.rank(v) for v in c if not f(v)]
        if not against:
            return math.inf
        return min(against) - min(self.rank(v) for v in c)

    # -- the updates ---------------------------------------------------------------------

    def restrict(self, pre):
        keep = [w for w in self.worlds if pre(w)]
        ks = set(keep)
        cells = {r: {w: frozenset(v for v in c[w] if v in ks) for w in keep} for r, c in self.cells.items()}
        return State(self.model, keep, cells, [w for w in self.designated if w in ks])

    def refine(self, r, f):
        s = self.copy()
        s.cells[r] = {w: frozenset(v for v in c if f(v) == f(w)) for w, c in self.cells[r].items()}
        return s

    def select(self, f):
        """The same state with only the designated worlds where f holds: a branch of the policy."""
        s = self.copy()
        s.designated = frozenset(w for w in self.designated if f(w))
        return s
