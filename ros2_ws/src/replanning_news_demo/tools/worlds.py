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
The runs of replanning on news: which doors are open, who is in the team, and
when the news arrives.

A run's doors are a valuation in which its act's news is true, since a public
announcement is truthful: seed 1 the first of them, seed 2 the second, and so
round. In the third act the cleaner has always seen the east door: its message
is true in every run.

The formulations:

  replan     the news is announced into the executor's epistemic state, the
             policy is interrupted, and the mission plans again from the
             model the announcement left;
  null       at the same instant a tautology is announced: the policy is
             interrupted and planned again from a model the announcement did
             not change, which separates what the news is worth from what
             planning again is worth;
  keep       the news is heard and not announced: the policy planned at the
             start runs to its end;
  classical  POPF over the classical model, planning in the most convenient
             valuation consistent with everything known, the news included,
             and planning again when the news or a door found shut rules that
             valuation out.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model as M  # noqa: E402

FORMULATIONS = ('replan', 'null', 'keep', 'classical')
ROBOTS = M.ROBOTS
START_AT = 6.0          # simulated seconds: every mission starts at this instant
NEWS_AT = 14.0          # seconds into the mission: the news arrives, mid-drive
TAUTOLOGY = '(or open-w (not open-w))'
tag = M.tag


def worlds(act):
    """The valuations in which the act's news is true."""
    return [v for v in M.valuations() if M.holds(act, act, (v, True))]


def world(act, seed):
    ws = worlds(act)
    return ws[(seed - 1) % len(ws)]
