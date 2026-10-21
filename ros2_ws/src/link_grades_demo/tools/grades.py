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
The grades of the link, the formulations, and the timings both share.

The link between r1 and r2 is cut by epistemic_comm's comm_monitor on an
open-loop schedule, in seconds from its first clock tick, which is the
simulator's first: no instant depends on where the robots are.

full     the link never falls;
partial  it falls CUT_AFTER s into the mission and returns outage(seed) s
         later: 24 s for seed 1, up to 100 s for seed 20;
none     it is down before the mission starts and never returns.

Every mission starts at START_AT simulated seconds. A message that asks for an
acknowledgement -- the epistemic share, the classical report with replanning
-- waits ACK_WAIT s for it, in one node, the same code path for both.
"""

START_AT = 25.0
CUT_AFTER = 10.0
ACK_WAIT = 30.0
NEVER = 1.0e6
GRADES = ('full', 'partial', 'none')
FORMULATIONS = ('epistemic', 'epistemic_open', 'visit', 'report', 'replan')


def outage(seed):
    return 20.0 + 4.0 * seed


def schedule(grade, seed):
    """(t_disc, t_recon) for comm_monitor."""
    if grade == 'full':
        return NEVER, 2 * NEVER
    if grade == 'partial':
        return START_AT + CUT_AFTER, START_AT + CUT_AFTER + outage(seed)
    return 0.5, NEVER
