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
The worlds of the collapsed wing, the formulations, and the timings they share.

A world says, for each way into the north wing (w, m, e), whether it is clear.
At least one is: seven worlds, which the seeds take in turn, seed 1 the first.

epistemic    Aletheia's policy over collapsed-wing.epddl: the scouts look, say what
             they found, and the medic crosses only where it knows the way is clear;
optimistic   POPF with every way assumed clear: the medic takes the shortest, and
             finds rubble, if any, with its own laser; the mission then marks that
             way blocked and replans;
gated        POPF with the medic's crossing gated on a fact (checked-clear ?d)
             that only a scout's check can give: the epistemic precondition
             written as a fact, with the check optimistic and replanning when it
             reports rubble.
"""

WAYS = ('w', 'm', 'e')
WORLDS = [dict(zip(WAYS, (bool(k & 4), bool(k & 2), bool(k & 1)))) for k in range(7, 0, -1)]
FORMULATIONS = ('epistemic', 'optimistic', 'gated')
START_AT = 25.0


def world(seed):
    return WORLDS[(seed - 1) % len(WORLDS)]
