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
The worlds of the belief and empathy demonstration, the seeds, the
formulations, and the timings they share.

A world says which particle each ground vehicle tracks, (p_g1, p_g2): 1 with no
fault, 2 with one, 3 with two. At most two faults happen in the team, so there
are six worlds, grouped as in the paper's Table I by the number of faults: none,
(1, 1); one, (2, 1) and (1, 2); two, (3, 1), (1, 3) and (2, 2).

A seed fixes the world, the site of the task and the times of the faults:
seeds 1 to 6 take the six worlds in turn with the first site, 7 to 12 with the
second, and so on, so that 24 seeds hold every world at every site once. Each
fault happens before the aerial vehicle can have found the task: a first fault
between 20 and 45 s after the start, a second between 5 s after the first and
50 s. The policy is computed when the task is found, and a fault after that
moment is outside its model.

epistemic   ePlanSys: Aletheia's policy over belief-empathy.epddl, computed
            when the aerial vehicle finds the task, executed by the epistemic
            executor; radio range R.
particles   Bramblett and Bezzo's procedure: the aerial vehicle assigns the task
            by least estimated makespan over its common-belief particles, seeks
            the chosen vehicle at its particles in order of likelihood, and
            reassigns by makespan on what it learns at each meeting; radio
            range R.
ideal       the paper's ideal method: every robot hears every other wherever it
            is, so the task and every fault are known to all at once, and the
            task goes to the vehicle of least makespan.
flock       the paper's flock method: the team keeps in contact throughout,
            sweeping the whole area together at the speed of its slowest
            member; a vehicle with two faults leaves the flock for the base.
"""

import random

import estimate as E
import layout as L

WORLDS = [(1, 1), (2, 1), (1, 2), (3, 1), (1, 3), (2, 2)]
FORMULATIONS = ('epistemic', 'particles', 'ideal', 'flock')
START_AT = 20.0
SEEDS = 24


def faults(world):
    return sum(p - 1 for p in world)


def scenario(seed):
    """The world, the site and the fault times of a seed."""
    world = WORLDS[(seed - 1) % len(WORLDS)]
    site = L.SITES[((seed - 1) // len(WORLDS)) % len(L.SITES)]
    rng = random.Random(1000 + seed)
    times = {}
    for g, p in zip(L.UGVS, world):
        if p >= 2:
            t1 = round(rng.uniform(20.0, 45.0), 1)
            times[g] = [t1]
            if p == 3:
                times[g].append(round(rng.uniform(t1 + 5.0, 50.0), 1))
    return {'seed': seed, 'world': dict(zip(L.UGVS, world)), 'site': site, 'side': L.side(site),
            'fault_times': times}


# ─── the variant with latent tool faults (belief-empathy-latent.epddl) ──────
#
# A pattern is (d1, t1, d2, t2), estimate.PATTERNS: the twelve with at most two
# faults of either kind and at least one working tool. Seed s takes pattern
# (s - 1) mod 12, and over the 24 seeds each pattern meets one site in each
# half. Drive faults happen at times drawn as above; a tool fault is latent and
# has no time.

LATENT_FORMULATIONS = ('epistemic', 'timed', 'published', 'strong', 'ideal')


def scenario_latent(seed):
    k, r = (seed - 1) % len(E.PATTERNS), (seed - 1) // len(E.PATTERNS)
    d1, t1, d2, t2 = E.PATTERNS[k]
    site = L.SITES[(k + 2 * r) % len(L.SITES)]
    rng = random.Random(2000 + seed)
    times = {}
    for g, d in (('g1', d1), ('g2', d2)):
        if d >= 1:
            first = round(rng.uniform(20.0, 45.0), 1)
            times[g] = [first]
            if d == 2:
                times[g].append(round(rng.uniform(first + 5.0, 50.0), 1))
    return {'seed': seed, 'variant': 'latent', 'pattern': [d1, t1, d2, t2],
            'world': {'g1': d1 + 1, 'g2': d2 + 1}, 'tool_fault': {'g1': bool(t1), 'g2': bool(t2)},
            'site': site, 'side': L.side(site), 'fault_times': times}
