#!/usr/bin/env python3
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
Every formulation against the least expected time, in the kinematic model.

    timed_bound.py --latent-pre PRE --direct DIRECT --replica-batch BATCH --out BOUND.json

The timed search's value at the root is the least expected time to the task
of any policy built from its moves over the aerial vehicle's knowledge: seek a
vehicle at a particle, send a vehicle met, try a vehicle sent. In the domain
with direct movement (variants/belief-empathy-latent-direct.epddl) those moves
include every decision the strong procedure makes, so the value bounds what
any procedure over the same knowledge can reach on average. Every formulation
is timed here by the same model (estimate.run_policy for a policy,
estimate.run_procedure for a procedure) in every fault pattern at every site,
each pattern counted once, which is the distribution the search optimises.

PRE is precheck.py's output (the latent tasks and Aletheia's policies, per
side); DIRECT holds the direct-movement tasks and their skeletons; BATCH is the
replica's batch, where Aletheia's replica policies are read.
"""

import argparse
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import estimate as E  # noqa: E402
import layout as L  # noqa: E402
import precheck  # noqa: E402
import timed_policy as TP  # noqa: E402
import worlds as W  # noqa: E402

SITES = {'east': [(4.0, 2.0), (4.0, 6.0)], 'west': [(-4.0, 2.0), (-4.0, 6.0)]}
SKELETONS = os.path.join(HERE, '..', 'policies', 'skeletons')
HAIL_EST = 2.0          # the paper's procedure's allowance for a hail answered, as in mission.py


def timed(task, skeletons, site):
    sk = TP.Skeleton.load(task, os.path.join(skeletons, TP.task_hash(task)[:16] + '.skel'))
    u, t = precheck.found(site)
    return TP.plan(sk, site, u, t)


def summary(ts):
    return {'mean': statistics.mean(ts), 'worst': max(ts), 'n': len(ts)}


# ─── the replica's procedures, as mission.py runs them ──────────────────────

def run_particles(world, site, u, t):
    """mission.py's `particles`: the vehicle of least estimated makespan,
    sought at its first particle not yet found empty, as if it were there; met
    and capable, sent if its makespan beats seeking the other."""
    excluded = {g: set() for g in L.UGVS}
    known = {}

    def est(g, b):
        q, when = L.intercept(u, t, g, b)
        return (when - t) + HAIL_EST + max(E.fly(q, site), E.drive_to_site(q, site, b - 1))

    def seeking(skip=None):
        out = []
        for g in L.UGVS:
            if g == skip:
                continue
            if g in known:
                if known[g] < 2:
                    out.append((est(g, known[g] + 1), ('seek', g, known[g] + 1)))
                continue
            left = [b for b in (1, 2, 3) if b not in excluded[g]]
            if left:
                out.append((est(g, left[0]), ('seek', g, left[0])))
        return out

    for _ in range(20):
        _, (_, g, b) = min(seeking(), key=lambda o: o[0])
        u, t = L.intercept(u, t, g, b)
        met = world[g] == b
        t += E.HAIL_MET if met else L.HAIL_WINDOW
        if not met:
            excluded[g].add(b)
            continue
        known[g] = b - 1
        here = L.particle(g, b, t)
        options = [(max(E.fly(u, site), E.drive_to_site(here, site, b - 1)), ('send',))] if b < 3 else []
        options += seeking(skip=g)
        if min(options, key=lambda o: o[0])[1][0] == 'send':
            return max(t + E.fly(u, site), t + E.drive_to_site(here, site, b - 1)) + L.DELTA
    return None


def run_ideal(world, site, u, t):
    return min(max(t + E.fly(u, site), t + E.drive_to_site(L.particle(g, world[g], t), site, world[g] - 1))
               + L.DELTA for g in L.UGVS if world[g] < 3)


def latent(pre, direct, skeletons):
    rows = {k: [] for k in ('aletheia', 'timed', 'optimum', 'published', 'strong', 'ideal', 'optimum_worst')}
    sites = {}
    for side, ss in SITES.items():
        task = json.load(open(os.path.join(pre, side, f'problem_latent_{side}.json')))
        aletheia = json.load(open(os.path.join(pre, side, 'policy.json')))
        dtask = json.load(open(os.path.join(direct, side, f'problem_latent_{side}.json')))
        for site in ss:
            u, t = precheck.found(site)
            mine = timed(task, skeletons, site)
            best = timed(dtask, os.path.join(direct, 'skeletons'), site)
            dsk = TP.Skeleton.load(dtask, os.path.join(direct, 'skeletons', TP.task_hash(dtask)[:16] + '.skel'))
            robust = TP.plan(dsk, site, u, t, 'worst')
            here = {k: [] for k in rows}
            for p in E.PATTERNS:
                here['aletheia'].append(E.run_policy(task, aletheia, p, site, u, t)[0])
                here['timed'].append(E.run_policy(task, mine['policy'], p, site, u, t)[0])
                here['optimum'].append(E.run_policy(dtask, best['policy'], p, site, u, t)[0])
                here['optimum_worst'].append(E.run_policy(dtask, robust['policy'], p, site, u, t)[0])
                for k in ('published', 'strong', 'ideal'):
                    here[k].append(E.run_procedure(k, p, site, u, t)[0])
            # the search's own value is the mean the replay finds
            assert abs(statistics.mean(here['optimum']) - best['expected']) < 0.05, site
            assert abs(statistics.mean(here['timed']) - mine['expected']) < 0.05, site
            # no procedure over the same knowledge does better on average
            assert statistics.mean(here['strong']) >= best['expected'] - 0.05, site
            assert statistics.mean(here['published']) >= best['expected'] - 0.05, site
            sites[f'{site[0]:+.0f},{site[1]:.0f}'] = {k: summary(v) for k, v in here.items()}
            for k in rows:
                rows[k] += here[k]
    return {'cases': len(rows['strong']), 'rows': {k: summary(v) for k, v in rows.items()}, 'sites': sites}


def replica(batch, skeletons):
    rows = {k: [] for k in ('aletheia', 'timed', 'particles', 'ideal')}
    sites = {}
    for side, ss in SITES.items():
        seed = next(s for s in range(1, W.SEEDS + 1) if W.scenario(s)['side'] == side)
        aletheia = json.load(open(os.path.join(batch, f'epistemic-s{seed}', 'policy.json')))
        task = json.load(open(os.path.join(batch, f'epistemic-s{seed}', 'task', f'problem_{side}.json')))
        for site in ss:
            u, t = precheck.found(site)
            mine = timed(task, skeletons, site)
            here = {k: [] for k in rows}
            for p1, p2 in W.WORLDS:
                world = {'g1': p1, 'g2': p2}
                p = (p1 - 1, 0, p2 - 1, 0)
                here['aletheia'].append(E.run_policy(task, aletheia, p, site, u, t)[0])
                here['timed'].append(E.run_policy(task, mine['policy'], p, site, u, t)[0])
                here['particles'].append(run_particles(world, site, u, t))
                here['ideal'].append(run_ideal(world, site, u, t))
            assert abs(statistics.mean(here['timed']) - mine['expected']) < 0.05, site
            assert statistics.mean(here['particles']) >= mine['expected'] - 0.05, site
            sites[f'{site[0]:+.0f},{site[1]:.0f}'] = {k: summary(v) for k, v in here.items()}
            for k in rows:
                rows[k] += here[k]
    return {'cases': len(rows['timed']), 'rows': {k: summary(v) for k, v in rows.items()}, 'sites': sites}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--latent-pre', required=True)
    ap.add_argument('--direct', required=True)
    ap.add_argument('--replica-batch', required=True)
    ap.add_argument('--skeletons', default=SKELETONS)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = {'latent': latent(a.latent_pre, a.direct, a.skeletons), 'replica': replica(a.replica_batch, a.skeletons)}
    json.dump(out, open(a.out, 'w'), indent=1)
    for part in ('replica', 'latent'):
        print(f"{part}, every pattern at every site ({out[part]['cases']}):")
        for k, v in out[part]['rows'].items():
            print(f"  {k:>14}: mean {v['mean']:6.1f} s   worst {v['worst']:6.1f} s")


if __name__ == '__main__':
    main()
