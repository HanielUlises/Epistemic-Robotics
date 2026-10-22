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
The latent-tool-fault variant on paper, before any run in the simulator.

    precheck.py OUT [--json OUT/precheck.json]

Plans belief-empathy-latent.epddl for a site in each half with plank and
Aletheia, then, for each of the 24 seeds, follows the policy and each of the
paper's procedures (published, strong, ideal) in the seed's pattern with the
kinematic model of tools/estimate.py, from the moment the aerial vehicle finds
the task. Prints, per seed, the steps each takes and the predicted time, and
whether the policy and the strong procedure decide differently.
"""

import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import estimate as E  # noqa: E402
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

EPDDL = os.environ.get('EPDDL', os.path.realpath(os.path.join(HERE, '..', '..', '..', '..', 'epddl-workspace',
                                                               'belief-empathy')))
PLANK = os.environ.get('PLANK', os.path.expanduser('~/plank/build/plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.expanduser('~/aletheia-subjective/build/epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl'))


def found(site):
    route, s = L.ROUTE['u'], 0.0
    while s <= L.length(route):
        p = L.at(route, s)[0]
        if math.dist(p, site) <= L.R_DETECT:
            return p, s / L.V_UAV
        s += 0.02
    raise ValueError(site)


def policy(out, side):
    d = os.path.join(out, side)
    os.makedirs(d, exist_ok=True)
    subprocess.run([PLANK, 'export', '-d', os.path.join(EPDDL, 'belief-empathy-latent.epddl'), '-p',
                    os.path.join(EPDDL, 'instances', f'problem_latent_{side}.epddl'), '-l', LIB,
                    os.path.join(EPDDL, 'hailing.epddl'), '-o', d], check=True, capture_output=True)
    task = os.path.join(d, f'problem_latent_{side}.json')
    pol = os.path.join(d, 'policy.json')
    if not os.path.exists(pol):
        subprocess.run([PLANNER, '--task', task, '--plan', pol, '--conditional', '--consistent-beliefs'],
                       check=True, capture_output=True)
    return json.load(open(task)), json.load(open(pol))


def short(steps):
    out = []
    for s in steps:
        if s.startswith('hail-p'):
            a, ev = s.split(': ')
            out.append(f"{a.split('_')[-1]}@{a[6]}{'+' if 'met' in ev else '-'}")
        elif s.startswith('go-site'):
            out.append(f"send {s.split('_')[1]}")
        elif s.startswith('perform'):
            out.append(f"attempt {s.split('_')[2].split(':')[0]}: {'done' if 'done' in s else 'failed'}")
        elif s.startswith(('fly-', 'back-', 'cross-')):
            continue
        else:
            out.append(s)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--json')
    a = ap.parse_args()
    pol = {side: policy(a.out, side) for side in ('east', 'west')}
    rows = []
    for seed in range(1, 25):
        sc = W.scenario_latent(seed)
        p, site = tuple(sc['pattern']), tuple(sc['site'])
        u, t = found(site)
        task, tree = pol[sc['side']]
        te, se = E.run_policy(task, tree, p, site, u, t)
        row = {'seed': seed, 'pattern': p, 'site': site, 'found': round(t, 1),
               'epistemic': (round(te, 1), short(se))}
        # the counterfactuals, cumulatively: attempts in order of arrival, then
        # no vehicle with two drive faults sent
        ta, _ = E.run_policy(task, tree, p, site, u, t, arrival_order=True)
        tb, _ = E.run_policy(task, tree, p, site, u, t, arrival_order=True, spare_slow=True)
        row['order'], row['spare'] = round(ta, 1), round(tb, 1)
        for strength in ('published', 'strong', 'ideal'):
            tp, sp = E.run_procedure(strength, p, site, u, t)
            row[strength] = (round(tp, 1), sp)
        rows.append(row)
    print(f"{'seed':>4} {'pattern':<13}{'site':<11}{'policy':>8}{'publ.':>8}{'strong':>8}{'ideal':>8}"
          f"   policy steps  |  strong steps")
    for r in rows:
        print(f"{r['seed']:>4} {str(r['pattern']):<13}{str(r['site']):<11}{r['epistemic'][0]:>8}"
              f"{r['published'][0]:>8}{r['strong'][0]:>8}{r['ideal'][0]:>8}   "
              f"{' '.join(r['epistemic'][1])}  |  {' '.join(r['strong'][1])}")
    for k in ('epistemic', 'published', 'strong', 'ideal'):
        ts = [r[k][0] for r in rows]
        print(f'{k:>10}: mean {sum(ts) / len(ts):.1f} s, worst {max(ts):.1f} s')
    diff = [r for r in rows if abs(r['epistemic'][0] - r['strong'][0]) > 1.0]
    print(f'policy and strong procedure differ by more than 1 s in {len(diff)} of {len(rows)} seeds; '
          f'policy sooner in {sum(r["epistemic"][0] < r["strong"][0] - 1 for r in rows)}')
    gaps = [(r['epistemic'][0] - r['order'], r['order'] - r['spare'], r['spare'] - r['strong'][0]) for r in rows]
    total = sum(r['epistemic'][0] - r['strong'][0] for r in rows) / len(rows)
    parts = [sum(g[k] for g in gaps) / len(rows) for k in range(3)]
    print(f'mean gap, policy minus strong procedure: {total:.1f} s = attempt order {parts[0]:.1f} s + '
          f'a slow vehicle sent {parts[1]:.1f} s + the rest {parts[2]:.1f} s')
    for r, g in zip(rows, gaps):
        print(f"  seed {r['seed']:>2}: gap {r['epistemic'][0] - r['strong'][0]:+7.1f} = order {g[0]:+6.1f} "
              f"slow {g[1]:+6.1f} rest {g[2]:+6.1f}")
    if a.json:
        json.dump(rows, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
