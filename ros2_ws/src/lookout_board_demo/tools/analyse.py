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
From the batch's run directories, one row per run and the paired comparison.

    analyse.py BATCH [--csv BATCH/runs.csv] [--json BATCH/analysis.json]

A run's row: its seed and world, the formulation, the outcome, the simulated
time from the order to the end, the distance the robot drove (from its pose,
the simulator's odometry), the lookouts it stood at, whether it read the
board, what the map says of the zone at the end, and whether that is backed:
the map's class of the zone is the true one. For the epistemic runs it also
records the policy's own report, the observed outcome of its look, and checks
that it agrees with the map.

The comparison is paired by seed, since a seed fixes the world and the start
for all three formulations: per world, medians; overall, the sign test and the
Hodges-Lehmann estimate of the difference in time and in distance, with the
worst case of each formulation.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import statistics as st

FORMS = ('epistemic', 'replan', 'conformant')


def distance(path):
    d, last = 0.0, None
    with open(path) as fh:
        for r in csv.DictReader(fh):
            p = (float(r['x']), float(r['y']))
            if last is not None:
                d += math.dist(p, last)
            last = p
    return d


def run_row(run):
    m = json.load(open(os.path.join(run, 'mission.json')))
    w = json.load(open(os.path.join(run, 'world.json')))
    actions = list(csv.DictReader(open(os.path.join(run, 'actions.csv'))))
    stood = [r['arguments'].split()[-1] for r in actions
             if r['action'] == 'look_zone' and r['status'] == 'EXECUTING']
    read = any(r['action'] == 'read_slot' and r['status'] == 'SUCCEEDED' for r in actions)
    truth = 'blocked' if w['blocked'] else 'clear'
    row = {'seed': w['seed'], 'world': w['world'], 'clear_lookout': w['clear'],
           'zone': truth, 'formulation': m['formulation'], 'result': m.get('result'),
           'success': bool(m.get('success')), 't_mission': round(m.get('t_mission_sim', 0.0), 2),
           'distance': round(distance(os.path.join(run, 'pose.csv')), 2),
           'lookouts': ' '.join(stood), 'n_lookouts': len(stood), 'read_board': read,
           'zone_on_map': m.get('zone_on_map'),
           'backed': m.get('zone_on_map') == truth,
           'plan_seconds': round(sum(p.get('seconds', 0.0) for p in m.get('plans', [])), 3),
           'plans': len(m.get('plans', [])), 'load': open(os.path.join(run, 'load')).read().split()[0]}
    if m['formulation'] == 'epistemic':
        log = open(os.path.join(run, 'launch.log'), errors='replace').read()
        looks = re.findall(r'observed (e-look\d-(?:free|blocked))', log)
        row['policy_saw'] = ('blocked' if looks[-1].endswith('blocked') else 'clear') if looks else ''
        row['goal_held'] = '[goal] (Kw r1 blocked-z) holds' in log
        row['agrees_with_map'] = row['policy_saw'] == row['zone_on_map']
    return row


def hodges_lehmann(d):
    walsh = [(d[i] + d[j]) / 2 for i in range(len(d)) for j in range(i, len(d))]
    return st.median(walsh)


def sign_test(d):
    """Two-sided exact sign test on the non-zero differences."""
    pos, neg = sum(1 for x in d if x > 0), sum(1 for x in d if x < 0)
    n, k = pos + neg, min(pos, neg)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n * 2
    return pos, neg, min(1.0, p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('batch')
    ap.add_argument('--csv')
    ap.add_argument('--json')
    a = ap.parse_args()
    rows = [run_row(r) for r in sorted(glob.glob(os.path.join(a.batch, 'runs', '*')))
            if not r.endswith('.first-attempt') and os.path.exists(os.path.join(r, 'mission.json'))
            and 'result' in json.load(open(os.path.join(r, 'mission.json')))]
    path = a.csv or os.path.join(a.batch, 'runs.csv')
    keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k)
                  if k in rows[0] else 99)
    with open(path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r['seed'], FORMS.index(r['formulation']))))

    by = {(r['formulation'], r['seed']): r for r in rows}
    seeds = sorted({r['seed'] for r in rows if all((f, r['seed']) in by for f in FORMS)})
    out = {'seeds': len(seeds), 'per_formulation': {}, 'per_world': {}, 'paired': {}}
    for f in FORMS:
        rs = [by[(f, s)] for s in seeds]
        out['per_formulation'][f] = {
            'success': sum(r['success'] for r in rs), 'backed': sum(r['backed'] for r in rs),
            'read_board': sum(r['read_board'] for r in rs),
            'median_time': st.median(r['t_mission'] for r in rs),
            'mean_time': round(st.mean(r['t_mission'] for r in rs), 1),
            'worst_time': max(r['t_mission'] for r in rs),
            'median_distance': st.median(r['distance'] for r in rs),
            'mean_distance': round(st.mean(r['distance'] for r in rs), 2),
            'worst_distance': max(r['distance'] for r in rs),
            'mean_lookouts': round(st.mean(r['n_lookouts'] for r in rs), 2),
            'max_lookouts': max(r['n_lookouts'] for r in rs),
            'plan_seconds_median': st.median(r['plan_seconds'] for r in rs)}
        if f == 'epistemic':
            out['per_formulation'][f]['goal_held'] = sum(r.get('goal_held', False) for r in rs)
            out['per_formulation'][f]['agrees_with_map'] = sum(r.get('agrees_with_map', False)
                                                               for r in rs)
    for wd in sorted({by[('epistemic', s)]['world'] for s in seeds}):
        ss = [s for s in seeds if by[('epistemic', s)]['world'] == wd]
        r0 = by[('epistemic', ss[0])]
        out['per_world'][wd] = {'clear_lookout': r0['clear_lookout'], 'zone': r0['zone'], 'runs': len(ss)}
        for f in FORMS:
            rs = [by[(f, s)] for s in ss]
            out['per_world'][wd][f] = {'backed': sum(r['backed'] for r in rs),
                                       'time': st.median(r['t_mission'] for r in rs),
                                       'distance': st.median(r['distance'] for r in rs),
                                       'lookouts': rs[0]['lookouts']}
    for f in ('replan', 'conformant'):
        for q in ('t_mission', 'distance'):
            d = [by[(f, s)][q] - by[('epistemic', s)][q] for s in seeds]
            pos, neg, p = sign_test(d)
            out['paired'][f'{f}_minus_epistemic_{q}'] = {
                'hodges_lehmann': round(hodges_lehmann(d), 2), 'median': round(st.median(d), 2),
                'epistemic_better': pos, 'epistemic_worse': neg, 'sign_test_p': p,
                'min': round(min(d), 2), 'max': round(max(d), 2)}
    json.dump(out, open(a.json or os.path.join(a.batch, 'analysis.json'), 'w'), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
