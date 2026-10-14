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
The batch of the gate signal, in numbers.

    analyse.py BATCH [--csv runs.csv] [--json analysis.json]

One row per run: the order, whether both deliveries were made and into which
bays, each robot's delivery time and distance driven, the team's makespan
(the later of the two deliveries, in simulated seconds from the common start),
and for the epistemic policy the moment r2 learned the order at the gate. A
run is correct when both robots delivered and into different bays, r1 into the
bay of its order.

Paired by seed, the epistemic policy against the cautious classical team, on
the makespan of the runs both completed correctly: the exact sign test and the
Hodges-Lehmann estimate of the difference, by order, since which team is
faster depends on the order and the pooled test would only measure the mix;
and the mean over the two orders, which is the expected makespan when either
order is as likely. Correctness against the optimistic team: McNemar's exact
test on the discordant seeds.
"""

import argparse
import csv
import glob
import json
import math
import os
import statistics as st

FORMS = ['epistemic', 'optimistic', 'cautious']


def distance(rows):
    d, last = 0.0, None
    for x, y in rows:
        if last is not None:
            d += math.dist(last, (x, y))
        last = (x, y)
    return d


def actions(run):
    out = []
    for r in csv.DictReader(open(os.path.join(run, 'actions.csv'))):
        if r['status'] in ('SUCCEEDED', 'FAILED'):
            out.append((float(r['t_sim']), r['stack'], r['action'], r['arguments'], r['status'], r['outcome']))
    return out


def last_end(acts, action, robot, t0):
    ts = [t for t, _, a, g, s, _ in acts if a == action and g.split()[:1] == [robot] and s == 'SUCCEEDED']
    return round(max(ts) - t0, 2) if ts else None


def run_row(run):
    form, seed = os.path.basename(run).rsplit('-s', 1)
    world = json.load(open(os.path.join(run, 'world.json')))
    row = {'formulation': form, 'seed': int(seed), 'order': world['order']}
    if form == 'epistemic':
        m = json.load(open(os.path.join(run, 'mission.json')))
        missions = {'r1': m, 'r2': m}
        results = [m.get('result')]
        bays = m.get('final_bay', {})
    else:
        missions = {r: json.load(open(os.path.join(run, f'mission_{r}.json'))) for r in ('r1', 'r2')}
        results = [missions[r].get('result') for r in ('r1', 'r2')]
        # Each robot's final bay, from its own mission's end: the other may still be driving then.
        bays = {r: missions[r].get('final_bay', {}).get(r, '') for r in ('r1', 'r2')}
    t0 = missions['r1'].get('t0_sim', 0.0)
    acts = actions(run)
    for r in ('r1', 'r2'):
        row[f'{r}_delivered'] = last_end(acts, 'unload', r, t0)
        row[f'{r}_bay'] = bays.get(r, '')
    row['result'] = '/'.join(results)
    row['completed'] = all(x == 'SUCCESS' for x in results) and None not in (row['r1_delivered'],
                                                                           row['r2_delivered'])
    other = {'x': 'y', 'y': 'x'}[world['order']]
    row['correct'] = bool(row['completed'] and row['r1_bay'] == world['order'] and row['r2_bay'] == other)
    row['same_bay'] = bool(row['completed'] and row['r1_bay'] == row['r2_bay'])
    row['makespan'] = max(row['r1_delivered'], row['r2_delivered']) if row['completed'] else None
    row['r1_in_bay'] = (last_end(acts, 'enter_bay', 'r1', t0) if form == 'epistemic'
                        else last_end(acts, 'back_route', 'r1', t0) or last_end(acts, 'enter_bay', 'r1', t0))
    row['r1_route'] = ('gate' if any(a == 'to_gate' for _, _, a, *_ in acts)
                       else 'back' if any(a == 'back_route' for _, _, a, *_ in acts) else '')
    row['r2_learned'] = last_end(acts, 'watch', 'r2', t0) if form == 'epistemic' else None
    row['r2_crossed'] = any(a == 'cross' for _, _, a, *_ in acts)
    poses = {'r1': [], 'r2': []}
    for p in csv.DictReader(open(os.path.join(run, 'pose.csv'))):
        if float(p['t_sim']) >= t0:
            poses[p['robot']].append((float(p['x']), float(p['y'])))
    for r in ('r1', 'r2'):
        row[f'{r}_distance'] = round(distance(poses[r]), 2)
    plans = [p for m in ({id(v): v for v in missions.values()}.values()) for p in m.get('plans', [])]
    row['planning_seconds'] = round(sum(p.get('seconds', 0.0) for p in plans), 3)
    return row


def hodges_lehmann(d):
    w = sorted((a + b) / 2 for i, a in enumerate(d) for b in d[i:])
    return st.median(w)


def sign_test(d):
    """Two-sided exact sign test on the non-zero differences."""
    pos, neg = sum(1 for x in d if x > 0), sum(1 for x in d if x < 0)
    n, k = pos + neg, min(pos, neg)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n * 2 if n else 1.0
    return pos, neg, min(1.0, p)


def mcnemar(b, c):
    """Exact two-sided McNemar test on b and c discordant pairs."""
    n, k = b + c, min(b, c)
    return min(1.0, sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n * 2) if n else 1.0


def summary(rs):
    done = [r for r in rs if r['completed']]
    ms = [r['makespan'] for r in done]
    out = {'runs': len(rs), 'completed': len(done), 'correct': sum(r['correct'] for r in rs),
           'same_bay': sum(r['same_bay'] for r in rs), 'crossed': sum(r['r2_crossed'] for r in rs)}
    if ms:
        out.update(median_makespan=st.median(ms), mean_makespan=round(st.mean(ms), 1),
                   worst_makespan=max(ms), best_makespan=min(ms),
                   mean_r1=round(st.mean(r['r1_delivered'] for r in done), 1),
                   mean_r2=round(st.mean(r['r2_delivered'] for r in done), 1),
                   mean_distance=round(st.mean(r['r1_distance'] + r['r2_distance'] for r in done), 2),
                   mean_r1_distance=round(st.mean(r['r1_distance'] for r in done), 2),
                   mean_r2_distance=round(st.mean(r['r2_distance'] for r in done), 2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('batch')
    ap.add_argument('--csv')
    ap.add_argument('--json')
    a = ap.parse_args()
    rows = []
    for run in sorted(glob.glob(os.path.join(a.batch, 'runs', '*'))):
        if run.endswith('.first-attempt'):
            continue
        form = os.path.basename(run).rsplit('-s', 1)[0]
        need = ['mission.json'] if form == 'epistemic' else ['mission_r1.json', 'mission_r2.json']
        if all(os.path.exists(os.path.join(run, n)) and 'result' in json.load(open(os.path.join(run, n)))
               for n in need):
            rows.append(run_row(run))
    keys = list(rows[0])
    with open(a.csv or os.path.join(a.batch, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r['seed'], FORMS.index(r['formulation']))))

    by = {(r['formulation'], r['seed']): r for r in rows}
    seeds = sorted({r['seed'] for r in rows if all((f, r['seed']) in by for f in FORMS)})
    out = {'seeds': len(seeds), 'per_formulation': {}, 'per_order': {}, 'paired': {}}
    for f in FORMS:
        out['per_formulation'][f] = summary([by[(f, s)] for s in seeds])
        for o in ('x', 'y'):
            out['per_order'].setdefault(o, {})[f] = summary([by[(f, s)] for s in seeds if by[(f, s)]['order'] == o])
    learned = [by[('epistemic', s)]['r2_learned'] for s in seeds if by[('epistemic', s)]['r2_learned'] is not None]
    if learned:
        out['r2_learned'] = {'median': st.median(learned), 'worst': max(learned)}
    cl = [by[(f, s)]['r1_in_bay'] for f in ('optimistic', 'cautious') for s in seeds
          if by[(f, s)]['r1_in_bay'] is not None]
    if cl:
        out['classical_r1_in_bay'] = {'median': st.median(cl), 'worst': max(cl)}
    ep = [by[('epistemic', s)]['r1_in_bay'] for s in seeds if by[('epistemic', s)]['r1_in_bay'] is not None]
    if ep:
        out['epistemic_r1_in_bay'] = {'median': st.median(ep), 'worst': max(ep)}

    for scope, ss in (('all', seeds), ('x', [s for s in seeds if by[('epistemic', s)]['order'] == 'x']),
                      ('y', [s for s in seeds if by[('epistemic', s)]['order'] == 'y'])):
        both = [s for s in ss if by[('epistemic', s)]['correct'] and by[('cautious', s)]['correct']]
        d = [by[('cautious', s)]['makespan'] - by[('epistemic', s)]['makespan'] for s in both]
        if d:
            pos, neg, p = sign_test(d)
            out['paired'][f'cautious_minus_epistemic_{scope}'] = {
                'pairs': len(d), 'epistemic_faster': pos, 'cautious_faster': neg, 'sign_p': p,
                'hodges_lehmann': round(hodges_lehmann(d), 1), 'median': round(st.median(d), 1)}
    out['expected_makespan'] = {
        f: round(st.mean(out['per_order'][o][f]['mean_makespan'] for o in ('x', 'y')), 1)
        for f in FORMS if all('mean_makespan' in out['per_order'][o][f] for o in ('x', 'y'))}
    for f in FORMS:
        out['per_formulation'][f]['median_r1_in_bay'] = st.median(
            by[(f, s)]['r1_in_bay'] for s in seeds if by[(f, s)]['r1_in_bay'] is not None)
        out['per_formulation'][f]['median_planning_seconds'] = st.median(
            by[(f, s)]['planning_seconds'] for s in seeds)
    b = sum(1 for s in seeds if by[('epistemic', s)]['correct'] and not by[('optimistic', s)]['correct'])
    c = sum(1 for s in seeds if by[('optimistic', s)]['correct'] and not by[('epistemic', s)]['correct'])
    out['paired']['correct_epistemic_vs_optimistic'] = {'only_epistemic': b, 'only_optimistic': c,
                                                         'mcnemar_p': mcnemar(b, c)}
    json.dump(out, open(a.json or os.path.join(a.batch, 'analysis.json'), 'w'), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
