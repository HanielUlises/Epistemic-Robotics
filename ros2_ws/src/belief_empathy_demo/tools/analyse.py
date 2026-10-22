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
The batch of the belief and empathy demonstration, run by run and in sum.

    analyse.py BATCH

Writes BATCH/runs.csv, one row per run, and BATCH/analysis.json, the sums per
formulation, per number of faults (the paper's Table I) and per world.
Everything is read from the run's own files: where the robots were, what was
said on the radio, what each robot reported, and when the task was found and
done.

Per run:
  done         the task was done: the aerial vehicle and a ground vehicle with
               fewer than two faults within layout.R_SITE of the site for
               layout.DELTA seconds, by the simulator's poses;
  t_mission    seconds from the start to the task done, the paper's mission time;
  t_found      seconds from the start to the task found;
  t_told       seconds from the start until the vehicle that did the task knew
               of it: the hail it answered, or the assignment it heard;
  by           the vehicle that did the task;
  hails        the particles the aerial vehicle called at, in order, and what
               it found: g2@1- for nobody at g2's first, g2@2+ for a meeting;
  p3           a third particle was called at;
  violations   meetings at a particle the vehicle did not hold (the model
               says none can happen);
  u_m, ugv_m   metres the aerial vehicle flew, and the doer drove, from the
               task found to the task done;
  plan_s       wall seconds the planner took (epistemic only);
  failed       attempts that failed on a latent tool fault (the variant only).

The variant with latent tool faults is told apart by its world.json, and its
sums are kept per pattern; its paired comparisons are of the policy and of the
published procedure against the strong one.

The sums carry Wilson intervals for proportions and t intervals for means, at
95 %; times are compared seed by seed, every formulation having run every seed
from the same start.
"""

import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
       11: 2.201, 12: 2.179, 14: 2.145, 16: 2.120, 18: 2.101, 20: 2.086, 23: 2.069, 25: 2.060, 30: 2.042}


def t95(df):
    return T95[min(T95, key=lambda k: (abs(k - df), k))]


def wilson(k, n, z=1.96):
    if n == 0:
        return [0.0, 0.0]
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(max(0.0, c - h), 3), round(min(1.0, c + h), 3)]


def mean_ci(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {'n': 0}
    m = float(np.mean(xs))
    out = {'n': len(xs), 'mean': round(m, 2), 'min': round(min(xs), 2), 'max': round(max(xs), 2)}
    if len(xs) >= 2:
        h = t95(len(xs) - 1) * float(np.std(xs, ddof=1)) / math.sqrt(len(xs))
        out['ci'] = [round(m - h, 2), round(m + h, 2)]
    return out


def rows(path):
    try:
        with open(path) as fh:
            return list(csv.DictReader(fh))
    except OSError:
        return []


def travelled(poses, robot, t0, t1):
    track = [(float(r['t_sim']) - W.START_AT, float(r['x']), float(r['y'])) for r in poses if r['robot'] == robot]
    track = [p for p in track if t0 <= p[0] <= t1]
    return sum(math.dist(a[1:], b[1:]) for a, b in zip(track, track[1:]))


def one(run):
    world = json.load(open(os.path.join(run, 'world.json')))
    launch = json.load(open(os.path.join(run, 'launch.json')))
    m = json.load(open(os.path.join(run, 'mission.json')))
    events = rows(os.path.join(run, 'events.csv'))
    poses = rows(os.path.join(run, 'pose.csv'))
    radio = rows(os.path.join(run, 'radio.csv'))
    done = m.get('done') or None
    found = m.get('found') or None
    by = done['by'] if done else None
    hails, violations, failed = [], 0, 0
    for e in events:
        data = json.loads(e['data'])
        if e['robot'] == 'u' and e['kind'] in ('met', 'absent'):
            hails.append(f"{data['g']}@{data['b']}{'+' if e['kind'] == 'met' else '-'}")
        if e['kind'] == 'model_violation':
            violations += 1
        if e['kind'] == 'task_failed':
            failed += 1
    told = None
    if by:
        for e in events:
            if e['robot'] == by and e['kind'] in ('heard', 'assigned'):
                told = float(e['t'])
                break
    t_found = found['t'] if found else None
    t_done = done['t'] if done else None
    p = world['world']
    return {
        'run': os.path.basename(run), 'seed': world['seed'], 'formulation': launch['formulation'],
        'variant': world.get('variant', 'replica'), 'pattern': ''.join(map(str, world.get('pattern', []))),
        'failed': failed,
        'p_g1': p['g1'], 'p_g2': p['g2'], 'faults': W.faults((p['g1'], p['g2'])),
        'site': f"{world['site'][0]:+.0f}/{world['site'][1]:+.0f}", 'side': world['side'],
        'result': m.get('result'), 'done': done is not None, 't_mission': t_done, 't_found': t_found,
        't_told': told, 'by': by, 'hails': ' '.join(hails), 'n_hails': len(hails),
        'p3': any('@3' in h for h in hails), 'violations': violations,
        'u_m': round(travelled(poses, 'u', t_found, t_done), 2) if t_done else None,
        'ugv_m': round(travelled(poses, by, t_found, t_done), 2) if t_done else None,
        'plan_s': m['plans'][0]['seconds'] if m.get('plans') else None,
        'radio_sent': len(radio), 'radio_delivered': sum(r['delivered'] == 'True' for r in radio),
        'decisions': m.get('decisions', []),
    }


def summary(rs):
    k = sum(r['done'] for r in rs)
    return {'runs': len(rs), 'done': k, 'done_ci': wilson(k, len(rs)),
            't_mission': mean_ci([r['t_mission'] for r in rs if r['done']]),
            't_found': mean_ci([r['t_found'] for r in rs]),
            'search': mean_ci([r['t_told'] - r['t_found'] for r in rs if r['t_told'] and r['t_found']]),
            'hails': mean_ci([r['n_hails'] for r in rs]),
            'p3': sum(r['p3'] for r in rs), 'violations': sum(r['violations'] for r in rs),
            'failed': sum(r['failed'] for r in rs),
            'u_m': mean_ci([r['u_m'] for r in rs if r['done']]),
            'ugv_m': mean_ci([r['ugv_m'] for r in rs if r['done']]),
            'plan_s': mean_ci([r['plan_s'] for r in rs if r['plan_s'] is not None])}


def main():
    batch = sys.argv[1]
    runs = []
    for d in sorted(os.listdir(batch)):
        path = os.path.join(batch, d)
        if os.path.isdir(path) and os.path.exists(os.path.join(path, 'mission.json')) \
                and os.path.exists(os.path.join(path, 'launch.json')):
            runs.append(one(path))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        keys = [k for k in runs[0] if k != 'decisions']
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        for r in sorted(runs, key=lambda r: (r['formulation'], r['seed'])):
            w.writerow(r)
    latent = runs[0]['variant'] == 'latent'
    out = {'variant': 'latent' if latent else 'replica', 'formulations': {}, 'by_faults': {}, 'by_world': {},
           'paired': {}}
    order = W.LATENT_FORMULATIONS if latent else W.FORMULATIONS
    forms = [f for f in order if any(r['formulation'] == f for r in runs)]
    for f in forms:
        mine = [r for r in runs if r['formulation'] == f]
        out['formulations'][f] = summary(mine)
        if latent:
            out['by_world'][f] = {pat: summary([r for r in mine if r['pattern'] == pat])
                                  for pat in sorted({r['pattern'] for r in mine})}
            continue
        out['by_faults'][f] = {str(k): summary([r for r in mine if r['faults'] == k]) for k in (0, 1, 2)}
        out['by_world'][f] = {f"{p1}{p2}{side[0]}": summary([r for r in mine if (r['p_g1'], r['p_g2']) == (p1, p2)
                                                             and r['side'] == side])
                              for p1, p2 in W.WORLDS for side in ('west', 'east')}
    # Seed by seed: each pair compared, a minus b.
    by_seed = {(r['formulation'], r['seed']): r for r in runs}
    pairs = ([('epistemic', 'strong'), ('timed', 'strong'), ('timed', 'epistemic'), ('published', 'strong'),
              ('epistemic', 'published'), ('timed', 'ideal')]
             if latent else [('epistemic', f) for f in forms if f != 'epistemic']
             + [('timed', f) for f in ('particles', 'ideal', 'flock')])
    for a_, b_ in pairs:
        if a_ not in forms or b_ not in forms:
            continue
        diffs, wins, same = [], 0, 0
        for s_ in sorted({r['seed'] for r in runs}):
            a, b = by_seed.get((a_, s_)), by_seed.get((b_, s_))
            if a and b and a['done'] and b['done']:
                d = a['t_mission'] - b['t_mission']
                diffs.append(d)
                wins += d < -1.0
                same += abs(d) <= 1.0
        key = b_ if a_ == 'epistemic' and not latent else f'{a_}-{b_}'
        out['paired'][key] = {'n': len(diffs), 'epistemic_minus': mean_ci(diffs), 'epistemic_faster': wins,
                              'within_1s': same, 'a': a_, 'b': b_}
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    print(f"{'formulation':<12}{'done':>8}{'mean':>9}{'worst':>9}{'hails':>7}{'p3':>4}{'fail':>5}{'viol':>5}")
    for f in forms:
        s_ = out['formulations'][f]
        print(f"{f:<12}{s_['done']:>4}/{s_['runs']:<3}{s_['t_mission'].get('mean', float('nan')):>9.1f}"
              f"{s_['t_mission'].get('max', float('nan')):>9.1f}{s_['hails'].get('mean', 0):>7.2f}{s_['p3']:>4}"
              f"{s_['failed']:>5}{s_['violations']:>5}")
    if not latent:
        for f in forms:
            bf = [out['by_faults'][f][k]['t_mission'].get('mean', float('nan')) for k in ('0', '1', '2')]
            print(f'{f:<12} by faults 0/1/2: ' + ' '.join(f'{v:.1f}' for v in bf))
    for k, p in out['paired'].items():
        e = p['epistemic_minus']
        print(f"{p['a']} - {p['b']}: {e.get('mean', float('nan')):+.1f} s over {p['n']} seeds, "
              f"{p['a']} sooner in {p['epistemic_faster']}, within 1 s in {p['within_1s']}")


if __name__ == '__main__':
    main()
