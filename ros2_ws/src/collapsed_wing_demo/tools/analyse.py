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
The batch of the collapsed wing, run by run and in sum.

    analyse.py BATCH

Writes BATCH/runs.csv, one row per run, and BATCH/analysis.json, the sums per
formulation and per world. Everything is read from the run's own files: where
the robots were, what each one's map decided, what was said on the radio and
what the executor ran.

Per run:
  reached      the medic ended within 0.5 m of the victim, by its own odometry;
  reported     the mission ended with the executor reporting success;
  t_mission    simulated seconds from the mission's start to its end;
  crossed      the way the medic reached the north wing by;
  told         that way had been said clear on the radio before the medic set
               out on it: what the medic knew, and not what it assumed;
  dead_ends    ways the medic set out on and found rubble in;
  medic_m      metres the medic drove; scouts_m, metres the two scouts drove;
  wrong        doorway decisions of a scout's map that contradict the world;
  known_<r>    m^2 of the building each robot's own SLAM map holds at the end.

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
import grid as G  # noqa: E402
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
       12: 2.179, 14: 2.145, 16: 2.120, 18: 2.101, 20: 2.086, 25: 2.060, 30: 2.042}


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
    if len(xs) < 2:
        return {'n': len(xs), 'mean': round(m, 2), 'min': round(min(xs), 2), 'max': round(max(xs), 2)}
    h = t95(len(xs) - 1) * float(np.std(xs, ddof=1)) / math.sqrt(len(xs))
    return {'n': len(xs), 'mean': round(m, 2), 'ci': [round(m - h, 2), round(m + h, 2)],
            'min': round(min(xs), 2), 'max': round(max(xs), 2)}


def rows(path):
    try:
        return list(csv.DictReader(open(path)))
    except OSError:
        return []


def run(d):
    launch = json.load(open(os.path.join(d, 'launch.json')))
    world = json.load(open(os.path.join(d, 'world.json')))
    m = json.load(open(os.path.join(d, 'mission.json')))
    t0 = m.get('t0_sim', 0.0)
    clear = world['clear']
    tag = ''.join(k.upper() if v else k for k, v in clear.items())
    track = {}
    for r in rows(os.path.join(d, 'pose.csv')):
        if float(r['t_sim']) >= t0:
            track.setdefault(r['robot'], []).append((float(r['x']), float(r['y'])))
    dist = {r: sum(math.dist(a, b) for a, b in zip(p, p[1:])) for r, p in track.items()}
    acts = rows(os.path.join(d, 'actions.csv'))
    radio = [(float(r['t_sim']), r['message']) for r in rows(os.path.join(d, 'radio.csv'))]
    crossed, told, dead = '', None, []
    for a in acts:
        if not a['action'].startswith('cross_'):
            continue
        way = a['action'][-1]
        if a['status'] == 'FAILED':
            dead.append(way)
        elif a['status'] == 'SUCCEEDED':
            crossed = way
            start = max((float(b['t_sim']) for b in acts if b['action'] == a['action'] and b['status'] == 'EXECUTING'),
                        default=float(a['t_sim']))
            told = any(msg == f'{way}=clear' and t <= start for t, msg in radio)
    wrong = 0
    for r, door in (('s1', 'e'), ('s2', 'w')):
        for row in rows(os.path.join(d, f'decision_{r}.csv')):
            if row['door'] == door and row['decision'] and (row['decision'] == 'clear') != clear[door]:
                wrong += 1
    known = {}
    for r in ('s1', 's2', 'm'):
        try:
            z = np.load(os.path.join(d, 'maps', f'{r}.npz'))
            known[r] = round(G.known_area(z['grid'][-1]), 2)
        except (OSError, KeyError, IndexError):
            known[r] = None
    end = m.get('medic_at_end')
    return {
        'dir': os.path.basename(d), 'seed': world['seed'], 'world': tag, 'formulation': launch['formulation'],
        'result': m.get('result', 'NO_OUTCOME'), 'reported': m.get('result') == 'SUCCESS',
        'reached': bool(end and math.dist(end, L.VICTIM) < 0.5),
        't_mission': m.get('t_mission_sim'), 'crossed': crossed, 'told': told,
        'dead_ends': len(dead), 'dead_ways': ''.join(dead),
        'medic_m': round(dist.get('m', 0.0), 2), 'scouts_m': round(dist.get('s1', 0.0) + dist.get('s2', 0.0), 2),
        'wrong': wrong, 'known_s1': known['s1'], 'known_s2': known['s2'], 'known_m': known['m'],
        'plans': len(m.get('plans', [])),
    }


def summary(runs, keys):
    k = {key: sum(1 for r in runs if r[key]) for key in ('reached', 'reported')}
    n = len(runs)
    return {
        'n': n, **{key: {'k': v, 'wilson': wilson(v, n)} for key, v in k.items()},
        'results': {x: sum(1 for r in runs if r['result'] == x) for x in sorted({r['result'] for r in runs})},
        't_mission': mean_ci([r['t_mission'] for r in runs if r['reached']]),
        'dead_ends': sum(r['dead_ends'] for r in runs),
        'runs_with_dead_end': sum(1 for r in runs if r['dead_ends']),
        'told': sum(1 for r in runs if r['told']),
        'medic_m': mean_ci([r['medic_m'] for r in runs if r['reached']]),
        'scouts_m': mean_ci([r['scouts_m'] for r in runs]),
        'wrong': sum(r['wrong'] for r in runs),
        **{f'known_{x}': mean_ci([r[f'known_{x}'] for r in runs]) for x in ('s1', 's2', 'm')},
    }


def main():
    batch = sys.argv[1]
    runs = []
    for name in sorted(os.listdir(batch)):
        d = os.path.join(batch, name)
        if os.path.isdir(d) and all(os.path.exists(os.path.join(d, f)) for f in ('mission.json', 'launch.json')):
            runs.append(run(d))
    runs.sort(key=lambda r: (W.FORMULATIONS.index(r['formulation']), r['seed']))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(runs[0]))
        w.writeheader()
        w.writerows(runs)
    out = {'runs': len(runs), 'seeds': len({r['seed'] for r in runs}), 'per_formulation': {}, 'per_world': {},
           'paired': {}}
    for f in W.FORMULATIONS:
        rs = [r for r in runs if r['formulation'] == f]
        out['per_formulation'][f] = summary(rs, None)
        for tag in sorted({r['world'] for r in rs}, key=lambda t: [w for w in W.WORLDS].index(
                {k: c.isupper() for k, c in zip(W.WAYS, t)})):
            out['per_world'].setdefault(tag, {})[f] = summary([r for r in rs if r['world'] == tag], None)
    by = {(r['formulation'], r['seed']): r for r in runs}
    for f in ('optimistic', 'gated'):
        diffs = [by[('epistemic', s)]['t_mission'] - by[(f, s)]['t_mission'] for s in sorted({r['seed'] for r in runs})
                 if (('epistemic', s) in by and (f, s) in by and by[('epistemic', s)]['reached']
                     and by[(f, s)]['reached'])]
        out['paired'][f] = {'n': len(diffs), 'epistemic_minus': mean_ci(diffs),
                            'epistemic_faster': sum(1 for x in diffs if x < 0)}
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    for f, s in out['per_formulation'].items():
        print(f"{f:11s} n={s['n']:2d} reached {s['reached']['k']:2d} reported {s['reported']['k']:2d} "
              f"t {s['t_mission'].get('mean', '-')} (worst {s['t_mission'].get('max', '-')})  dead ends "
              f"{s['dead_ends']} in {s['runs_with_dead_end']} runs  told {s['told']}  medic "
              f"{s['medic_m'].get('mean', '-')} m  wrong {s['wrong']}  results {s['results']}")
    for tag, fs in out['per_world'].items():
        print(f'  {tag}: ' + '  '.join(f"{f} {s['reached']['k']}/{s['n']} {s['t_mission'].get('mean', '-')}s"
                                        for f, s in fs.items()))


if __name__ == '__main__':
    main()
