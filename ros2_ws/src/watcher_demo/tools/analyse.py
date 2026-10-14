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
The batch of the watcher, run by run and in sum.

    analyse.py BATCH

Reads BATCH/runs/<formulation>-s<seed>/ and writes BATCH/runs.csv, one row per
run, and BATCH/analysis.json, the figures the page, the report and the film
quote.

A run is judged by what the robots' own lasers recorded (observer_node), not
by what the executor reports:

  delivered    the carrier ended in the bay its order names;
  ally_knows   the ally's laser saw the carrier in that bay's gate, or saw its
               lamp, and so the ally can name the bay;
  leaked       the watcher's laser saw the carrier in a gate or in a bay, or saw
               its lamp: the watcher can name the bay;
  kept         delivered, the ally knows, and nothing leaked: the mission's goal,
               read off the sensors.

The executor's own report, SUCCESS or not, is kept beside it; for the policy,
so is what the executor's model says r2 and r3 know at the end, so that the
model can be checked against the sensors run by run.
"""

import csv
import json
import math
import os
import sys
from statistics import median


def wilson(k, n, z=1.959964):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(max(0.0, c - h), 3), round(min(1.0, c + h), 3))


def mcnemar(b, c):
    """Exact two-sided McNemar p for b and c discordant pairs."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(0, k + 1)) / 2 ** n
    return min(1.0, 2 * p)


def hodges_lehmann(d):
    w = sorted((d[i] + d[j]) / 2 for i in range(len(d)) for j in range(i, len(d)))
    return round(median(w), 1) if w else None


def load(path, default=None):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return default


def main():
    batch = sys.argv[1]
    runs = os.path.join(batch, 'runs')
    out = []
    for name in sorted(os.listdir(runs)):
        d = os.path.join(runs, name)
        if not os.path.isdir(d) or name.endswith('.first-attempt'):
            continue
        form, seed = name.rsplit('-s', 1)
        w = load(os.path.join(d, 'world.json'), {})
        m = load(os.path.join(d, 'mission.json'), {})
        a = load(os.path.join(d, 'observer_r2.json'), {})
        o = load(os.path.join(d, 'observer_r3.json'), {})
        order = w.get('order')
        final = (m.get('final_bay') or {}).get('r1', '')
        delivered = final == order
        ally_knows = a.get('conclusion') == order
        leaked = bool(o.get('conclusion'))
        model = m.get('model') or {}
        row = {
            'seed': int(seed), 'formulation': form, 'order': order, 'watcher': w.get('watcher'),
            'same_side': int(bool(w.get('same_side'))), 'result': m.get('result', 'NO_OUTCOME'),
            't_mission': m.get('t_mission_sim'), 'final_bay': final, 'delivered': int(delivered),
            'ally_saw': a.get('conclusion', ''), 'watcher_saw': o.get('conclusion', ''),
            'ally_knows': int(ally_knows), 'leaked': int(leaked),
            'watcher_sightings': sum((o.get('sightings') or {}).values()),
            'watcher_regions': ' '.join(sorted((o.get('first_seen') or {}).keys())),
            'kept': int(delivered and ally_knows and not leaked),
            'reported_success': int(m.get('result') == 'SUCCESS'),
            'model_kw_r2': model.get('(Kw r2 dest-x)'), 'model_kw_r3': model.get('(Kw r3 dest-x)'),
        }
        row['success_with_leak'] = int(row['reported_success'] and leaked)
        if form == 'epistemic' and model:
            row['model_matches_sensors'] = int(model.get('(Kw r2 dest-x)') == ally_knows and
                                               model.get('(Kw r3 dest-x)') == leaked)
        out.append(row)
    keys = list(dict.fromkeys(k for r in out for k in r))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        wr = csv.DictWriter(fh, fieldnames=keys)
        wr.writeheader()
        wr.writerows(out)

    forms = ['epistemic', 'direct', 'discreet']
    res = {'seeds': len({r['seed'] for r in out}), 'per_formulation': {}, 'per_world': {}, 'paired': {}}
    for f in forms:
        rs = [r for r in out if r['formulation'] == f]
        n = len(rs)
        k = sum(r['kept'] for r in rs)
        guessed = sum(1 for r in rs if r['leaked'] and r['watcher_saw'] == r['order'])
        blind = sum(1 for r in rs if not r['leaked'])
        times = [r['t_mission'] for r in rs if r['t_mission'] is not None and r['reported_success']]
        res['per_formulation'][f] = {
            'runs': n, 'kept': k, 'kept_ci': wilson(k, n),
            'reported_success': sum(r['reported_success'] for r in rs),
            'delivered': sum(r['delivered'] for r in rs), 'ally_knows': sum(r['ally_knows'] for r in rs),
            'leaked': sum(r['leaked'] for r in rs), 'success_with_leak': sum(r['success_with_leak'] for r in rs),
            # the watcher names the bay it saw, and guesses when it saw none
            'watcher_accuracy': round((guessed + 0.5 * blind) / n, 3) if n else None,
            'watcher_sightings_total': sum(r['watcher_sightings'] for r in rs),
            'median_time': round(median(times), 1) if times else None,
        }
        if f == 'epistemic':
            res['per_formulation'][f]['model_matches_sensors'] = sum(r.get('model_matches_sensors', 0) for r in rs)
    for order in ('x', 'y'):
        for watch in ('w', 'e'):
            key = f'{order}-{watch}'
            res['per_world'][key] = {}
            for f in forms:
                rs = [r for r in out if r['formulation'] == f and r['order'] == order and r['watcher'] == watch]
                times = [r['t_mission'] for r in rs if r['kept'] and r['t_mission'] is not None]
                res['per_world'][key][f] = {
                    'runs': len(rs), 'kept': sum(r['kept'] for r in rs), 'leaked': sum(r['leaked'] for r in rs),
                    'ally_knows': sum(r['ally_knows'] for r in rs),
                    'median_time': round(median(times), 1) if times else None}
    by = {(r['formulation'], r['seed']): r for r in out}
    for f in forms[1:]:
        pairs = [(by[('epistemic', s)], by[(f, s)]) for (g, s) in by if g == f and ('epistemic', s) in by]
        b = sum(1 for e, c in pairs if e['kept'] and not c['kept'])
        c_ = sum(1 for e, c in pairs if c['kept'] and not e['kept'])
        both = [(e['t_mission'] - c['t_mission']) for e, c in pairs if e['kept'] and c['kept']
                and e['t_mission'] is not None and c['t_mission'] is not None]
        res['paired'][f] = {'pairs': len(pairs), 'only_epistemic': b, 'only_classical': c_,
                            'mcnemar_p': mcnemar(b, c_), 'both_kept': len(both),
                            'time_diff_hl': hodges_lehmann(both),
                            'time_diff_median': round(median(both), 1) if both else None}
    json.dump(res, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    for f in forms:
        p = res['per_formulation'][f]
        print(f'{f:10s} kept {p["kept"]}/{p["runs"]} {p["kept_ci"]}  reported {p["reported_success"]}  '
              f'leaked {p["leaked"]}  success-with-leak {p["success_with_leak"]}  ally {p["ally_knows"]}  '
              f'watcher accuracy {p["watcher_accuracy"]}  median {p["median_time"]} s')
    for k, v in res['per_world'].items():
        print(k, {f: (v[f]['kept'], v[f]['runs'], v[f]['median_time']) for f in forms})
    print(json.dumps(res['paired']))


if __name__ == '__main__':
    main()
