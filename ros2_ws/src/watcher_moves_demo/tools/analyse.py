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
The batch of the watcher that moves, run by run and in sum.

    analyse.py BATCH

Reads BATCH/runs/<formulation>-<strategy>-s<seed>/ and writes BATCH/runs.csv
and BATCH/analysis.json. A run is judged by what the robots' own lasers
recorded, and the watcher is judged twice:

  by the events   it names a bay if its laser had the carrier in a gate or a
                  bay, or saw the lamp -- what it saw means the bay by itself;
  by the policy   it also knows the policy, and has watched it run before: the
                  ordered places its laser had the carrier at (windows, gate
                  approaches, gates, bays) and the lamp make a trace, and it
                  names the bay of every other run of the same formulation,
                  under the same strategy, from the same post, that showed it
                  the same trace, if all of them had one bay. Each run is
                  judged on the others only.

  kept      delivered, the ally named the bay, and the watcher named none,
            by the events;
  hidden    kept, and the watcher that knows the policy named none either.

Per formulation, the worst of the two strategies is the watcher's best.
"""

import csv
import json
import math
import os
import sys
from collections import defaultdict
from statistics import median

FORMS = ('v1', 'shortest', 'opaque', 'direct', 'discreet')
STRATS = ('stay', 'chase')
PLACES = ('window_w', 'window_e', 'approach_w', 'approach_e', 'gate_x', 'gate_y', 'bay_x', 'bay_y')


def wilson(k, n, z=1.959964):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(max(0.0, c - h), 3), round(min(1.0, c + h), 3))


def load(path, default=None):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return default


def trace(d):
    """What the watcher saw, in order: the places it had the carrier at, and the lamp."""
    events = []
    try:
        for r in csv.DictReader(open(os.path.join(d, 'observer_r3.csv'))):
            if r['event'] == 'begin' and r['region'] in PLACES:
                events.append((float(r['t_sim']), r['region']))
            elif r['event'] == 'lamp':
                events.append((float(r['t_sim']), f'lamp {r["region"]}'))
    except OSError:
        return ()
    out = []
    for _, e in sorted(events):
        if not out or out[-1] != e:
            out.append(e)
    return tuple(out)


def main():
    batch = sys.argv[1]
    runs = os.path.join(batch, 'runs')
    out = []
    for name in sorted(os.listdir(runs)):
        d = os.path.join(runs, name)
        if not os.path.isdir(d) or name.endswith('.first-attempt'):
            continue
        rest, seed = name.rsplit('-s', 1)
        form, strat = rest.rsplit('-', 1)
        w = load(os.path.join(d, 'world.json'), {})
        m = load(os.path.join(d, 'mission.json'), {})
        a = load(os.path.join(d, 'observer_r2.json'), {})
        o = load(os.path.join(d, 'observer_r3.json'), {})
        wt = load(os.path.join(d, 'watcher.json'), {})
        order = w.get('order')
        final = (m.get('final_bay') or {}).get('r1', '')
        moves = wt.get('moves') or []
        out.append({
            'seed': int(seed), 'formulation': form, 'strategy': strat, 'order': order, 'watcher': w.get('watcher'),
            'same_side': int(bool(w.get('same_side'))), 'result': m.get('result', 'NO_OUTCOME'),
            't_mission': m.get('t_mission_sim'), 'final_bay': final, 'delivered': int(final == order),
            'ally_saw': a.get('conclusion', ''), 'ally_knows': int(a.get('conclusion') == order),
            'watcher_saw': o.get('conclusion', ''), 'leaked_events': int(bool(o.get('conclusion'))),
            'watcher_moved': int(bool(moves)), 'watcher_arrived': int(bool(moves) and 't_arrived' in moves[0]),
            'trace': ' > '.join(trace(d)), 'reported_success': int(m.get('result') == 'SUCCESS'),
            'model_kw_r2': (m.get('model') or {}).get('(Kw r2 dest-x)'),
            'model_kw_r3': (m.get('model') or {}).get('(Kw r3 dest-x)'),
        })
    # the watcher that knows the policy, judged on the other runs only
    seen = defaultdict(list)
    for r in out:
        seen[(r['formulation'], r['strategy'], r['watcher'], r['trace'])].append(r)
    for r in out:
        others = {x['order'] for x in seen[(r['formulation'], r['strategy'], r['watcher'], r['trace'])] if x is not r}
        r['policy_names'] = next(iter(others)) if len(others) == 1 else ''
        r['leaked_policy'] = int(bool(r['leaked_events']) or r['policy_names'] == r['order'])
        r['kept'] = int(r['delivered'] and r['ally_knows'] and not r['leaked_events'])
        r['hidden'] = int(r['kept'] and not r['leaked_policy'])
    keys = list(dict.fromkeys(k for r in out for k in r))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        wr = csv.DictWriter(fh, fieldnames=keys)
        wr.writeheader()
        wr.writerows(out)

    res = {'seeds': len({r['seed'] for r in out}), 'cells': {}, 'worst': {}}
    for f in FORMS:
        worst = None
        for s in STRATS:
            rs = [r for r in out if r['formulation'] == f and r['strategy'] == s]
            n = len(rs)
            if not n:
                continue
            times = [r['t_mission'] for r in rs if r['t_mission'] is not None and r['reported_success']]
            cell = {'runs': n, 'kept': sum(r['kept'] for r in rs), 'hidden': sum(r['hidden'] for r in rs),
                    'hidden_ci': wilson(sum(r['hidden'] for r in rs), n),
                    'reported_success': sum(r['reported_success'] for r in rs),
                    'leaked_events': sum(r['leaked_events'] for r in rs),
                    'leaked_policy': sum(r['leaked_policy'] for r in rs),
                    'ally_knows': sum(r['ally_knows'] for r in rs), 'delivered': sum(r['delivered'] for r in rs),
                    'watcher_moved': sum(r['watcher_moved'] for r in rs),
                    'watcher_arrived': sum(r['watcher_arrived'] for r in rs),
                    'median_time': round(median(times), 1) if times else None}
            res['cells'][f'{f}-{s}'] = cell
            if worst is None or cell['hidden'] < worst[1]['hidden']:
                worst = (s, cell)
        if worst:
            res['worst'][f] = {'strategy': worst[0], **worst[1]}
    json.dump(res, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    for k, c in res['cells'].items():
        print(f'{k:18s} hidden {c["hidden"]}/{c["runs"]}  kept {c["kept"]}  leaked by events {c["leaked_events"]}  '
              f'by the policy {c["leaked_policy"]}  reported {c["reported_success"]}  median {c["median_time"]} s')


if __name__ == '__main__':
    main()
