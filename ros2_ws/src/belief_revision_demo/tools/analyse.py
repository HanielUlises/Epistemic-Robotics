#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
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
A batch of runs, tabulated.

    analyse.py BATCH

BATCH/runs.csv       one row a run: the night, the formulation, the result,
                     the mission's time from the couriers' setting out, each
                     courier's own time, the doors looked at and found shut,
                     the lobbies entered, the reports, the distance driven, and
                     the time tools/plan.py predicted for the same night;
BATCH/analysis.json  per act and formulation: the mean time of each night, the
                     expectation over the nights under the porter's
                     frequencies (tools/ranks.py), the summed couriers' time,
                     how many runs arrived, and the predicted figures.

A run's time is taken from the actions' own records: from the start of the
first action to the end of the last, in simulated seconds.
"""

import csv
import glob
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def run_row(d):
    m = json.load(open(os.path.join(d, 'mission.json')))
    w = json.load(open(os.path.join(d, 'world.json')))
    act, f = m['act'], m['formulation']
    acts = read(os.path.join(d, 'actions.csv')) if os.path.exists(os.path.join(d, 'actions.csv')) else []
    t0 = m.get('t0_sim', 0.0)
    started = [float(a['t_sim']) for a in acts if a['status'] == 'EXECUTING']
    ended = [float(a['t_sim']) for a in acts if a['status'] in ('SUCCEEDED', 'FAILED')]
    first = min(started) if started else t0
    done = {}
    for a in acts:
        if a['status'] == 'SUCCEEDED' and a['action'] == 'drive':
            r, x, y = a['arguments'].split()
            if (act < 3 and y in ('rc', 'rd')) or (act == 3 and y in ('hc', 'ht')):
                done[r] = round(float(a['t_sim']) - first, 1)
    looks = [(a['arguments'].split()[0], a['arguments'].split()[-1][-2:], a['message'])
             for a in acts if a['action'] == 'look' and a['status'] == 'SUCCEEDED']
    enters = [(a['arguments'].split()[0], a['arguments'].split()[1][-1])
              for a in acts if a['action'] == 'lobby' and a['status'] == 'SUCCEEDED'
              and a['arguments'].split()[2][0] in 'vn' and a['arguments'].split()[3][0] in 'jk']
    tells = [a['arguments'] for a in acts if a['action'] == 'tell' and a['status'] == 'SUCCEEDED']
    driven = {}
    poses = read(os.path.join(d, 'pose.csv')) if os.path.exists(os.path.join(d, 'pose.csv')) else []
    last = {}
    for p in poses:
        r = p['robot']
        if r == 'p' or float(p['t_sim']) < first:
            continue
        xy = (float(p['x']), float(p['y']))
        if r in last:
            driven[r] = driven.get(r, 0.0) + math.dist(last[r], xy)
        last[r] = xy
    world = (tuple(w['loop']), w['plan'])
    tree = tree_of(act, f)
    predicted, pdone = P.timeline(tree, world)
    t_mission = round((max(ended) - first) if ended else 0.0, 1)
    round_time = None
    rounds = [a for a in acts if a['action'] == 'round']
    if rounds:
        s = [float(a['t_sim']) for a in rounds if a['status'] == 'EXECUTING']
        e = [float(a['t_sim']) for a in rounds if a['status'] == 'SUCCEEDED']
        if s and e:
            round_time = round(min(e) - min(s), 1)
    return {
        'run': os.path.basename(d), 'act': act, 'formulation': f, 'seed': w['seed'], 'world': w['tag'],
        'frequency': R.frequency(world), 'result': m.get('result'), 'arrived': m.get('arrived'),
        't_mission': t_mission, 't_couriers': round(sum(done.values()), 1),
        'done': json.dumps(done), 'looks': ' '.join(f'{r}:{v}={o}' for r, v, o in looks),
        'shut_found': sum(1 for _, _, o in looks if o == 'shut'), 'entered': len(enters), 'tells': len(tells),
        'driven_m': round(sum(driven.values()), 1), 'round_s': round_time,
        'predicted': predicted, 'predicted_done': json.dumps(pdone),
    }


_TREES = {}


def tree_of(act, f):
    if (act, f) not in _TREES:
        _TREES[(act, f)] = P.Team(act, f).tree()
    return _TREES[(act, f)]


def main():
    batch = sys.argv[1]
    rows = []
    for d in sorted(glob.glob(os.path.join(batch, 'a*-s*'))):
        if os.path.exists(os.path.join(d, 'mission.json')) and os.path.exists(os.path.join(d, 'world.json')):
            try:
                rows.append(run_row(d))
            except Exception as e:  # noqa: BLE001
                print(f'{d}: {e}')
    if not rows:
        print('no runs')
        return
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    out = {}
    for act in sorted({r['act'] for r in rows}):
        out[act] = {}
        for f in R.FORMULATIONS:
            rs = [r for r in rows if r['act'] == act and r['formulation'] == f]
            if not rs:
                continue
            per = {}
            for r in rs:
                per.setdefault(r['world'], []).append(r)
            # a night's time is the mean of the runs that arrived; a run that did not is
            # counted, and its time is not a mission's
            nights = {}
            for k, v in per.items():
                ok = [x for x in v if x['arrived']]
                if not ok:
                    continue
                nights[k] = {'mean': round(statistics.mean(x['t_mission'] for x in ok), 1),
                             'sd': round(statistics.stdev(x['t_mission'] for x in ok), 1) if len(ok) > 1 else 0.0,
                             'couriers': round(statistics.mean(x['t_couriers'] for x in ok), 1),
                             'runs': len(v), 'arrived': len(ok),
                             'predicted': v[0]['predicted'], 'frequency': v[0]['frequency']}
            covered = sum(n['frequency'] for n in nights.values())
            exp = sum(n['frequency'] * n['mean'] for n in nights.values()) / covered if covered else None
            expc = sum(n['frequency'] * n['couriers'] for n in nights.values()) / covered if covered else None
            out[act][f] = {'nights': nights, 'expected': round(exp, 1) if exp else None,
                           'expected_couriers': round(expc, 1) if expc else None,
                           'expected_predicted': round(P.expected(act, f, tree_of(act, f)), 1),
                           'covered_frequency': round(covered, 4),
                           'runs': len(rs), 'arrived': sum(bool(r['arrived']) for r in rs),
                           'results': {k: sum(r['result'] == k for r in rs) for k in {r['result'] for r in rs}}}
            print(f"act {act} {f:13s}: {len(rs)} runs, {out[act][f]['arrived']} arrived, expected "
                  f"{out[act][f]['expected']} s (predicted {out[act][f]['expected_predicted']})  " +
                  ' '.join(f"{k}:{v['mean']:.0f}/{v['predicted']:.0f}" for k, v in sorted(nights.items())))
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
