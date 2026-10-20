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
The batch of the link grades, run by run and in sum.

    analyse.py BATCH

Writes BATCH/runs.csv, one row per run, and BATCH/analysis.json, the sums per
formulation and grade. Everything is read from the run's own files; nothing is
taken from what the system reported about itself that its maps do not show.

Per run:
  reported     the mission ended with the executor reporting success;
  backed       reported, and at the end each robot's map -- its own evidence
               and what reached it over the link -- decides the corridor, and
               decides it right;
  unbacked     reported and not backed: the system said both know, and a
               robot's map does not;
  t_mission    simulated seconds from the mission's start to its end;
  t_backed     when the later of the two robots' maps first decided the
               corridor right, whether or not the mission had ended by then;
  ack          the share or the report: acknowledged, failed, or sent without
               waiting; and for the partial grade, when the link returned
               relative to the end of r1's inspection, and whether the outage
               outlasted the acknowledgement's wait;
  distance     metres each robot drove;
  known        m^2 of the hall each robot's map holds at the end, own and merged.

The sums carry Wilson intervals for proportions and t intervals for means, at
95 %.
"""

import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grades as R  # noqa: E402
import grid as G  # noqa: E402

T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
       11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093,
       20: 2.086, 25: 2.060, 30: 2.042, 40: 2.021, 60: 2.000}


def t95(df):
    keys = sorted(T95)
    return T95[max(k for k in keys if k <= df)] if df >= 1 else float('nan')


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(c - h, 3), round(c + h, 3))


def mean_ci(xs):
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if not xs:
        return {'n': 0}
    m = float(np.mean(xs))
    if len(xs) < 2:
        return {'n': len(xs), 'mean': round(m, 2)}
    h = t95(len(xs) - 1) * float(np.std(xs, ddof=1)) / math.sqrt(len(xs))
    return {'n': len(xs), 'mean': round(m, 2), 'ci': [round(m - h, 2), round(m + h, 2)],
            'min': round(min(xs), 2), 'max': round(max(xs), 2)}


def rows(path):
    return list(csv.DictReader(open(path))) if os.path.exists(path) else []


def reachable():
    """The hall's cells a robot could ever see as free: the floor inside the
    outer walls, outside every wall, the space closed above the corridor excluded."""
    g = G.truth(False)
    seen = np.zeros_like(g)
    i, j = G.cell(0.0, -2.0)
    todo = [(j, i)]
    seen[j, i] = True
    while todo:
        j, i = todo.pop()
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = j + dj, i + di
            if 0 <= a < G.H and 0 <= b < G.W and not g[a, b] and not seen[a, b]:
                seen[a, b] = True
                todo.append((a, b))
    return seen


def run(d, floor):
    w = json.load(open(os.path.join(d, 'world.json')))
    la = json.load(open(os.path.join(d, 'launch.json')))
    m = json.load(open(os.path.join(d, 'mission.json')))
    truth = 'blocked' if w['blocked'] else 'clear'
    t0 = m.get('t0_sim', R.START_AT)
    end = t0 + m.get('t_mission_sim', 0.0)
    out = {'dir': os.path.basename(d), 'seed': w['seed'], 'blocked': w['blocked'], 'formulation': la['formulation'],
           'grade': la['grade'], 'result': m.get('result'), 't_mission': m.get('t_mission_sim'),
           'replanned': any(s['step'] == 'replan' for s in m.get('steps', [])), 'ack_wait': la['ack_wait']}
    # The maps.
    first, at_end = {}, {}
    for r in ('r1', 'r2'):
        ch = [x for x in rows(os.path.join(d, f'decision_{r}.csv')) if x['map'] == 'merged']
        right = [float(x['t_sim']) for x in ch if x['decision'] == truth]
        wrong = [float(x['t_sim']) for x in ch if x['decision'] not in ('', truth)]
        first[r] = right[0] - t0 if right else None
        out[f'wrong_{r}'] = len(wrong)
        before = [x for x in ch if float(x['t_sim']) <= end + 0.5]
        at_end[r] = before[-1]['decision'] if before else ''
        own = [x for x in rows(os.path.join(d, f'decision_{r}.csv')) if x['map'] == 'own' and x['decision'] == truth]
        out[f'first_own_{r}'] = round(float(own[0]['t_sim']) - t0, 2) if own else None
        out[f'first_{r}'] = round(first[r], 2) if first[r] is not None else None
        out[f'end_{r}'] = at_end[r]
        mp = os.path.join(d, 'maps', f'{r}.npz')
        if os.path.exists(mp):
            z = np.load(mp)
            k = int(np.searchsorted(z['t'], end + 0.5)) - 1
            k = max(0, min(k, len(z['t']) - 1))
            out[f'known_own_{r}'] = round(G.known_area(z['own'][k], floor), 2)
            out[f'known_merged_{r}'] = round(G.known_area(z['merged'][k], floor), 2)
        pose = [x for x in rows(os.path.join(d, 'pose.csv')) if x['robot'] == r and t0 <= float(x['t_sim']) <= end]
        out[f'distance_{r}'] = round(sum(math.dist((float(a['x']), float(a['y'])), (float(b['x']), float(b['y'])))
                                         for a, b in zip(pose, pose[1:])), 2)
    out['reported'] = out['result'] == 'SUCCESS'
    out['backed'] = out['reported'] and all(at_end[r] == truth for r in ('r1', 'r2'))
    out['unbacked'] = out['reported'] and not out['backed']
    out['t_backed'] = round(max(first.values()), 2) if all(v is not None for v in first.values()) else None
    out['floor_m2'] = round(float(floor.sum()) * G.RES * G.RES, 2)
    if la['formulation'] == 'epistemic':
        model = m.get('model') or {}
        out['model_kw_r2'] = model.get('(Kw r2 blocked)')
    # The message, and the link.
    acts = rows(os.path.join(d, 'actions.csv'))
    msg = [a for a in acts if a['action'] in ('share', 'report')]
    out['ack'] = ''
    if msg:
        last = [a for a in msg if a['status'] in ('SUCCEEDED', 'FAILED')]
        if last:
            out['ack'] = ('failed' if last[0]['status'] == 'FAILED' else
                          'acknowledged' if last[0]['message'] == 'acknowledged' else 'sent')
            start = [float(a['t_sim']) for a in msg if a['status'] == 'EXECUTING'][0]
            out['ack_seconds'] = round(float(last[0]['t_sim']) - start, 2)
    insp = [float(a['t_sim']) for a in acts if a['action'] == 'inspect' and a['arguments'] == 'r1'
            and a['status'] == 'SUCCEEDED']
    link = rows(os.path.join(d, 'link.csv'))
    downs = [float(x['t_declared']) for x in link if x['state'] == 'down']
    ups = [float(x['t_declared']) for x in link if x['state'] == 'up']
    out['link_down'] = round(downs[0] - t0, 2) if downs else None
    out['link_up'] = round(ups[0] - t0, 2) if ups else None
    if la['grade'] == 'partial' and insp and ups:
        out['link_back_after_inspection'] = round(ups[0] - insp[0], 2)
        out['outage_outlasted_wait'] = ups[0] - insp[0] > la['ack_wait']
    gate = rows(os.path.join(d, 'gate.csv'))
    out['gate_dropped'] = sum(1 for x in gate if x['event'] == 'dropped')
    out['gate_passed'] = sum(1 for x in gate if x['event'] == 'passed')
    return out


def summary(runs):
    out = {}
    for f in R.FORMULATIONS:
        for g in R.GRADES:
            rs = [r for r in runs if r['formulation'] == f and r['grade'] == g]
            if not rs:
                continue
            n = len(rs)
            k = {key: sum(1 for r in rs if r[key]) for key in ('reported', 'backed', 'unbacked', 'replanned')}
            out[f'{f}/{g}'] = {
                'n': n,
                **{key: {'k': v, 'wilson': wilson(v, n)} for key, v in k.items()},
                'results': {x: sum(1 for r in rs if r['result'] == x) for x in sorted({r['result'] for r in rs})},
                't_mission': mean_ci([r['t_mission'] for r in rs if r['reported']]),
                't_backed': mean_ci([r['t_backed'] for r in rs if r['t_backed'] is not None]),
                'distance_r2': mean_ci([r['distance_r2'] for r in rs]),
                'distance_total': mean_ci([r['distance_r1'] + r['distance_r2'] for r in rs]),
                'known_merged_r2': mean_ci([r.get('known_merged_r2') for r in rs]),
                'known_own_r2': mean_ci([r.get('known_own_r2') for r in rs]),
                'known_merged_r1': mean_ci([r.get('known_merged_r1') for r in rs]),
                'acks': {x: sum(1 for r in rs if r['ack'] == x) for x in sorted({r['ack'] for r in rs})},
                'wrong_decisions': sum(r['wrong_r1'] + r['wrong_r2'] for r in rs),
            }
    return out


def main():
    batch = sys.argv[1]
    floor = reachable()
    runs = []
    for name in sorted(os.listdir(batch)):
        d = os.path.join(batch, name)
        if os.path.isdir(d) and os.path.exists(os.path.join(d, 'mission.json')) and \
                os.path.exists(os.path.join(d, 'launch.json')):
            runs.append(run(d, floor))
    runs.sort(key=lambda r: (R.FORMULATIONS.index(r['formulation']), R.GRADES.index(r['grade']), r['seed']))
    keys = sorted({k for r in runs for k in r}, key=lambda k: (k not in runs[0], list(runs[0]).index(k)
                                                                if k in runs[0] else 0, k))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(runs)
    s = summary(runs)
    json.dump({'runs': len(runs), 'floor_m2': runs[0]['floor_m2'] if runs else None, 'ack_wait': R.ACK_WAIT,
               'cells': s}, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    for key, v in s.items():
        print(f"{key:20s} n={v['n']:2d} reported {v['reported']['k']:2d} backed {v['backed']['k']:2d} "
              f"unbacked {v['unbacked']['k']:2d} replanned {v['replanned']['k']:2d}  "
              f"t {v['t_mission'].get('mean', '-')}  t_backed {v['t_backed'].get('mean', '-')}  "
              f"r2 drove {v['distance_r2'].get('mean', '-')}  r2 knows {v['known_merged_r2'].get('mean', '-')} m2  "
              f"acks {v['acks']}")


if __name__ == '__main__':
    main()
