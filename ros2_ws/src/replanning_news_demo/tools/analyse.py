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
The batch of replanning on news, measured.

    analyse.py BATCH        writes BATCH/runs.csv and BATCH/analysis.json

A run is judged by where the courier ended, by its own odometry: within 0.5 m
of the resident's bedside, and the executor's result. Its time is simulated
seconds from the mission's start to its end. Per run also: the distance the
courier drove; the doors it found shut by driving into them (the classical
formulation only can); how long the news took to stop the running action and
to bring a new policy; the planning times; and what the announcement left.

Runs are paired by act and seed: the four formulations of a pair start from
the same world and the same poses. The paired differences are summarised with
their mean, a 95 % t interval and the Wilcoxon signed-rank test.
"""

import csv
import json
import math
import os
import sys

import numpy as np
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import model as M  # noqa: E402

FORMS = ('replan', 'null', 'keep', 'classical')


def driven(path):
    pts = []
    try:
        for r in csv.DictReader(open(path)):
            if r['robot'] == 'c':
                pts.append((float(r['x']), float(r['y'])))
    except OSError:
        return None
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def one(run):
    m = json.load(open(os.path.join(run, 'mission.json')))
    launch = json.load(open(os.path.join(run, 'launch.json')))
    try:
        news = json.load(open(os.path.join(run, 'news.json')))
    except OSError:
        news = {}
    plans = m.get('plans', [])
    steps = m.get('steps', [])
    row = {'run': os.path.basename(run), 'act': launch['act'], 'seed': launch['seed'], 'world': launch['tag'],
           'formulation': launch['formulation'], 'result': m.get('result'), 'delivered': bool(m.get('delivered')),
           't_mission': m.get('t_mission_sim'), 'driven_m': driven(os.path.join(run, 'pose.csv')),
           'plans': len(plans), 'plan_s_first': plans[0]['seconds'] if plans else None,
           'plan_s_again': plans[1]['seconds'] if len(plans) > 1 else None,
           'shut_met': len(m.get('found_shut', [])), 'news_at': news.get('t_mission'),
           'worlds_after': (news.get('answer') or {}).get('worlds') if isinstance(news.get('answer'), dict) else None}
    # the news's effect on the epistemic formulations: the first policy's end, and the next policy ready
    if row['formulation'] in ('replan', 'null') and len(plans) > 1 and row['news_at'] is not None:
        row['stopped_after'] = round(steps[0]['t_end'] - row['news_at'], 2) if steps else None
        row['replanned_after'] = round(plans[1]['t_ready'] - row['news_at'], 2)
    if row['formulation'] == 'classical':
        row['news_replan'] = 'news_replan' in m
        row['doors_tried'] = ''.join(c['door'] for c in m.get('chosen', []))
    looks = tells = 0
    try:
        for r in csv.DictReader(open(os.path.join(run, 'actions.csv'))):
            if r['status'] == 'SUCCEEDED':
                looks += r['action'].startswith('look_')
                tells += r['action'].startswith('tell_')
    except OSError:
        pass
    row['looks'], row['tells'] = looks, tells
    return row


def summary(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {}
    a = np.array(xs, float)
    return {'n': len(a), 'mean': round(float(a.mean()), 2), 'sd': round(float(a.std(ddof=1)), 2) if len(a) > 1 else 0.0,
            'min': round(float(a.min()), 2), 'max': round(float(a.max()), 2), 'median': round(float(np.median(a)), 2)}


def paired(rows, a, b, key='t_mission'):
    """b minus a, paired by act and seed, over runs both delivered."""
    by = {}
    for r in rows:
        by.setdefault((r['act'], r['seed']), {})[r['formulation']] = r
    diffs = []
    for k, f in sorted(by.items()):
        if a in f and b in f and f[a]['delivered'] and f[b]['delivered']:
            diffs.append(f[b][key] - f[a][key])
    if not diffs:
        return {}
    d = np.array(diffs)
    out = {'n': len(d), 'mean': round(float(d.mean()), 2), 'min': round(float(d.min()), 2),
           'max': round(float(d.max()), 2), 'positive': int((d > 0.5).sum()), 'zero': int((abs(d) <= 0.5).sum()),
           'negative': int((d < -0.5).sum())}
    if len(d) > 1:
        half = stats.t.ppf(0.975, len(d) - 1) * d.std(ddof=1) / math.sqrt(len(d))
        out['ci95'] = [round(float(d.mean() - half), 2), round(float(d.mean() + half), 2)]
    if len(d) > 1 and np.any(d != 0):
        out['wilcoxon_p'] = float(stats.wilcoxon(d).pvalue)
    return out


def main():
    batch = sys.argv[1]
    rows = []
    for name in sorted(os.listdir(batch)):
        run = os.path.join(batch, name)
        if os.path.isdir(run) and os.path.exists(os.path.join(run, 'mission.json')) and \
                os.path.exists(os.path.join(run, 'launch.json')):
            rows.append(one(run))
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)

    out = {'runs': len(rows), 'per_act': {}}
    for act in (1, 2, 3):
        ar = [r for r in rows if r['act'] == act]
        if not ar:
            continue
        a = {'robots': list(M.ROBOTS[act]), 'news': M.NEWS_TEXT[act], 'kind': M.NEWS_KIND[act],
             'seeds': len({r['seed'] for r in ar}), 'formulations': {}, 'per_world': {}, 'paired': {}}
        for f in FORMS:
            fr = [r for r in ar if r['formulation'] == f]
            if not fr:
                continue
            a['formulations'][f] = {
                'n': len(fr), 'delivered': sum(r['delivered'] for r in fr),
                'results': {k: sum(r['result'] == k for r in fr) for k in {r['result'] for r in fr}},
                't_mission': summary([r['t_mission'] for r in fr if r['delivered']]),
                'driven_m': summary([r['driven_m'] for r in fr if r['delivered']]),
                'plans': summary([r['plans'] for r in fr]), 'shut_met': sum(r['shut_met'] for r in fr),
                'runs_meeting_shut': sum(r['shut_met'] > 0 for r in fr),
                'looks': summary([r['looks'] for r in fr]), 'tells': summary([r['tells'] for r in fr]),
                'plan_s_first': summary([r['plan_s_first'] for r in fr]),
                'plan_s_again': summary([r.get('plan_s_again') for r in fr]),
                'stopped_after': summary([r.get('stopped_after') for r in fr]),
                'replanned_after': summary([r.get('replanned_after') for r in fr]),
            }
        for world in sorted({r['world'] for r in ar}):
            wr = [r for r in ar if r['world'] == world]
            a['per_world'][world] = {f: summary([r['t_mission'] for r in wr if r['formulation'] == f and r['delivered']])
                                     for f in FORMS}
            a['per_world'][world]['shut_met'] = {f: sum(r['shut_met'] for r in wr if r['formulation'] == f)
                                                for f in FORMS}
        for x, y in (('replan', 'keep'), ('replan', 'null'), ('null', 'keep'), ('replan', 'classical')):
            a['paired'][f'{y}-{x}'] = paired(ar, x, y)
            a['paired'][f'{y}-{x}_driven'] = paired(ar, x, y, 'driven_m')
        out['per_act'][str(act)] = a

    # the closing card's numbers
    film = {'headline': f'{len(rows)} runs: {sum(len({r["seed"] for r in rows if r["act"] == a}) for a in (1, 2, 3))} '
                        f'seeds, each under all four formulations', 'per_act': {}, 'closing_lines': []}
    for act, a in out['per_act'].items():
        film['per_act'][act] = {'kind': a['kind'], 'formulations': {
            f: {'n': s['n'], 'delivered': s['delivered'], 't_mean': s['t_mission'].get('mean', float('nan')),
                'dead_ends': s['shut_met']} for f, s in a['formulations'].items()}}
    for act, a in out['per_act'].items():
        p = a['paired'].get('keep-replan', {})
        q = a['paired'].get('null-replan', {})
        if p and q:
            film['closing_lines'].append(
                f'act {act}: the news saved {p["mean"]:.1f} s against the policy kept, faster in {p["positive"]} '
                f'of {p["n"]} pairs; {q["mean"]:.1f} s against a tautology announced instead.')
    out['film'] = film
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    for act, a in out['per_act'].items():
        print(f'act {act} ({a["kind"]}), {a["seeds"]} seeds')
        for f, s in a['formulations'].items():
            t = s['t_mission']
            print(f'  {f:9s} delivered {s["delivered"]}/{s["n"]}  t {t.get("mean")} s (sd {t.get("sd")}, '
                  f'max {t.get("max")})  driven {s["driven_m"].get("mean")} m  shut met {s["shut_met"]}  '
                  f'plans {s["plans"].get("mean")}')
        for k in ('keep-replan', 'null-replan', 'keep-null', 'classical-replan'):
            p = a['paired'].get(k)
            if p:
                print(f'  {k:17s} mean {p["mean"]} s  CI {p.get("ci95")}  +{p["positive"]} ={p["zero"]} '
                      f'-{p["negative"]}  p {p.get("wilcoxon_p")}')


if __name__ == '__main__':
    main()
