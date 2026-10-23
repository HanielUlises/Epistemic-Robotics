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
The results of a batch, Gazebo's (batch.sh) or the kinematic study's
(study.py).

    analyse.py gazebo BATCH        writes BATCH/runs.csv and BATCH/analysis.json
    analyse.py study STUDY         writes STUDY/analysis.json

A run is judged by its mission time: when every task is done and every robot
is home (and, in Case II, the whole area has been seen), by the simulator's
clock, the planner's wall time included. Pairs are the same seed (Gazebo) or
the same environment, team and case (the study) under two formulations.
"""

import csv
import glob
import json
import os
import re
import statistics as st
import sys

FORMS = ('epistemic', 'proposed', 'ideal', 'flock')


def describe(v):
    v = [x for x in v if x is not None]
    if not v:
        return None
    return {'n': len(v), 'mean': round(st.mean(v), 1), 'median': round(st.median(v), 1),
            'sd': round(st.pstdev(v), 1), 'min': round(min(v), 1), 'max': round(max(v), 1)}


def paired(a, b):
    d = [x - y for x, y in zip(a, b) if x is not None and y is not None]
    if not d:
        return None
    return {'n': len(d), 'mean': round(st.mean(d), 1), 'median': round(st.median(d), 1),
            'wins': sum(x < -0.5 for x in d), 'ties': sum(abs(x) <= 0.5 for x in d), 'losses': sum(x > 0.5 for x in d),
            'min': round(min(d), 1), 'max': round(max(d), 1)}


def en_route(d, text):
    """Whether, when the first policy of a run was ended by global contact,
    every robot was already on its way to the task's site: the frame nearest
    the executor's cancellation, by wall time."""
    m = re.search(r'\[(\d+\.\d+)\] \[[^]]*\]: Received request to cancel goal', text)
    p = os.path.join(d, 'frames.jsonl')
    if not m or not os.path.exists(p):
        return None
    w = float(m.group(1))
    best = None
    for line in open(p):
        f = json.loads(line)
        if best is None or abs(f['wall'] - w) < abs(best['wall'] - w):
            best = f
    return all(v[2] == 'task' for v in best['robots'].values())


def gazebo(batch):
    rows = []
    for d in sorted(glob.glob(os.path.join(batch, 'c*-*-s*'))):
        p = os.path.join(d, 'mission.json')
        if not os.path.exists(p):
            continue
        m = json.load(open(p))
        w = json.load(open(os.path.join(d, 'world.json')))
        name = os.path.basename(d)
        case, form, seed = int(name[1]), name.split('-')[1], int(name.split('-s')[-1])
        tasks = m.get('tasks') or {}
        pol = m.get('policies') or []
        log = os.path.join(d, 'launch.log')
        text = open(log, errors='ignore').read() if os.path.exists(log) else ''
        # a policy still running when all robots come into contact is ended
        # there by the mission (the global allocation takes over)
        cancelled = text.count('Received request to cancel goal')
        succeeded = sum(p.get('result') == 'SUCCESS' for p in pol)
        rows.append({'case': case, 'formulation': form, 'seed': seed, 'result': m.get('result'),
                     'mission': m.get('mission'),
                     'tasks_done': max((t['done_t'] or 0) for t in tasks.values()) if tasks else None,
                     'found': ';'.join(f"{k}:{t['k']}:{t['found_t']}:{t['found_by']}:{t['done_t']}" for k, t in tasks.items()),
                     'policies': len(pol), 'plan_seconds': round(sum(p.get('seconds', 0) for p in pol), 3),
                     'policy_results': ';'.join(p.get('result', '') for p in pol),
                     'policies_completed': succeeded, 'policies_handed_over': cancelled,
                     'handed_over_en_route': int(bool(cancelled and en_route(d, text))),
                     'absent': (m.get('counts') or {}).get('absent', 0),
                     'met': (m.get('counts') or {}).get('met', 0),
                     'travelled': round(sum((m.get('travelled') or {}).values()), 1),
                     'wall': round(m['t1_wall'] - m['t0_wall'], 1) if m.get('t1_wall') and m.get('t0_wall') else None,
                     'draw': w.get('draw')})
    with open(os.path.join(batch, 'runs.csv'), 'w', newline='') as f:
        if rows:
            wr = csv.DictWriter(f, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
    out = {'runs': len(rows), 'success': sum(r['result'] == 'SUCCESS' for r in rows), 'cases': {}}
    for case in (1, 2):
        c = {'by_formulation': {}, 'paired': {}}
        seeds = sorted({r['seed'] for r in rows if r['case'] == case})
        get = {(r['formulation'], r['seed']): r for r in rows if r['case'] == case}
        for form in FORMS:
            v = [get[(form, s)]['mission'] for s in seeds if (form, s) in get]
            c['by_formulation'][form] = dict(describe(v) or {}, runs=sum((form, s) in get for s in seeds), success=sum(
                get[(form, s)]['result'] == 'SUCCESS' for s in seeds if (form, s) in get),
                tasks_done=describe([get[(form, s)]['tasks_done'] for s in seeds if (form, s) in get]))
        for other in ('proposed', 'ideal', 'flock'):
            both = [s for s in seeds if ('epistemic', s) in get and (other, s) in get]
            c['paired'][f'epistemic-{other}'] = paired([get[('epistemic', s)]['mission'] for s in both],
                                                       [get[(other, s)]['mission'] for s in both])
        pl = [r['plan_seconds'] for r in rows if r['case'] == case and r['formulation'] == 'epistemic' and r['policies']]
        c['planning'] = describe(pl)
        c['policies'] = sum(r['policies'] for r in rows if r['case'] == case and r['formulation'] == 'epistemic')
        c['policies_completed'] = sum(r['policies_completed'] for r in rows
                                      if r['case'] == case and r['formulation'] == 'epistemic')
        c['policies_handed_over'] = sum(r['policies_handed_over'] for r in rows
                                        if r['case'] == case and r['formulation'] == 'epistemic')
        c['handed_over_en_route'] = sum(r['handed_over_en_route'] for r in rows
                                        if r['case'] == case and r['formulation'] == 'epistemic')
        out['cases'][case] = c
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)
    return out


def study(path):
    rows = [json.loads(line) for line in open(os.path.join(path, 'runs.jsonl'))]
    out = {'runs': len(rows), 'unfinished': sum(r['mission'] is None for r in rows), 'cells': {}, 'paper': {}}
    get = {(r['case'], r['n'], r['env'], r['method']): r for r in rows}
    envs = sorted({r['env'] for r in rows})
    for case in (1, 2, 3):
        for n in (2, 3, 5, 8):
            cell = {}
            for form in FORMS:
                cell[form] = describe([get[(case, n, e, form)]['mission'] for e in envs if (case, n, e, form) in get])
            cell['epistemic-proposed'] = paired([get[(case, n, e, 'epistemic')]['mission'] for e in envs],
                                                [get[(case, n, e, 'proposed')]['mission'] for e in envs])
            out['cells'][f'{case}-{n}'] = cell
        means = {f: st.mean([r['mission'] for r in rows if r['case'] == case and r['method'] == f]) for f in FORMS}
        sds = {f: st.mean([st.pstdev([r['mission'] for r in rows if r['case'] == case and r['method'] == f
                                      and r['n'] == n]) for n in (2, 3, 5, 8)]) for f in FORMS}
        out['paper'][case] = {'mean': {f: round(v, 1) for f, v in means.items()},
                              'proposed_minus_ideal': round(means['proposed'] - means['ideal'], 1),
                              'flock_minus_proposed': round(means['flock'] - means['proposed'], 1),
                              'sd': {f: round(v, 1) for f, v in sds.items()}}
    counts = {}
    for r in rows:
        for k, v in r['counts'].items():
            counts.setdefault(r['method'], {}).setdefault(k, 0)
            counts[r['method']][k] += v
    out['counts'] = counts
    pl = [p for r in rows if r['method'] == 'epistemic' and r.get('planning') for p in r['planning']]
    secs = [p['seconds'] for p in pl]
    out['planning'] = {'policies': len(pl), 'solved': sum(p['solved'] for p in pl),
                       'seconds': {'mean': round(st.mean(secs), 4), 'median': round(st.median(secs), 4),
                                   'max': round(max(secs), 3)} if secs else None,
                       'agents': describe([p['agents'] for p in pl])}
    json.dump(out, open(os.path.join(path, 'analysis.json'), 'w'), indent=1)
    return out


def main():
    kind, path = sys.argv[1], sys.argv[2]
    out = gazebo(path) if kind == 'gazebo' else study(path)
    print(json.dumps(out, indent=1)[:4000])


if __name__ == '__main__':
    main()
