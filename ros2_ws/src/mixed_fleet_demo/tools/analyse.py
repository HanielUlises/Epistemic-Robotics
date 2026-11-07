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
                     the mission's time from the robots' setting out, each
                     robot's own time, the doors looked at and what was found,
                     the reports of evidence and of belief, the distance
                     driven, and the time tools/plan.py predicted for the same
                     policy and night;
BATCH/analysis.json  per act and formulation: the mean time of each night, the
                     expectation over the nights under the porter's
                     frequencies (tools/ranks.py), the summed robots' time, how
                     many runs arrived, and the predicted figures: by the
                     planner's own model of the groups, and by ePlanSys's
                     schedule (tools/schedule.py). Where two
                     formulations plan the same path on a night, the same
                     actions in the same order, their runs are draws of one
                     process: 'pooled' is the mean over all of them, and
                     'expected_pooled' the expectation over the pooled means,
                     so that a difference between formulations comes only from
                     the nights where what they do differs.

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
import schedule as S  # noqa: E402


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


_TREES = {}
_SCHED = {}


def team_tree(act, f):
    if (act, f) not in _TREES:
        _TREES[(act, f)] = P.plan(act, f)
    return _TREES[(act, f)]


def executor_time(act, f, world):
    """The time the policy takes on a night when its groups run as ePlanSys schedules
    them (tools/schedule.py), with the declared durations."""
    if (act, f) not in _SCHED:
        team, tree = team_tree(act, f)
        _SCHED[(act, f)] = S.schedule(tree, team.fleet)
    return S.timeline(_SCHED[(act, f)], world)[0]


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
            if y.startswith('bed'):
                done[r] = round(float(a['t_sim']) - first, 1)
    looks = [(a['arguments'].split()[0], a['arguments'].split()[-1][-2], a['message'])
             for a in acts if a['action'] == 'look' and a['status'] == 'SUCCEEDED']
    tells = [a['arguments'] for a in acts if a['action'] == 'tell' and a['status'] == 'SUCCEEDED']
    beliefs = [a['arguments'] for a in acts if a['action'] == 'believe' and a['status'] == 'SUCCEEDED']
    driven, last = {}, {}
    poses = read(os.path.join(d, 'pose.csv')) if os.path.exists(os.path.join(d, 'pose.csv')) else []
    for p in poses:
        r = p['robot']
        if r == 'p' or float(p['t_sim']) < first:
            continue
        xy = (float(p['x']), float(p['y']))
        if r in last:
            driven[r] = driven.get(r, 0.0) + math.dist(last[r], xy)
        last[r] = xy
    world = R.worlds().index(tuple(w['loop']))
    team, tree = team_tree(act, f)
    predicted, pdone = P.timeline(P.paths(tree, world), team.fleet)
    t_mission = round((max(ended) - first) if ended else 0.0, 1)
    return {
        'run': os.path.basename(d), 'act': act, 'formulation': f, 'seed': w['seed'], 'world': w['tag'],
        'frequency': R.frequency(tuple(w['loop'])), 'result': m.get('result'), 'arrived': m.get('arrived'),
        't_mission': t_mission, 't_robots': round(sum(done.values()), 1),
        'done': json.dumps(done), 'looks': ' '.join(f'{r}:{v}={o}' for r, v, o in looks),
        'shut_found': sum(1 for _, _, o in looks if o == 'shut'), 'tells': len(tells),
        'beliefs': ' | '.join(beliefs), 'n_beliefs': len(beliefs),
        'driven_m': round(sum(driven.values()), 1),
        'predicted': predicted, 'predicted_done': json.dumps(pdone),
        'executor': executor_time(act, f, world),
    }


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
    # the formulations whose paths coincide on a night, per act
    same = {}
    for act in sorted({r['act'] for r in rows}):
        for k, w in enumerate(R.worlds()):
            sig = {}
            for f in R.FORMULATIONS + R.ALSO:
                team, tree = team_tree(act, f)
                sig.setdefault(tuple(P.grounded(r, a) for r, a in P.paths(tree, k)), []).append(f)
            for fs in sig.values():
                for f in fs:
                    same[(act, R.name(w), f)] = fs
    for act in sorted({r['act'] for r in rows}):
        out[act] = {}
        for f in R.FORMULATIONS + R.ALSO:
            rs = [r for r in rows if r['act'] == act and r['formulation'] == f]
            if not rs:
                continue
            per = {}
            for r in rs:
                per.setdefault(r['world'], []).append(r)
            nights = {}
            for k, v in per.items():
                ok = [x for x in v if x['arrived']]
                if not ok:
                    continue
                nights[k] = {'mean': round(statistics.mean(x['t_mission'] for x in ok), 1),
                             'sd': round(statistics.stdev(x['t_mission'] for x in ok), 1) if len(ok) > 1 else 0.0,
                             'robots': round(statistics.mean(x['t_robots'] for x in ok), 1),
                             'runs': len(v), 'arrived': len(ok),
                             'predicted': v[0]['predicted'], 'executor': v[0]['executor'],
                             'frequency': v[0]['frequency']}
            for k, n in nights.items():
                pool = [x for x in rows if x['act'] == act and x['world'] == k and x['arrived']
                        and x['formulation'] in same[(act, k, f)]]
                n['pooled'] = round(statistics.mean(x['t_mission'] for x in pool), 1)
                n['pooled_runs'] = len(pool)
                n['same_as'] = [g for g in same[(act, k, f)] if g != f]
            covered = sum(n['frequency'] for n in nights.values())
            expp = sum(n['frequency'] * n['pooled'] for n in nights.values()) / covered if covered else None
            exp = sum(n['frequency'] * n['mean'] for n in nights.values()) / covered if covered else None
            expr = sum(n['frequency'] * n['robots'] for n in nights.values()) / covered if covered else None
            team, tree = team_tree(act, f)
            expx = sum(R.frequency(w) * executor_time(act, f, k) for k, w in enumerate(R.worlds()))
            out[act][f] = {'nights': nights, 'expected': round(exp, 1) if exp else None,
                           'expected_executor': round(expx, 1),
                           'expected_pooled': round(expp, 1) if expp else None,
                           'expected_robots': round(expr, 1) if expr else None,
                           'expected_predicted': round(P.expected(team, tree), 1),
                           'covered_frequency': round(covered, 4),
                           'runs': len(rs), 'arrived': sum(bool(r['arrived']) for r in rs),
                           'results': {k: sum(r['result'] == k for r in rs) for k in {r['result'] for r in rs}}}
            print(f"act {act} {f:12s}: {len(rs)} runs, {out[act][f]['arrived']} arrived, expected "
                  f"{out[act][f]['expected']} s, pooled {out[act][f]['expected_pooled']} s "
                  f"(predicted {out[act][f]['expected_predicted']})  " +
                  ' '.join(f"{k}:{v['mean']:.0f}/{v['predicted']:.0f}" for k, v in sorted(nights.items())))
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
