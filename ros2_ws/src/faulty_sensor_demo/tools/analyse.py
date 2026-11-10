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
                     the time from the first action's start to the last bed's
                     delivery (the mission's end, when the recorder was stopped
                     before the last drive's success reached it), the passages a bed-mover drove into and found
                     shut, what was read and said, whether the actions the
                     executor ran are the ones the policy has for the night,
                     and the time tools/plan.py and tools/schedule.py predict;
BATCH/analysis.json  per act and formulation, over the act's nights weighted by
                     their frequencies (tools/ranks.py): the expected time and
                     the probability that a bed-mover drives into a shut
                     passage, over all nights, over those with every laser
                     sound, with exactly one short-sighted, and with more;
                     predicted, and measured.

A policy takes the same path on every night on which its lasers read the
same: the same actions in the same order. The batch runs each path once, on
the first night that takes it, and a night's measured time is its path's.
"""

import collections
import csv
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402
import schedule as S  # noqa: E402

VOCAB = {}


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def path_of(act, f, night):
    """The policy's path on a night, as the vocabulary actions the executor runs."""
    pl, tree = P.plan(act, f)
    groups = P.trace(tree, night)
    m = S.mapping(act)
    return tuple(m[P.grounded(r, a, f)]['action'].strip('()') for g in groups for r, a in g)


def paths(act, f):
    """The distinct paths of a policy: {path: [nights]}, nights in the act's order."""
    out = {}
    for night in R.nights(act):
        out.setdefault(path_of(act, f, night), []).append(night)
    return out


def jobs(acts=None, forms=None):
    """The runs a batch needs: (act, formulation, seed), one per distinct path."""
    import worlds as W
    out = []
    for act in acts or sorted(R.FLEETS):
        for f in forms or R.FORMULATIONS:
            for nights in paths(act, f).values():
                out.append((act, f, W.seed_of(act, R.tag(nights[0]))))
    return out


def run_row(d):
    m = json.load(open(os.path.join(d, 'mission.json')))
    w = json.load(open(os.path.join(d, 'world.json')))
    act, f = m['act'], m['formulation']
    night = (tuple(w['loop']), frozenset(w['short']))
    acts = read(os.path.join(d, 'actions.csv')) if os.path.exists(os.path.join(d, 'actions.csv')) else []
    started = [float(a['t_sim']) for a in acts if a['status'] == 'EXECUTING']
    first = min(started) if started else 0.0
    done, wrong, readings, said = {}, [], [], []
    began, ended = collections.Counter(), collections.Counter()
    for a in acts:
        if a['action'] == 'INIT':
            continue
        if a['status'] == 'EXECUTING':
            began[f"{a['action']} {a['arguments']}"] += 1
        if a['status'] == 'SUCCEEDED':
            ended[f"{a['action']} {a['arguments']}"] += 1
        if a['status'] != 'SUCCEEDED':
            continue
        g = a['arguments'].split()
        if a['action'] == 'drive' and g[-1].startswith('bed'):
            done[g[0]] = round(float(a['t_sim']) - first, 1)
        if a['action'] == 'backout':
            wrong.append(f'{g[0]}:{g[1][-1]}')
        if a['action'] == 'look':
            readings.append(f"{g[0]}:{g[2]}={a['outcome'].rsplit('-', 1)[-1]}")
        if a['action'] == 'say':
            said.append(f'{g[0]}:{g[1][-1]}={g[2]}')
    # the recorder may be stopped before the last drive's success reaches it: that bed
    # was delivered when the mission ended
    for a in acts:
        g = a['arguments'].split()
        if a['status'] == 'EXECUTING' and a['action'] == 'drive' and g[-1].startswith('bed') and g[0] not in done \
                and m.get('arrived'):
            done[g[0]] = round(m['t0_sim'] + m['t_mission_sim'] - first, 1)
    pl, tree = P.plan(act, f)
    groups = P.trace(tree, night)
    predicted, pdone, pwrong = P.timeline(groups, pl.fleet)
    want = path_of(act, f, night)
    return {
        'run': os.path.basename(d), 'act': act, 'formulation': f, 'seed': w['seed'], 'night': w['tag'],
        'frequency': R.frequency(act, night), 'result': m.get('result'), 'arrived': m.get('arrived'),
        't_mission': max(done.values()) if done else 0.0, 'done': json.dumps(done),
        'wrong': ' '.join(wrong), 'n_wrong': len(wrong),
        'readings': ' '.join(readings), 'said': ' '.join(said),
        # an action ran if the recorder has its start or its success: it does not always get both
        'path_ok': (began | ended) == collections.Counter(want),
        'predicted': predicted, 'predicted_wrong': len(pwrong),
        'executor': S.timeline(S.schedule(tree, f, act), night)[0],
    }


def weighted(act, value, sel):
    """The expectation of value(night) over the act's nights that sel keeps."""
    ns = [n for n in R.nights(act) if sel(n)]
    z = sum(R.frequency(act, n) for n in ns)
    vals = [(R.frequency(act, n), value(n)) for n in ns]
    if not z or any(v is None for _, v in vals):
        return None
    return sum(fr * v for fr, v in vals) / z


SETS = {'all': lambda n: True, 'sound': lambda n: not n[1], 'one': lambda n: len(n[1]) == 1,
        'more': lambda n: len(n[1]) > 1}


def main():
    batch = sys.argv[1]
    sys.setrecursionlimit(10000)
    rows = []
    for d in sorted(glob.glob(os.path.join(batch, 'a*-s*'))):
        if os.path.exists(os.path.join(d, 'mission.json')) and os.path.exists(os.path.join(d, 'world.json')):
            try:
                rows.append(run_row(d))
            except Exception as e:  # noqa: BLE001
                print(f'{d}: {e!r}')
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
            pl, tree = P.plan(act, f)
            ps = paths(act, f)
            by_night, missing = {}, []
            for path, nights in ps.items():
                mine = [r for r in rs if R.night_of(act, r['night']) in nights and r['result'] == 'SUCCESS'
                        and r['arrived']]
                for n in nights:
                    if mine:
                        by_night[n] = {'t': sum(r['t_mission'] for r in mine) / len(mine),
                                       'wrong': max(r['n_wrong'] for r in mine), 'runs': len(mine),
                                       'path_ok': all(r['path_ok'] for r in mine)}
                    else:
                        missing.append(R.tag(n))
            pred = {n: P.timeline(P.trace(tree, n), pl.fleet) for n in R.nights(act)}
            entry = {'runs': len(rs), 'succeeded': sum(1 for r in rs if r['result'] == 'SUCCESS' and r['arrived']),
                     'paths': len(ps), 'missing': missing,
                     'path_ok': all(v['path_ok'] for v in by_night.values()),
                     'fleet_expects': pl.value, 'nights': {}}
            for n in R.nights(act):
                entry['nights'][R.tag(n)] = {
                    'frequency': R.frequency(act, n), 'predicted': pred[n][0], 'predicted_wrong': len(pred[n][2]),
                    'measured': by_night.get(n, {}).get('t'), 'wrong': by_night.get(n, {}).get('wrong')}
            for name, sel in SETS.items():
                entry[name] = {
                    'predicted': weighted(act, lambda n: pred[n][0], sel),
                    'predicted_wrong': weighted(act, lambda n: float(bool(pred[n][2])), sel),
                    'measured': weighted(act, lambda n: by_night.get(n, {}).get('t'), sel),
                    'wrong': weighted(act, lambda n: None if n not in by_night else float(bool(by_night[n]['wrong'])),
                                      sel)}
            out[act][f] = entry
    json.dump(out, open(os.path.join(batch, 'analysis.json'), 'w'), indent=1)

    def fmt(v, spec='.1f'):
        return '-' if v is None else format(v, spec)

    for act, fs in out.items():
        print(f'act {act}')
        for f, e in fs.items():
            print(f"  {f:10s} {e['succeeded']}/{e['runs']} runs of {e['paths']} paths; " +
                  '; '.join(f"{k}: {fmt(e[k]['measured'])} s ({fmt(e[k]['predicted'])}), "
                            f"shut passage {fmt(e[k]['wrong'], '.3f')} ({fmt(e[k]['predicted_wrong'], '.3f')})"
                            for k in SETS if e[k]['predicted'] is not None) +
                  ('' if e['path_ok'] else '  PATHS DIFFER') + (f"  missing {e['missing']}" if e['missing'] else ''))


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--jobs':
        sys.setrecursionlimit(10000)
        for act, f, seed in jobs([int(x) for x in sys.argv[2:]] or None):
            print(act, seed, f)
    else:
        main()
