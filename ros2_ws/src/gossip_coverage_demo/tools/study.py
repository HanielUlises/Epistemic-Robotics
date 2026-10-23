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
The paper's simulation study (Section 6), in the kinematic simulator.

    study.py --out DIR [--envs 15] [--sizes 2 3 5 8] [--cases 1 2]
             [--methods ideal proposed flock epistemic] [--jobs 14]
    study.py --one ENV N CASE METHOD [--record FILE]

Fifteen areas, the same for every team size, case and method: environment e
has the obstacles of seed e, and the tasks of seed 1000 e + 10 N + case. Each
run is one line of DIR/runs.jsonl; a run already there is not run again.
"""

import argparse
import json
import multiprocessing as mp
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gossip import env as E  # noqa: E402
from gossip import sim as S  # noqa: E402


def area_for(spec, e, n, case):
    occ = E.generate(spec, e)
    reqs = E.case_requirements(case, n, 1000 * e + 10 * n + case)
    tasks = E.place_tasks(spec, occ, 1000 * e + 10 * n + case, reqs)
    return E.Area(spec, occ, tasks, seed=e)


def one(job):
    e, n, case, method = job
    spec = E.SIM
    area = area_for(spec, e, n, case)
    t = time.time()
    out = S.run(area, n, case, method, seed=1000 * e + 10 * n + case)
    out.update(env=e, wall=round(time.time() - t, 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--envs', type=int, default=15)
    ap.add_argument('--sizes', type=int, nargs='+', default=[2, 3, 5, 8])
    ap.add_argument('--cases', type=int, nargs='+', default=[1, 2])
    ap.add_argument('--methods', nargs='+', default=['ideal', 'proposed', 'flock'])
    ap.add_argument('--jobs', type=int, default=max(1, os.cpu_count() - 2))
    ap.add_argument('--one', nargs=4)
    ap.add_argument('--record')
    a = ap.parse_args()
    if a.one:
        e, n, case, method = int(a.one[0]), int(a.one[1]), int(a.one[2]), a.one[3]
        area = area_for(E.SIM, e, n, case)
        t = time.time()
        out = S.run(area, n, case, method, seed=1000 * e + 10 * n + case, record=bool(a.record))
        out.update(env=e, wall=round(time.time() - t, 1))
        if a.record:
            out['occ'] = area.occ.astype(int).tolist()
            json.dump(out, open(a.record, 'w'))
        ev = out.pop('events')
        out.pop('frames', None)
        out.pop('occ', None)
        print(json.dumps(out, indent=1))
        for x in ev:
            print(x)
        return
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, 'runs.jsonl')
    done = set()
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            done.add((r['env'], r['n'], r['case'], r['method']))
    jobs = [(e, n, c, m) for e in range(1, a.envs + 1) for n in a.sizes for c in a.cases for m in a.methods
            if (e, n, c, m) not in done]
    print(f'{len(jobs)} runs', flush=True)
    with mp.Pool(a.jobs) as pool, open(path, 'a') as f:
        for out in pool.imap_unordered(one, jobs):
            out.pop('events', None)
            f.write(json.dumps(out) + '\n')
            f.flush()
            print(out['method'], out['n'], out['case'], out['env'], out['mission'], out['wall'], flush=True)


if __name__ == '__main__':
    main()
