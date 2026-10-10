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
The communication study: every strategy on the same instances, swept.

    run_study.py --out /tmp/study --seeds 200 --workers 8
    run_study.py --out /tmp/study --seeds 20 --regimes budget      # a quick look

One CSV row per instance, strategy, merge rule and swept value, with the seed,
so that every comparison can be paired. The epistemic strategy is solved once
per instance; its messages are then replayed on the maps, and the shift must
succeed there as it does in the model, which checks the planner against the
same simulator the protocols run in.

Cells, each with --seeds instances, drawn until communication is needed (the
shift fails with no message; the instances it skipped are counted):

  budget       8 and 16 robots, 3 bays, every pair linked or a sparse radio
               graph; budgets from 0 to unlimited
  secrecy      8 and 12 robots; unlimited and a few budgets
  joint        8 robots, one and two pairs; unlimited and a few budgets
  elimination  8 robots, 4 and 6 bays; budgets; protocols with and without
               inference by elimination
  skew         8 robots, 3 bays; clock deviation 0 to 40 s with 10 s between
               the two changes of one bay; unlimited budget
"""

import argparse
import csv
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import epistemic as E  # noqa: E402
import instances as I  # noqa: E402
import protocols as P  # noqa: E402

BUDGETS = [0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512, None]
FEW = [2, 4, 8, 16, 32, 64, None]
SIGMAS = [0.0, 2.5, 5.0, 10.0, 20.0, 40.0]

FIELDS = ['regime', 'cell', 'seed', 'robots', 'bays', 'haulers', 'links', 'contractors', 'pairs',
          'strategy', 'rule', 'infer', 'budget', 'sigma', 'success', 'reason', 'messages', 'stale_after',
          'overwrote_fresh', 'plan_seconds', 'plan_messages_raw', 'inferred_sends', 'skipped_before']


class Make:
    """A picklable instance builder: the instances module's function, and its
    arguments other than the seed."""

    def __init__(self, fn, *args, **kwargs):
        self.fn, self.args, self.kwargs = fn, args, kwargs

    def __call__(self, seed):
        return getattr(I, self.fn)(*self.args, seed, **self.kwargs)


def cells(regimes):
    out = []
    if 'budget' in regimes:
        for n in (8, 16):
            out.append(('budget', f'n{n}-full', Make('budget', n, 3), BUDGETS, [None]))
            out.append(('budget', f'n{n}-radio', Make('budget', n, 3, p_link=0.3), BUDGETS, [None]))
    if 'secrecy' in regimes:
        for n in (8, 12):
            out.append(('secrecy', f'n{n}', Make('secrecy', n, 3), FEW, [None]))
        out.append(('secrecy', 'n8-hauls', Make('secrecy', 8, 3, hauling=True), FEW, [None]))
    if 'joint' in regimes:
        for k in (1, 2):
            out.append(('joint', f'pairs{k}', Make('joint', 8, 3, pairs=k), FEW, [None]))
    if 'elimination' in regimes:
        for b in (4, 6):
            out.append(('elimination', f'b{b}', Make('elimination', 8, b), BUDGETS, [None]))
    if 'skew' in regimes:
        out.append(('skew', 'n8', Make('skew', 8, 3, sigma=1.0), [None], SIGMAS))
    return out


def plan_maps(inst, rep):
    """The plan's messages replayed on the maps: each report sets the
    receiver's reading to what the sender believes, as fresh as anything the
    sender has seen. Returns the maps and how many reports carried a belief
    the sender had no reading of, inferred and not observed."""
    maps = P.initial_maps(inst)
    inferred = 0
    for i, j, t, kind in rep['messages']:
        value = 'o' if kind in ('send-open', 'tell-open') else 'x'
        if maps[i].get(t) is None:
            inferred += 1
        top = max([r.version for m in maps.values() for r in m.values()] + [0])
        stamp = max([r.stamp for m in maps.values() for r in m.values()] + [0.0])
        maps[j][t] = P.Reading(value, stamp + 1.0, top + 1, i)
    return maps, inferred


def scaled(inst, sigma):
    """The same instance with every clock offset scaled to deviation sigma:
    the offsets were drawn at deviation 1, so the instances are paired
    across sigma."""
    inst.clock_offset = {a: o * sigma for a, o in inst.clock_offset.items()}
    return inst


def work(job):
    regime, cell, make, budgets, sigmas, seed, skipped, out, timeout = job
    rows = []
    inst = make(seed)
    base = dict(regime=regime, cell=cell, seed=seed, robots=len(inst.agents), bays=len(inst.bays),
                haulers=len(inst.haulers), links=len(inst.links) if inst.links is not None else -1,
                contractors=len(inst.contractors), pairs=len(inst.pairs), skipped_before=skipped)
    rep = E.solve(inst, os.path.join(out, 'tasks', regime, cell, str(seed)), timeout=timeout, threads=1)
    plan_msgs = len(rep.get('messages', [])) if rep.get('solved') and rep.get('goal') else None
    inferred = 0
    if plan_msgs is not None:
        maps, inferred = plan_maps(inst, rep)
        ok, why = P.outcome(inst, maps, crossed=rep['crossed'])
        if not ok:
            raise RuntimeError(f'{regime} {cell} {seed}: the plan succeeds in the model and not on the maps: {why}')
    for sigma in sigmas:
        if sigma is not None:
            inst = scaled(make(seed), sigma)
        for budget in budgets:
            ok = plan_msgs is not None and (budget is None or plan_msgs <= budget)
            reason = '' if ok else (rep.get('error', 'no policy') if plan_msgs is None else 'over budget')
            rows.append(dict(base, strategy='plan', rule='update', infer=1, budget=budget, sigma=sigma,
                             success=int(ok), reason=reason, messages=plan_msgs if ok else '',
                             stale_after='', overwrote_fresh=0, plan_seconds=round(rep.get('seconds', 0.0), 3),
                             plan_messages_raw=rep.get('messages_before_elimination', ''),
                             inferred_sends=inferred))
            for proto in ('flood', 'pull', 'gossip'):
                rules = ('recency', 'version') if budget is not None else P.RULES
                for rule in rules:
                    for infer in ((1, 0) if regime == 'elimination' else (1,)):
                        run = P.PROTOCOLS[proto](inst, rule, budget, seed=seed)
                        good, why = P.outcome(inst, run.maps, infer=bool(infer), rule=rule)
                        rows.append(dict(base, strategy=proto, rule=rule, infer=infer, budget=budget,
                                         sigma=sigma, success=int(good), reason=why, messages=run.sent,
                                         stale_after=P.stale_entries(inst, run.maps),
                                         overwrote_fresh=run.overwrote_fresh, plan_seconds='',
                                         plan_messages_raw='', inferred_sends=''))
    return rows


def jobs_for(regimes, n_seeds, out, timeout):
    jobs = []
    for regime, cell, make, budgets, sigmas in cells(regimes):
        seed, taken, skipped = 1, 0, 0
        while taken < n_seeds:
            inst = make(seed)
            if P.needs_communication(inst):
                jobs.append((regime, cell, make, budgets, sigmas, seed, skipped, out, timeout))
                taken += 1
                skipped = 0
            else:
                skipped += 1
            seed += 1
    return jobs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seeds', type=int, default=200)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--timeout', type=int, default=120)
    ap.add_argument('--regimes', nargs='*', default=['budget', 'secrecy', 'joint', 'elimination', 'skew'])
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    jobs = jobs_for(args.regimes, args.seeds, args.out, args.timeout)
    print(f'{len(jobs)} instances', flush=True)
    path = os.path.join(args.out, 'study.csv')
    started = time.time()
    with open(path, 'w', newline='') as fh, mp.get_context('fork').Pool(args.workers) as pool:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for k, rows in enumerate(pool.imap_unordered(work, jobs, chunksize=1), start=1):
            w.writerows(rows)
            if k % 50 == 0:
                fh.flush()
                print(f'{k}/{len(jobs)} instances, {time.time() - started:.0f} s', flush=True)
    print(path)


if __name__ == '__main__':
    main()
