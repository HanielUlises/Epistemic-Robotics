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
The coordinated attack measured as it grows: robots on the beacon floor, and
robots and message levels on the radio floor.

    scaling.py --out DIR [--budget 300] [--jobs 8]

Every cell is written by tools/scaled.py, grounded by plank and solved by
Aletheia at one thread, once with AO* and once with replanning over the
all-outcomes determinization, each with the whole budget. On the beacon
floor every policy found is replayed by trace.py, which has to report the
goal holding at every leaf, so every lift in it under C over all n robots.
On the radio floor there is no policy to find, and what is measured is the
price of the proof: AO* proves it by exhausting the space, replanning by
refuting the determinization, which is sound for unsolvability, since a
policy of the task would be one of its determinization.

The planner's own budget is not always kept: a replanning run on a large
radio floor was seen at twice its deadline, inside a step that does not
check the clock. A run is killed at the budget plus a grace, and recorded so.

Writes DIR/scaling.csv and prints the two tables in the README's form.
Expansions on a run that ends without a policy are printed only by a
planner that logs them there; with one that does not, the column is empty.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scaled  # noqa: E402

EPDDL = os.path.join(HERE, '..', 'epddl')
HOME = os.path.expanduser('~')
PLANK = os.environ.get('PLANK', os.path.join(HOME, 'plank', 'build', 'plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.join(
    HOME, 'eplansys_ws', 'install', 'aletheia', 'bin', 'epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.join(
    HOME, 'plank', 'benchmarks', 'libraries', 'intermediate.epddl'))

# The cells. The beacon floor's cost hardly moves with the message levels,
# which it never uses, so it is measured at four; the radio floor's grows
# with both, and each row stops where the last cell ran out of budget.
BEACON = [(n, 4) for n in range(2, 11)]
RADIO = [(2, m) for m in range(1, 7)] + [(3, m) for m in range(1, 4)] + \
        [(4, 1), (4, 2), (5, 1)]
STRATEGIES = ('aostar', 'replan')


def ground(out, n, m, floor):
    d = os.path.join(out, f'n{n}-m{m}')
    os.makedirs(d, exist_ok=True)
    domain = os.path.join(d, scaled.domain_name(m) + '.epddl')
    with open(domain, 'w') as fh:
        fh.write(scaled.domain_text(m))
    name = scaled.problem_name(n, m, floor)
    problem = os.path.join(d, name + '.epddl')
    with open(problem, 'w') as fh:
        fh.write(scaled.problem_text(n, m, floor))
    with open(os.path.join(d, name + '.plank.log'), 'w') as fh:
        subprocess.run([PLANK, 'export', '-d', domain, '-p', problem, '-l', LIB,
                        os.path.join(EPDDL, 'lossy.epddl'), '-o', os.path.join(d, name)],
                       stdout=fh, stderr=subprocess.STDOUT, check=True)
    return os.path.join(d, name, name + '.json')


def value(line, key):
    if key + '=' not in line:
        return None
    return int(line.split(key + '=')[1].split()[0])


def measure(cell, out, budget):
    floor, n, m, strategy = cell
    task = os.path.join(out, f'n{n}-m{m}', *(2 * [scaled.problem_name(n, m, floor)]))
    task += '.json'
    run_dir = os.path.join(os.path.dirname(task), strategy)
    os.makedirs(run_dir, exist_ok=True)
    plan = os.path.join(run_dir, 'plan.json')
    if os.path.exists(plan):
        os.remove(plan)
    cmd = [PLANNER, '--task', task, '--plan', plan, '--timeout', str(budget), '--threads', '1',
           '--strategy', strategy, '--no-portfolio']
    t0 = time.time()
    killed = False
    try:
        run = subprocess.run(cmd, capture_output=True, text=True, timeout=1.5 * budget + 30)
        log = run.stdout + run.stderr
    except subprocess.TimeoutExpired as e:
        killed = True
        log = ''.join(x.decode() if isinstance(x, bytes) else x
                      for x in (e.stdout, e.stderr) if x)
    seconds = time.time() - t0
    with open(os.path.join(run_dir, 'search.log'), 'w') as fh:
        fh.write(log)

    row = {'floor': floor, 'robots': n, 'levels': m, 'strategy': strategy,
           'seconds': round(seconds, 2)}
    for line in log.splitlines():
        if line.startswith('[parser] Loaded:'):
            words = line.split()
            row['atoms'] = int(words[2])
            row['actions'] = int(words[words.index('actions') - 1])
        if line.startswith('[symmetry]'):
            row['swaps'] = int(line.split()[1])
        if 'Expanded=' in line:
            row['expanded'] = value(line, 'Expanded')
        if 'Solution found' in line:
            row['result'] = 'policy'
            if 'at depth' in line:
                row['depth'] = int(line.split('at depth')[1].split()[0])
        elif 'exhausted at depth' in line:
            row['result'] = 'no policy: space exhausted'
            row['depth'] = int(line.split('at depth')[1].split()[0])
        elif '[replan] No solution exists' in line:
            row['result'] = 'no policy: determinization refuted'
        elif 'Timeout at depth' in line:
            # Printed at the top of the next iteration: the search was cut
            # off inside the depth before the one named.
            row['result'] = 'timeout'
            row['depth'] = int(line.split('at depth')[1].split('.')[0]) - 1
        elif 'Deadline exceeded' in line:
            row['result'] = 'timeout'
    if killed:
        row['result'] = f'killed at {seconds:.0f} s, past its {budget:.0f} s budget'
    row.setdefault('result', 'no answer')

    if row['result'] == 'policy':
        tree = json.load(open(plan))
        row['leaves'] = leaves(tree)
        row['depth'] = row.get('depth') or height(tree)
        trace = subprocess.run([sys.executable, os.path.join(HERE, 'trace.py'), '--task', task,
                                '--plan', plan], capture_output=True, text=True)
        with open(os.path.join(run_dir, 'trace.log'), 'w') as fh:
            fh.write(trace.stdout + trace.stderr)
        holds = trace.stdout.count('goal holds')
        row['traced'] = 'C at every leaf' if trace.returncode == 0 and holds == row['leaves'] \
            else f'FAILED ({holds} of {row["leaves"]} leaves)'
    print(f'  {floor:6s} n={n:<2d} m={m}  {strategy:6s}  {row["result"]:36s} '
          f'{row.get("expanded", "")!s:>8s}  {seconds:7.1f} s', flush=True)
    return row


def leaves(node):
    if node is None:
        return 1
    return sum(leaves(b['subtree']) for b in node['branches']) or 1


def height(node):
    if node is None:
        return 0
    return 1 + max((height(b['subtree']) for b in node['branches']), default=0)


def table(rows, floor):
    out = []
    cells = sorted({(r['robots'], r['levels']) for r in rows if r['floor'] == floor})
    if floor == 'beacon':
        out.append('| robots | atoms | actions | AO\\* | replan | policy | traced |')
        out.append('| ---: | ---: | ---: | --- | --- | --- | --- |')
    else:
        out.append('| robots | levels | atoms | actions | AO\\* | replan |')
        out.append('| ---: | ---: | ---: | ---: | --- | --- |')
    for n, m in cells:
        by = {r['strategy']: r for r in rows
              if r['floor'] == floor and r['robots'] == n and r['levels'] == m}
        a, rp = by.get('aostar', {}), by.get('replan', {})

        def cell(r):
            if not r:
                return ''
            what = r['result']
            if what == 'policy':
                what = f'depth {r["depth"]}'
            elif what.startswith('no policy: space'):
                what = f'exhausted at depth {r["depth"]}'
            elif what.startswith('no policy: determinization'):
                what = 'refuted'
            elif what == 'timeout' and r.get('depth') is not None:
                what = f'cut off at depth {r["depth"]}'
            exp = (', ' + f'{r["expanded"]:,}'.replace(',', ' ')
                   if r.get('expanded') is not None else '')
            return f'{what}{exp}, {r["seconds"]:.1f} s'
        base = a or rp
        if floor == 'beacon':
            solved = a if a.get('result') == 'policy' else rp
            out.append(f'| {n} | {base.get("atoms", "")} | {base.get("actions", "")} | '
                       f'{cell(a)} | {cell(rp)} | '
                       f'{solved.get("leaves", "")} leaves | {solved.get("traced", "")} |')
        else:
            out.append(f'| {n} | {m} | {base.get("atoms", "")} | {base.get("actions", "")} | '
                       f'{cell(a)} | {cell(rp)} |')
    return '\n'.join(out)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', required=True)
    p.add_argument('--budget', type=float, default=300.0, help='seconds per search')
    p.add_argument('--jobs', type=int, default=4,
                   help='searches at once; each is one thread')
    p.add_argument('--only', choices=('beacon', 'radio'))
    args = p.parse_args()

    cells = []
    if args.only != 'radio':
        cells += [('beacon', n, m, s) for n, m in BEACON for s in STRATEGIES]
    if args.only != 'beacon':
        cells += [('radio', n, m, s) for n, m in RADIO for s in STRATEGIES]
    print(f'{len(cells)} searches, {args.budget:.0f} s each, {args.jobs} at once, with '
          f'{PLANNER}')
    # Grounded once per floor, before any search, so two strategies never
    # write the same task.
    for floor, n, m in sorted({c[:3] for c in cells}):
        ground(args.out, n, m, floor)
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(lambda c: measure(c, args.out, args.budget), cells))

    keys = ['floor', 'robots', 'levels', 'strategy', 'atoms', 'actions', 'swaps', 'result',
            'depth', 'expanded', 'seconds', 'leaves', 'traced']
    with open(os.path.join(args.out, 'scaling.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in keys})
    print()
    for floor in ('beacon', 'radio'):
        if any(r['floor'] == floor for r in rows):
            print(table(rows, floor))
            print()
    failed = [r for r in rows if r.get('traced', 'C').startswith('FAILED')]
    wrong = [r for r in rows if r['floor'] == 'radio' and r['result'] == 'policy']
    for r in failed:
        print(f'PROBLEM: the beacon policy for {r["robots"]} robots does not trace')
    for r in wrong:
        print(f'PROBLEM: a radio policy for {r["robots"]} robots, {r["levels"]} levels')
    return 1 if failed or wrong else 0


if __name__ == '__main__':
    sys.exit(main())
