#!/usr/bin/env python3
"""The planning study of the watcher that moves.

    study.py OUT.json [--moves 0 1 2 3]

For each number of changes of post K the watcher has, with the gates and
without them: the domain written by make_domain.py, grounded by plank, and
planned twice by Aletheia's AO* -- branching on every designated event, as the
installed planner does, and branching on what each action's actor observes
(--subjective auto) -- both with --consistent-beliefs. Of each policy: its
depth, nodes, leaves, expansions and time; how many of its branch points its
actor cannot follow (executable.py); and whether it keeps the order from a
watcher that knows it, by the sightlines (opacity_sightlines.py).
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PLANK = os.path.expanduser('~/plank/build/plank')
LIB = os.path.expanduser('~/plank/benchmarks/libraries/intermediate.epddl')
OBJECTIVE = os.path.expanduser('~/eplansys_ws/install/aletheia/bin/epistemic_planner')
SUBJECTIVE = os.path.expanduser('~/aletheia-subjective/build/epistemic_planner')


def tree(plan):
    nodes, leaves = 0, 0

    def walk(n):
        nonlocal nodes, leaves
        if n is None:
            leaves += 1
            return
        nodes += 1
        for b in n['branches']:
            walk(b['subtree'])
    walk(plan)
    return nodes, leaves


def plan(binary, task, out, extra):
    t = time.time()
    r = subprocess.run([binary, '--task', task, '--plan', out, '--conditional', '--consistent-beliefs',
                        '--timeout', '1200', *extra], capture_output=True, text=True)
    text = r.stdout + r.stderr
    row = {'seconds': round(time.time() - t, 2), 'solved': 'Solution found' in text}
    if not row['solved']:
        return row
    m = re.search(r'Solution found at depth (\d+)\s+Expanded=(\d+)', text)
    row['depth'], row['expanded'] = int(m.group(1)), int(m.group(2))
    row['nodes'], row['leaves'] = tree(json.load(open(out)))
    ex = subprocess.run([sys.executable, os.path.join(HERE, 'executable.py'), task, out], capture_output=True,
                        text=True).stdout
    row['unfollowable'] = int(re.search(r'(\d+) branch points', ex).group(1))
    op = subprocess.run([sys.executable, os.path.join(HERE, 'opacity_sightlines.py'), task, out],
                        capture_output=True, text=True)
    row['opaque'] = op.returncode == 0
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--moves', type=int, nargs='+', default=[0, 1, 2, 3])
    a = ap.parse_args()
    rows = []
    for gates in (True, False):
        for k in a.moves:
            d = os.path.join(ROOT, 'gen', f'{"" if gates else "nogate-"}k{k}')
            subprocess.run([sys.executable, os.path.join(ROOT, 'make_domain.py'), '--moves', str(k), '--out', d]
                           + ([] if gates else ['--no-gates']), check=True)
            subprocess.run([PLANK, 'export', '-d', f'{d}/watcher-mobile.epddl', '-p', f'{d}/instances/problem_{k}.epddl',
                            '-l', LIB, f'{d}/sightlines-mobile.epddl', '-o', f'{d}/out'], check=True,
                           capture_output=True)
            task = f'{d}/out/problem_{k}.json'
            g = json.load(open(task))
            row = {'gates': gates, 'moves': k, 'atoms': len(g['language']['atoms']), 'actions': len(g['actions']),
                   'objective': plan(OBJECTIVE, task, f'{d}/out/plan-objective.json', []),
                   'subjective': plan(SUBJECTIVE, task, f'{d}/out/plan-subjective.json', ['--subjective', 'auto'])}
            rows.append(row)
            print(json.dumps(row))
    json.dump(rows, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
