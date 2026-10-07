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
How many radio messages each level of mutual knowledge costs, with n robots.

    ladder.py --robots 3 --depth 3 --out DIR
    ladder.py --robots 4 --depth 2 --out DIR --protocol DIR/protocol.json
    ladder.py --table 2,3,4 --depths 1,2,3,4 --out DIR
    ladder.py --replay protocols/radio-n4-m2.json --out DIR

The messages are tools/scaled.py's: level l from i to j says K_i E^(l-1)
job(s). The planner is given the radio floor with the goal E^k job(s) in
place of lifted, and AO* deepens one action at a time, so the depth of the
first policy it finds is the fewest actions that reach E^k: the order read,
and then the fewest messages. The policy is then replayed by trace.py's
product update, which prints the depth of mutual knowledge after every
message and has to agree that E^k holds at the end, that C does not, and
that lift does not apply.

The count is not what the levels alone suggest. A robot that knows job(s)
can only have learnt it from someone who knew it first, so even a level 1
message from north tells south that north heard from south. Who could have
told whom is in the model, and the planner uses it.

--protocol writes the messages of each branch, which the mission runs on
the radio floor in Gazebo. --replay takes such a file and applies it to the
radio floor the mission runs, whose goal is lifted, with the same checks.
"""

import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace import Model  # noqa: E402
import scaled  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EPDDL = os.path.join(HERE, '..', 'epddl')
HOME = os.path.expanduser('~')
PLANK = os.environ.get('PLANK', os.path.join(HOME, 'plank', 'build', 'plank'))
PLANNER = os.environ.get('EPISTEMIC_PLANNER', os.path.join(
    HOME, 'eplansys_ws', 'install', 'aletheia', 'bin', 'epistemic_planner'))
LIB = os.environ.get('PLANK_LIB', os.path.join(
    HOME, 'plank', 'benchmarks', 'libraries', 'intermediate.epddl'))


def ground(out, n, m, depth=None):
    """Writes and grounds the radio floor, with the goal E^depth, or lifted
    without one. Returns the grounded task's path."""
    os.makedirs(out, exist_ok=True)
    domain = os.path.join(out, scaled.domain_name(m) + '.epddl')
    with open(domain, 'w') as fh:
        fh.write(scaled.domain_text(m))
    name = scaled.problem_name(n, m, 'radio', depth)
    problem = os.path.join(out, name + '.epddl')
    with open(problem, 'w') as fh:
        fh.write(scaled.problem_text(n, m, 'radio', depth))
    log = os.path.join(out, name + '.plank.log')
    with open(log, 'w') as fh:
        if subprocess.run([PLANK, 'export', '-d', domain, '-p', problem, '-l', LIB,
                           os.path.join(EPDDL, 'lossy.epddl'), '-o', os.path.join(out, name)],
                          stdout=fh, stderr=subprocess.STDOUT).returncode:
            raise SystemExit(open(log).read())
    return os.path.join(out, name, name + '.json')


def solve(task, timeout):
    """AO* with the whole budget. Returns (plan or None, log, seconds)."""
    plan = os.path.join(os.path.dirname(task), 'plan.json')
    t0 = time.time()
    run = subprocess.run([PLANNER, '--task', task, '--plan', plan, '--timeout', str(timeout),
                          '--strategy', 'aostar', '--no-portfolio', '--threads', '1'],
                         capture_output=True, text=True)
    seconds = time.time() - t0
    log = run.stdout + run.stderr
    with open(os.path.join(os.path.dirname(task), 'search.log'), 'w') as fh:
        fh.write(log)
    if 'Solution found' not in log:
        return None, log, seconds
    return json.load(open(plan)), log, seconds


def branches(task, plan):
    """The policy's two branches, as the stand each settles and the messages
    after the order is read."""
    root = plan
    read = task['actions'][root['action']]
    out = {}
    for b in root['branches']:
        event = read['designated'][b['event']]
        seq, node = [], b['subtree']
        while node is not None:
            seq.append(node['action'])
            node = node['branches'][0]['subtree'] if node['branches'] else None
        out[(root['action'], event)] = seq
    return out


def replay(task, read, outcome, seq, show=True):
    """Applies the order, then the messages, with the product update; returns
    the depths after each and whether lift applies at the end."""
    model = Model.of(task).update(task['actions'][read], outcome).contracted()
    stand = next(a for a in model.labels[model.designated[0]] if a.startswith('job_'))[4:]
    depths = []
    for action in seq:
        act = task['actions'][action]
        if not model.applicable(act):
            raise SystemExit(f'{action} does not apply')
        model = model.update(act).contracted()
        k, _ = model.depth(f'job_{stand}')
        depths.append(k)
        if show:
            print(f'    {action:28s} {len(model.worlds):5d} worlds  '
                  + ('C' if k is None else f'E^{k}'))
    lifts = model.applicable(task['actions'][f'lift_{stand}'])
    return stand, depths, lifts


def ladder(n, depth, m, out, timeout, show=True):
    """The fewest messages that reach E^depth with n robots, checked by the
    product update. Returns (messages per stand, problems, row)."""
    task_path = ground(os.path.join(out, f'n{n}-e{depth}'), n, m, depth)
    task = json.load(open(task_path))
    plan, log, seconds = solve(task_path, timeout)
    row = {'robots': n, 'depth': depth, 'levels': m, 'seconds': round(seconds, 2)}
    for line in log.splitlines():
        if 'Solution found' in line:
            row['expanded'] = int(line.split('Expanded=')[1].split()[0])
        if 'Timeout' in line or 'exhausted' in line:
            row['result'] = line.split('] ', 1)[1]
        if 'Timeout at depth' in line:
            # Printed at the top of the next iteration, so the search was cut
            # off inside depth X-1 and every depth up to X-2 was searched
            # without a policy: at least X-1 actions, the order read and X-2
            # messages.
            row['at_least'] = int(line.split('at depth')[1].split('.')[0]) - 2
    if plan is None:
        row['messages'] = None
        if show:
            print(f'{n} robots, E^{depth}: no policy ({row.get("result", "no answer")})')
        return None, [], row
    problems, protocol = [], {}
    for (read, outcome), seq in branches(task, plan).items():
        if show:
            print(f'{n} robots, E^{depth}, branch {outcome}: {len(seq)} messages '
                  f'({row["expanded"]} expansions, {seconds:.1f} s)')
            print(f'    {read}  [{outcome}]')
        stand, depths, lifts = replay(task, read, outcome, seq, show)
        protocol[stand] = seq
        if depths[-1] is None:
            problems.append(f'C after {seq}')
        elif depths[-1] < depth:
            problems.append(f'the planner\'s branch ends at E^{depths[-1]}, not E^{depth}')
        if lifts:
            problems.append(f'lift_{stand} applies after {seq}')
        if show:
            print(f'    lift_{stand}  ' + ('APPLICABLE' if lifts else 'NOT APPLICABLE'))
    row['messages'] = max(len(s) for s in protocol.values())
    return protocol, problems, row


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robots', type=int, default=2)
    p.add_argument('--depth', type=int, default=4)
    p.add_argument('--messages', type=int,
                   help='message levels on the floor; default the depth asked for')
    p.add_argument('--table', help='robot counts, comma separated: a table of message counts')
    p.add_argument('--depths', default='1,2,3,4', help='with --table, the depths')
    p.add_argument('--timeout', type=float, default=300.0, help='seconds per search')
    p.add_argument('--out', required=True)
    p.add_argument('--protocol', help='write the messages of each branch to this file')
    p.add_argument('--csv', help='with --table, also write the rows here')
    p.add_argument('--replay', help='a protocol file to apply to the radio floor')
    args = p.parse_args()

    problems = []
    if args.replay:
        protocol = json.load(open(args.replay))
        n, depth, m = protocol['robots'], protocol['depth'], protocol['levels']
        task = json.load(open(ground(os.path.join(args.out, f'replay-n{n}-m{m}'), n, m)))
        reader = scaled.READER
        for stand, outcome in zip(scaled.STANDS, ('e-here', 'e-elsewhere')):
            seq = protocol[stand]
            print(f'{n} robots, {args.replay}, {stand}: {len(seq)} messages')
            print(f'    read-order_{reader}_{scaled.STANDS[0]}  [{outcome}]')
            _, depths, lifts = replay(task, f'read-order_{reader}_{scaled.STANDS[0]}',
                                      outcome, seq)
            print(f'    lift_{stand}  ' + ('APPLICABLE' if lifts else 'NOT APPLICABLE'))
            if depths[-1] is None or depths[-1] < depth:
                problems.append(f'{stand}: ends at '
                                + ('C' if depths[-1] is None else f'E^{depths[-1]}')
                                + f', not E^{depth}')
            if lifts:
                problems.append(f'lift_{stand} applies')
    elif args.table:
        rows = []
        robots = [int(v) for v in args.table.split(',')]
        depths = [int(v) for v in args.depths.split(',')]
        for n in robots:
            for k in depths:
                _, found, row = ladder(n, k, args.messages or k, args.out, args.timeout,
                                       show=False)
                problems += found
                rows.append(row)
                if row['messages'] is None:
                    found_text = row.get('result', 'no answer')
                    if 'at_least' in row:
                        found_text += f'; at least {row["at_least"]} messages'
                else:
                    found_text = (f'{row["messages"]} messages, {row["expanded"]} expansions, '
                                  f'{row["seconds"]} s')
                print(f'  n={n} E^{k}: {found_text}', flush=True)
                if row['messages'] is None:
                    break
        print()
        print('| robots | ' + ' | '.join(f'E^{k}' for k in depths) + ' |')
        print('| --- |' + ' ---: |' * len(depths))
        for n in robots:
            cells = []
            for k in depths:
                row = next((r for r in rows if r['robots'] == n and r['depth'] == k), None)
                if row is None:
                    cells.append('')
                elif row['messages'] is not None:
                    cells.append(str(row['messages']))
                else:
                    cells.append(f'≥ {row["at_least"]}' if 'at_least' in row else '>')
            print(f'| {n} | ' + ' | '.join(cells) + ' |')
        if args.csv:
            keys = ['robots', 'depth', 'levels', 'messages', 'at_least', 'expanded', 'seconds',
                    'result']
            with open(args.csv, 'w') as fh:
                fh.write(','.join(keys) + '\n')
                for r in rows:
                    fh.write(','.join(str(r.get(k, '')) for k in keys) + '\n')
    else:
        protocol, problems, row = ladder(args.robots, args.depth, args.messages or args.depth,
                                         args.out, args.timeout)
        if protocol and args.protocol:
            with open(args.protocol, 'w') as fh:
                json.dump({'robots': args.robots, 'depth': args.depth,
                           'levels': args.messages or args.depth, **protocol}, fh, indent=2)
                fh.write('\n')
            print(f'{args.protocol}: the messages of each branch')
        if protocol is None:
            problems.append('the planner found no policy')

    for msg in problems:
        print('PROBLEM: ' + msg)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
