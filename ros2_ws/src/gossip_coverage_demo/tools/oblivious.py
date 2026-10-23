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
The paper's eq. (18) read literally: robots out of contact believe that
nothing happened. A policy planned with Unsure observers is replayed in the
model of the oblivious library (variants/gossip-oblivious.epddl), and at
every step this reports, for every robot at the actual world:

  reflexive  whether the actual world is among the worlds it considers
             possible: the T axiom that eq. (17)'s "knowledge and belief are
             equivalent" rests on;
  false      a formula it "knows" that is false: K_k not K_j task with K_j
             task true;
  empty      whether it considers no world possible: it believes a
             contradiction, and every knowledge formula holds of it vacuously.

    oblivious.py INSTANCE.epddl POLICY.json [--json OUT]

The same report with the Unsure library is printed alongside: every robot
keeps the actual world, and nobody knows anything false.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.environ.get('ER_TOOLS', os.path.join(HERE, '..', '..', '..', '..', 'tools')))
from gossip import epddl as P  # noqa: E402
from check_policy import follow, holds_at  # noqa: E402


def export(instance, library):
    d = tempfile.mkdtemp(prefix='gossip-obl-')
    subprocess.run([P.PLANK, 'export', '-d', os.path.join(P.WORKSPACE, 'gossip-coverage.epddl'), '-p', instance,
                    '-l', P.INTERMEDIATE, library, '-o', d], check=True, capture_output=True)
    name = os.path.splitext(os.path.basename(instance))[0]
    return json.load(open(os.path.join(d, f'{name}.json')))


def report(task, policy):
    agents = task['language']['agents']
    out = []
    for w in sorted(task['initial-state']['designated'], key=lambda w: int(w[1:])):
        steps = follow(task, policy, w, agents)
        for st in steps:
            model, actual = st['after']
            row = {'world': w, 'action': st['action'], 'event': st['event'], 'robots': {}}
            for k in agents:
                image = model.image(k, actual)
                false = [j for j in agents if j != k
                         and holds_at(model, actual, f'(K {j} task)')
                         and holds_at(model, actual, f'(K {k} (not (K {j} task)))')]
                row['robots'][k] = {'reflexive': actual in image, 'empty': not image, 'false': false}
            out.append(row)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('instance')
    ap.add_argument('policy')
    ap.add_argument('--json')
    a = ap.parse_args()
    policy = json.load(open(a.policy))
    unsure = export(a.instance, os.path.join(P.WORKSPACE, 'gossip.epddl'))
    oblivious = export(a.instance, os.path.join(P.WORKSPACE, 'variants', 'gossip-oblivious.epddl'))
    result = {}
    for name, task in (('unsure', unsure), ('oblivious', oblivious)):
        try:
            rows = report(task, policy)
        except SystemExit as e:
            rows = [{'error': str(e)}]
        result[name] = rows
        print(f'== {name}')
        for r in rows:
            if 'error' in r:
                print('  ', r['error'])
                continue
            bad = {k: v for k, v in r['robots'].items() if not v['reflexive'] or v['empty'] or v['false']}
            marks = '; '.join(f"{k}: {'contradiction' if v['empty'] else ''}"
                              f"{'' if v['reflexive'] or v['empty'] else 'not reflexive'}"
                              f"{(' knows falsely that ' + ', '.join(v['false']) + ' does not know') if v['false'] else ''}"
                              for k, v in bad.items())
            print(f"  {r['world']} {r['action']:<16} {r['event']:<12} {marks or 'every robot keeps the actual world'}")
    if a.json:
        json.dump(result, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
