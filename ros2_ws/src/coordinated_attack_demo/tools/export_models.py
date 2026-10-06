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
Exports the Kripke model after every action of three action sequences, for the
figures of the project pages and the report.

    export_models.py --radio radio.json --beacon beacon.json --out models.json

  radio   read-order(south, s1), tell, ack, ack2, ack3: the protocol the
          radio floor runs, with lift refused at the end
  beacon  go-view(north), read-order(south, s1), go-view(south),
          signal(south, s1), lift(s1): the branch of the policy the recorded
          run took
  alone   read-order(south, s1), go-view(south), signal(south, s1): the
          beacon lit with north at no viewpoint

The models are computed by trace.py's product update from the tasks plank
ground, as the trace is, and only the part reachable from the designated
worlds is kept: a world no agent can reach from the actual one plays no part
in what anyone knows. For each step the export carries the worlds, each
agent's relation, the designated set, the depth of mutual knowledge of the
settled stand and the chain that bounds it.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace import Model  # noqa: E402

SEQUENCES = {
    'radio': ('radio', ['read-order_south_s1', 'tell_south_north_s1', 'ack_north_south_s1',
                        'ack2_south_north_s1', 'ack3_north_south_s1']),
    'beacon': ('beacon', ['go-view_north', 'read-order_south_s1', 'go-view_south',
                          'signal_south_s1', 'lift_s1']),
    'alone': ('beacon', ['read-order_south_s1', 'go-view_south', 'signal_south_s1']),
}


def reachable(model):
    seen = set(model.designated)
    todo = list(model.designated)
    while todo:
        u = todo.pop()
        for a in model.agents:
            for v in model.sees(a, u):
                if v not in seen:
                    seen.add(v)
                    todo.append(v)
    return seen


def snapshot(model, action, outcome):
    keep = reachable(model)
    order = sorted(keep, key=lambda w: (w not in model.designated, len(w), w))
    names = {w: f'u{i}' for i, w in enumerate(order)}
    stand = next((s for s in ('s1', 's2')
                  if all(f'job_{s}' in model.labels[w] for w in model.designated)), None)
    depth, chain_agents = (None, [])
    if stand:
        depth, chain_agents = model.depth(f'job_{stand}')
    # The chain as worlds: the breadth-first path the depth was read from.
    path = []
    if stand and chain_agents:
        from collections import deque
        parent = {w: None for w in model.designated}
        q = deque(model.designated)
        end = None
        while q and end is None:
            u = q.popleft()
            for a in model.agents:
                for v in model.sees(a, u):
                    if v in parent:
                        continue
                    parent[v] = (u, a)
                    if f'job_{stand}' not in model.labels[v]:
                        end = v
                        break
                    q.append(v)
                if end:
                    break
        x = end
        while x is not None and parent[x] is not None:
            u, a = parent[x]
            path.append([names[u], a, names[x]])
            x = u
        path.reverse()
    return {
        'action': action,
        'outcome': outcome,
        'worlds': [{'id': names[w], 'history': w,
                    'job': sorted(a[4:] for a in model.labels[w] if a.startswith('job_')),
                    'atoms': sorted(model.labels[w]),
                    'designated': w in model.designated} for w in order],
        'relations': {a: sorted([names[w], names[v]] for w in order for v in model.sees(a, w)
                                if v in keep and w != v)
                      for a in model.agents},
        'stand': stand,
        'depth': 'C' if (stand and depth is None) else depth,
        'chain': path,
        'knows': {a: bool(stand) and all(f'job_{stand}' in model.labels[v]
                                         for w in model.designated for v in model.sees(a, w))
                  for a in model.agents},
        'size': len(model.worlds),
    }


def run(task, names):
    model = Model.of(task)
    steps = [snapshot(model, 'initial', None)]
    for name in names:
        act = task['actions'][name]
        outcome = act['designated'][0] if len(act['designated']) > 1 else None
        if not model.applicable(act):
            steps.append({'action': name, 'refused': True})
            break
        model = model.update(act, outcome)
        steps.append(snapshot(model, name, outcome))
    return steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radio', required=True)
    ap.add_argument('--beacon', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    tasks = {'radio': json.load(open(args.radio)), 'beacon': json.load(open(args.beacon))}
    out = {}
    for key, (floor, names) in SEQUENCES.items():
        out[key] = run(tasks[floor], names)
        # lift is tried after the radio protocol, and is refused there.
        if key == 'radio':
            act = tasks['radio']['actions']['lift_s1']
            m = Model.of(tasks['radio'])
            for n in names:
                a = tasks['radio']['actions'][n]
                o = a['designated'][0] if len(a['designated']) > 1 else None
                m = m.update(a, o)
            out[key].append({'action': 'lift_s1', 'refused': not m.applicable(act)})
    with open(args.out, 'w') as fh:
        json.dump(out, fh, separators=(',', ':'))
    for key, steps in out.items():
        print(key, ' | '.join(
            f'{s["action"]}: ' + ('refused' if s.get('refused') else
                                  f'{len(s["worlds"])}/{s["size"]} worlds, depth {s["depth"]}')
            for s in steps))


if __name__ == '__main__':
    main()
