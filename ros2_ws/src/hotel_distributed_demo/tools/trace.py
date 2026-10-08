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
The hotel's distributed leak, traced update by update.

    trace.py --task out/epistemic/epistemic.json --plan plan.json --whole whole.json
    trace.py --task out/filter/filter.json --leak L3_room1 --plan plan.json --whole whole.json
    trace.py --task out/epistemic/epistemic.json --leak L3_room1 \\
             --actions go_concierge_lobby_restaurant tell-column_cleaner_restaurant ...

--whole names a grounded task whose goal is the whole goal (the epistemic
fleet's), so that every fleet's policy is judged by the same three conjuncts
whatever goal it was planned for:

  safe        the leak is contained
  stand-down  every responder knows that every responder knows it: E_R E_R safe
  secret      the guest cannot rule out any room: <guest> source(z) for every z

With --plan every branch of the policy is walked from the designated worlds
the planner starts from, and each leaf is judged at each of its designated
worlds. With --leak the actual world is the one where that room is the
source, and only the branch it takes is followed.

After every update the trace prints, for each agent at the actual world, the
rooms it cannot rule out and what it knows of the leak being contained; and,
along the way, the lift rides (moves between floors), the inspections and the
messages, private and public.

The product update is coordinated_attack_demo's, in its tools/trace.py, which
evaluates observability once per state as plank and Aletheia do. Models are
contracted after every update, which changes no formula's truth value.
"""

import argparse
import importlib.util
import json
import os
import sys


def _trace_module():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', '..', 'coordinated_attack_demo', 'tools')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory('coordinated_attack_demo'), 'tools'))
    except Exception:   # noqa: BLE001  outside a sourced workspace
        pass
    for path in candidates:
        source = os.path.join(path, 'trace.py')
        if os.path.exists(source):
            spec = importlib.util.spec_from_file_location('ca_trace', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError('coordinated_attack_demo/tools/trace.py not found')


Model = _trace_module().Model

AGENTS = ('cleaner', 'concierge', 'porter', 'guest')


def rooms_of(model):
    return sorted({a[len('source_'):] for w in model.worlds
                   for a in model.labels[w] if a.startswith('source_')})


def floor_of(zone):
    return zone.split('_')[0] if zone.startswith('L') and '_' in zone else 'L1'


def candidates(model, agent, worlds, rooms):
    """The rooms the agent cannot rule out from any of @p worlds."""
    return sorted({z for w in worlds for v in model.sees(agent, w)
                   for z in rooms if f'source_{z}' in model.labels[v]})


def safe_status(model, agent, worlds):
    seen = {'safe' in model.labels[v] for w in worlds for v in model.sees(agent, w)}
    return 'knows contained' if seen == {True} else \
        'knows leaking' if seen == {False} else 'does not know'


def conjuncts(whole):
    """The whole goal's three conjuncts, as formula nodes, by shape: the
    atom, the nested group box, and the conjunction of diamonds."""
    out = {}
    for f in whole['goal']['formula']['formulas']:
        if f == 'safe':
            out['safe'] = f
        elif isinstance(f, dict) and 'modality-name' in f:
            out['stand-down'] = f
        else:
            out['secret'] = f
    assert set(out) == {'safe', 'stand-down', 'secret'}, out.keys()
    return out


def verdict(model, parts, worlds):
    return {k: all(model.holds(w, f) for w in worlds) for k, f in parts.items()}


def report(model, worlds, rooms, indent):
    for a in AGENTS:
        if a not in model.relations:
            continue
        c = candidates(model, a, worlds, rooms)
        shown = ', '.join(c) if len(c) <= 2 else f'{len(c)} rooms'
        print(f'{indent}{a:9s} rooms: {shown:24s} {safe_status(model, a, worlds)}')


class Tally:

    def __init__(self):
        self.rides = self.inspections = self.private = self.public = 0

    def count(self, name):
        head = name.split('_')[0]
        if head == 'go':
            parts = name.split('_')
            # go_<agent>_<from>_<to>, zones with underscores: split on the
            # agent, then find the boundary between two zone names.
            rest = '_'.join(parts[2:])
            a, b = split_zones(rest)
            self.rides += floor_of(a) != floor_of(b)
        elif head == 'inspect':
            self.inspections += 1
        elif head.startswith('tell-'):
            self.private += 1
        elif head.startswith('page-'):
            self.public += 1

    def copy(self):
        t = Tally()
        t.__dict__.update(self.__dict__)
        return t

    def __str__(self):
        return (f'{self.rides} lift rides, {self.inspections} inspections, '
                f'{self.private} private and {self.public} public messages')


ZONES = []


def split_zones(rest):
    for z in sorted(ZONES, key=len, reverse=True):
        if rest.startswith(z + '_') and rest[len(z) + 1:] in ZONES:
            return z, rest[len(z) + 1:]
    raise ValueError('cannot split ' + rest)


def zones_of(task):
    """Every zone the task's go actions name."""
    names = set()
    for w in task['initial-state']['worlds']:
        for a in task['initial-state']['labels'][w]:
            if a.startswith('room_'):
                names.add(a[len('room_'):])
    names |= {'lobby', 'restaurant'}
    return sorted(names)


def outcome_of(model, act, world):
    pre = act.get('preconditions') or {}
    for e in act['designated']:
        if model.holds(world, (pre.get(e) or {}).get('formula')):
            return e
    return None


def walk(node, task, model, parts, rooms, indent, tally, leaves):
    """Every branch of the policy whose outcome can occur at one of the
    model's designated worlds; with the actual world alone designated, the
    branch the run takes."""
    worlds = model.designated
    if node is None:
        v = verdict(model, parts, worlds)
        src = sorted({z for w in worlds for z in rooms if f'source_{z}' in model.labels[w]})
        print(f'{indent}leaf, source {", ".join(src)}: ' +
              '  '.join(f'{k} {"holds" if ok else "FAILS"}' for k, ok in v.items()) +
              f'  ({tally})')
        leaves.append((src, v, tally))
        return
    act = task['actions'][node['action']]
    if not model.applicable(act):
        print(f'{indent}{node["action"]}  NOT APPLICABLE')
        leaves.append((None, {'applicable': False}, tally))
        return
    for b in node['branches']:
        outcome = act['designated'][b['event']] if len(act['designated']) > 1 else None
        child = model.update(act, outcome).contracted()
        if not child.designated:
            continue
        t = tally.copy()
        t.count(node['action'])
        print(f'{indent}{node["action"]}' + (f'  [{outcome}]' if outcome else ''))
        report(child, child.designated, rooms, indent + '    ')
        walk(b['subtree'], task, child, parts, rooms, indent + '  ', t, leaves)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True)
    p.add_argument('--whole', help='a grounded task whose goal is the whole goal')
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--plan')
    g.add_argument('--actions', nargs='+')
    p.add_argument('--leak', help='the room that is the source at the actual world')
    args = p.parse_args()

    task = json.load(open(args.task))
    whole = json.load(open(args.whole)) if args.whole else task
    parts = conjuncts(whole)
    model = Model.of(task)
    rooms = rooms_of(model)
    ZONES[:] = zones_of(task)

    if args.leak:
        hits = [w for w in model.designated if f'source_{args.leak}' in model.labels[w]]
        if not hits:
            raise SystemExit(f'no designated world where {args.leak} is the source')
        model = Model(model.worlds, model.relations, model.labels, hits)

    print('initial model' + (f', leak in {args.leak}' if args.leak else ''))
    report(model, model.designated, rooms, '    ')
    leaves = []
    if args.plan:
        plan = json.load(open(args.plan))
        if plan is None:
            print('the planner returned no plan')
            return 1
        walk(plan, task, model, parts, rooms, '  ', Tally(), leaves)
    else:
        tally = Tally()
        for name in args.actions:
            act = task['actions'][name]
            if not model.applicable(act):
                print(f'  {name}  NOT APPLICABLE')
                return 1
            out = outcome_of(model, act, model.designated[0]) \
                if len(act['designated']) > 1 else None
            model = model.update(act, out).contracted()
            tally.count(name)
            print(f'  {name}' + (f'  [{out}]' if out else ''))
            report(model, model.designated, rooms, '      ')
        v = verdict(model, parts, model.designated)
        print('end: ' + '  '.join(f'{k} {"holds" if ok else "FAILS"}' for k, ok in v.items())
              + f'  ({tally})')
        leaves.append((None, v, tally))

    if args.plan:
        n = len(leaves)
        held = {k: sum(1 for _, v, _ in leaves if v.get(k)) for k in parts}
        print(f'{n} leaves; ' + ', '.join(f'{k} at {held[k]}' for k in parts))
    return 0


if __name__ == '__main__':
    sys.exit(main())
