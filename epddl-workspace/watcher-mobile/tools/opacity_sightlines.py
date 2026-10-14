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
Opacity of a watcher policy against a watcher that knows it, by the sightlines.

    opacity_sightlines.py TASK.json PLAN.json

tools/opacity.py reads what the watcher observes off the domain's
observability declarations. This reads it off the hall instead, as
tools/check_views.py found it, whatever the domain declares: after every step
the watcher knows what it did itself, sees the carrier if the carrier stands on
the watcher's side (at a window or a gate approach of that hall), sees the
lamp only at the window of its side, and the gate of its side. Nothing else:
not the boards, not the corridor, not the other hall.

The watcher's moves are the events whose names say so (`-go-`); every run of
the policy is walked, one per choice of them, and so every strategy a moving
watcher could follow is covered. The policy is opaque iff any two worlds the
watcher cannot tell apart at the start and that differ on the order produce
the same set of traces. Exits non-zero otherwise, naming a pair of runs that
tells them apart.
"""

import json
import sys

sys.path.insert(0, __file__.rsplit('/', 1)[0])
sys.path.insert(0, __file__.rsplit('/', 1)[0] + '/../../../ros2_ws/src/watcher_demo/tools')
from opacity import holds  # noqa: E402

PLACES = {'at-win-w_r1': ('w', 'the west window'), 'at-win-e_r1': ('e', 'the east window'),
          'at-ga-w_r1': ('w', 'the west gate approach'), 'at-ga-e_r1': ('e', 'the east gate approach')}


def side_of_watcher(world):
    return 'w' if 'west_r3' in world else 'e'


def observe(name, event, world):
    """What the watcher sees after one step, in the state the step left."""
    seen = []
    if '-go-' in event:
        seen.append(f'it moves to the {"west" if "-go-w" in event else "east"} post')
    me = side_of_watcher(world)
    for atom, (side, what) in PLACES.items():
        if atom in world and side == me:
            seen.append(f'r1 at {what}')
    if name.startswith(('say-w', 'say-e')) and name[4] == me:
        seen.append(f'the lamp: {event}')
    if name.startswith('gate-') and {'x': 'w', 'y': 'e'}[name[5]] == me:
        seen.append(f'r1 through {name[:6]}')
    return '; '.join(seen) or None


def runs(task, plan, world):
    out = []

    def go(node, world, steps, trace):
        if node is None:
            out.append((steps, tuple(t for t in trace if t is not None)))
            return
        name = node['action']
        action = task['actions'][name]
        designated = action['designated']
        for event in [e for e in designated if holds(action['preconditions'][e], world)]:
            w = set(world)
            for atom, val in ((action.get('effects') or {}).get(event) or {}).items():
                (w.add if holds(val, world) else w.discard)(atom)
            branch = next((b for b in node['branches'] if designated[b['event']] == event), None)
            if branch is None and len(node['branches']) == 1:
                branch = node['branches'][0]
            if branch is None:
                raise SystemExit(f'{name}: the policy has no branch for {event}')
            go(branch['subtree'], w, steps + [f'{name}:{event}'], trace + [observe(name, event, w)])
    go(plan, set(world), [], [])
    return out


def main():
    task, plan = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    init = task['initial-state']
    every = {w: runs(task, plan, init['labels'][w]) for w in init['designated']}
    rel = init['relations']['r3']
    bad = []
    for w in init['designated']:
        for v in rel[w]:
            if v <= w or ('dest-x' in init['labels'][w]) == ('dest-x' in init['labels'][v]):
                continue
            tw, tv = {t for _, t in every[w]}, {t for _, t in every[v]}
            for t in sorted(tw ^ tv):
                src = w if t in tw else v
                steps = next(s for s, tt in every[src] if tt == t)
                bad.append((w, v, src, t, steps))
    for w in init['designated']:
        print(f'{w}: {len(every[w])} runs, {len({t for _, t in every[w]})} distinct traces')
    if bad:
        w, v, src, t, steps = bad[0]
        print(f'NOT OPAQUE: {len(bad)} traces tell {w} from {v} (and others); e.g. in {src}:')
        print('   the run   ', ' '.join(steps))
        print('   the watcher sees', ' | '.join(t) or 'nothing')
        return 1
    print('opaque by the sightlines: every trace the watcher can see, whatever it does, is seen in a world of '
          'either order')
    return 0


if __name__ == '__main__':
    sys.exit(main())
