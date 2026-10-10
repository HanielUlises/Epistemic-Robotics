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
What each fleet will do on the floor, worked out before it runs.

    fleets.py                    the four fleets, as a table
    fleets.py --json out.json    the same, for the launch to hand the mission
    fleets.py --transient        robots that saw a change no longer looking

Every fleet sees the forklift's two changes, and the robots each change
flips are the floor's (check_floor.py). The fleets then differ in how they
send maps:

  epistemic   the radio floor's policy: each hauler whose map has t3 wrong
              is sent t3 by a robot that believes its map stale, merged by
              observation time
  fuse        every robot sends every other its map of every bay, sender by
              sender, merged by epistemic_slam::fuse's rule: of two settled
              readings the more confident, a tie keeping the receiver's
  overwrite   the same, the sender's reading replacing the receiver's
  recency     the same, merged by observation time

A bay's reading is passed whole: under every rule a map either takes the
sender's reading of a bay or keeps its own, because every cell the forklift
touched was observed at once.

A robot whose station sees a changed bay goes on seeing it. A stale reading
written over its map of that bay is read off again by its own laser within
a few seconds, long before its own turn to send comes round, so it sends
what its laser reads and ends with it. The prediction counts the writes that
replaced a correct reading with a stale one, which on the floor are visible
only for those few seconds and only because the robot was still looking.

Then each hauler drives to its drop through whichever bay its map shows
clear, by the shortest route, or through the bay its plan names. On the way
its laser keeps reading the floor: positions along the route are tested for
sight of each changed bay, as check_floor.py tests the stations, and when its
map's reading of a bay flips the hauler plans again. A hauler with no route
on its map stops. This is how the map a stale hauler starts with can be
repaired by its own laser on the way, and a prediction that left it out
would be wrong about the fleets that send no useful map.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_floor as C  # noqa: E402
import layout as L  # noqa: E402

FLEETS = ('epistemic', 'fuse', 'overwrite', 'recency')

# The secret floor: r4 is a contractor's robot that hauls and must not learn
# that t1 was staged. The plan, and three protocols from the communication
# study, whose sends are computed by study/protocols.py on this floor and
# executed by the mission in the same order.
SECRET_FLEETS = ('secret', 'flood', 'flood3', 'pull')
PROTOCOL = {'flood': ('flood', None), 'flood3': ('flood', 3), 'pull': ('pull', None)}


def after_forklift(sees):
    """Each robot's reading of each bay after the forklift: 'x' blocked, 'o'
    clear, with the time it was observed, 0 for the shift map."""
    maps = {}
    for a in L.ROBOTS:
        maps[a] = {}
        for t in L.BAYS:
            v = 'x' if t in L.SHIFT_BLOCKED else 'o'
            at = 0
            if t in L.CHANGES and sees[a][t] == 'all':
                v = 'x' if L.CHANGES[t] == 'stage' else 'o'
                at = 1 + list(L.CHANGES).index(t)
            maps[a][t] = (v, at)
    return maps


def broadcast(maps, rule, sees, looking=True):
    """Every robot sends every other its map of every bay, sender by sender.
    Returns the maps after, the number sent, and the number of writes that
    made a correct reading stale."""
    maps = {a: dict(m) for a, m in maps.items()}
    actual = set(L.after_forklift())
    truth = {t: ('x' if t in actual else 'o') for t in L.BAYS}

    def look():
        if not looking:
            return
        for a in L.ROBOTS:
            for t in L.CHANGES:
                if sees[a][t] == 'all':
                    maps[a][t] = (truth[t], 1 + list(L.CHANGES).index(t))

    sends = regressions = 0
    for i in L.ROBOTS:
        look()
        for j in L.ROBOTS:
            if i == j:
                continue
            for t in L.BAYS:
                sends += 1
                mine, theirs = maps[j][t], maps[i][t]
                if rule == 'overwrite' or (rule == 'recency' and theirs[1] > mine[1]):
                    if mine[0] == truth[t] and theirs[0] != truth[t]:
                        regressions += 1
                    maps[j][t] = theirs
                # confidence: 0 against 100 is a tie, and the receiver keeps.
    look()
    return maps, sends, regressions


def policy(maps):
    """The radio policy: t3 to each hauler whose map has it wrong, from a
    robot whose map has it right."""
    maps = {a: dict(m) for a, m in maps.items()}
    fresh = [a for a in L.ROBOTS if maps[a]['t3'][0] == 'o']
    sends = 0
    for h in L.HAULERS:
        if maps[h]['t3'][0] != 'o':
            maps[h]['t3'] = maps[fresh[0]]['t3']
            sends += 1
    return maps, sends


def path_cells(grid, start, goal):
    """A shortest route over the inflated @p grid, as world points."""
    passable = ~C.inflate(grid, C.INFLATION)
    dist = C.flood(passable, goal)
    c, r = C.to_cell(*start)
    if dist[r, c] < 0:
        return None
    out = [C.centre(c, r)]
    while dist[r, c] > 0:
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < dist.shape[0] and 0 <= nc < dist.shape[1] \
                    and dist[nr, nc] == dist[r, c] - 1:
                r, c = nr, nc
                break
        out.append(C.centre(c, r))
    return out


def drive(h, held, through=None, parked=()):
    """Drive hauler @p h by its map @p held ({bay: 'x'|'o'}), and return
    (delivered, bays entered, its map at the end, what happened)."""
    held = dict(held)
    x, y, _ = L.ROBOTS[h]
    pos = (x, y)
    goal = L.DROPS[h]
    actual = set(L.after_forklift())
    cleared = {t: set() for t, k in L.CHANGES.items() if k == 'clear'}
    load = {t: C.load_cells(t) for t in L.CHANGES}
    entered, story = [], []
    world = C.plan(sorted(actual))
    for _ in range(6):
        believed = [t for t in L.BAYS if held[t] == 'x' or (through and t != through)]
        grid = C.plan(believed)
        for (px, py) in parked:
            C.fill(grid, (px - 0.3, py - 0.3, px + 0.3, py + 0.3), C.OCCUPIED)
        route = path_cells(grid, pos, goal)
        if route is None:
            story.append('no route on its map')
            return False, entered, held, story
        replanned = False
        for k, p in enumerate(route):
            for t in L.BAYS:
                x0, y0, x1, y1 = L.bay_box(t)
                if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and t not in entered:
                    entered.append(t)
            # The world stops a hauler at a load its map did not show.
            c, r = C.to_cell(*p)
            if world[r, c] == C.OCCUPIED:
                for t in L.BAYS:
                    x0, y0, x1, y1 = L.bay_box(t)
                    if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                        held[t] = 'x'
                story.append('stopped at a load its map did not show')
                pos = route[max(0, k - 6)]
                replanned = True
            if k % 5 == 0 or replanned:
                for t in L.CHANGES:
                    if L.CHANGES[t] == 'stage':
                        g = C.plan(sorted(actual))
                        # A map reads a bay blocked at its first occupied
                        # cell: one is enough here, where a station's claim
                        # asks for a margin.
                        n = sum(C.visible(g, p, cell) for cell in load[t])
                        if n >= 1 and held[t] != 'x':
                            held[t] = 'x'
                            story.append(f'its laser reads {t} blocked on the way')
                            replanned = True
                    else:
                        g = C.plan(sorted(actual - {t}))
                        cleared[t] |= {cell for cell in load[t] if C.visible(g, p, cell)}
                        if len(cleared[t]) == len(load[t]) and held[t] != 'o':
                            held[t] = 'o'
                            story.append(f'its laser reads {t} clear on the way')
                            replanned = True
            if replanned:
                pos = p if world[C.to_cell(*p)[1], C.to_cell(*p)[0]] != C.OCCUPIED else pos
                break
        else:
            return True, entered, held, story
    story.append('gave up')
    return False, entered, held, story


def floor_instance(sees):
    """The floor as an instance of the communication study."""
    study = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'study')
    sys.path.insert(0, study)
    import instances as SI  # noqa: E402
    changes = [SI.Change(t, kind, {a for a in L.ROBOTS if sees[a][t] == 'all'},
                         10.0 + 10.0 * k)
               for k, (t, kind) in enumerate(sorted(L.CHANGES.items(), key=lambda kv: kv[0] != 't3'))]
    return SI.Instance('secrecy', 1, list(L.ROBOTS), list(L.BAYS), set(L.SHIFT_BLOCKED), changes,
                       list(L.HAULERS), contractors=set(L.CONTRACTORS), secret_bays=set(L.SECRET))


def protocol_run(fleet, sees):
    study = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'study')
    sys.path.insert(0, study)
    import protocols as SP  # noqa: E402
    proto, budget = PROTOCOL[fleet]
    return SP.PROTOCOLS[proto](floor_instance(sees), 'recency', budget, seed=1)


def predict(fleet, sees, looking=True):
    maps = after_forklift(sees)
    regressions = 0
    sequence, requests = [], 0
    if fleet in ('epistemic', 'secret'):
        maps, sends = policy(maps)
    elif fleet in PROTOCOL:
        run = protocol_run(fleet, sees)
        maps = {a: {t: (r.value, r.version) for t, r in run.maps[a].items()} for a in L.ROBOTS}
        sends, requests = run.sent, run.requests
        sequence = [list(x) for x in run.log]
        regressions = run.overwrote_fresh
    else:
        maps, sends, regressions = broadcast(
            maps, {'fuse': 'confidence'}.get(fleet, fleet), sees, looking)
    actual = set(L.after_forklift())
    stale_before = sum((maps[a][t][0] == 'x') != (t in actual) for a in L.ROBOTS for t in L.BAYS)
    held_after = {a: ''.join(maps[a][t][0] for t in L.BAYS) for a in L.ROBOTS}
    hauls, parked = {}, []
    for h in L.HAULERS:
        start = {t: maps[h][t][0] for t in L.BAYS}
        ok, entered, end, story = drive(
            h, start, 't3' if fleet in ('epistemic', 'secret') else None, parked)
        hauls[h] = {'delivered': ok, 'entered': entered, 'map_at_start': ''.join(start.values()),
                    'map_at_end': ''.join(end[t] for t in L.BAYS), 'story': story}
        parked.append(L.DROPS[h] if ok else L.ROBOTS[h][:2])
    leak = [c for c in L.CONTRACTORS for t in L.SECRET if maps[c][t][0] == 'x']
    return {'fleet': fleet, 'sends': sends, 'regressions': regressions,
            'sequence': sequence, 'requests': requests, 'leak': leak,
            'maps_after_exchange': held_after,
            'stale_after_exchange': stale_before, 'hauls': hauls,
            'delivered': [h for h in L.HAULERS if hauls[h]['delivered']]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--json', help='write the predictions here')
    ap.add_argument('--fleets', nargs='*', default=list(FLEETS + SECRET_FLEETS))
    ap.add_argument('--transient', action='store_true',
                    help='the robots that saw a change stop looking at it before the maps are sent')
    args = ap.parse_args()
    sees = C.sight_lines()
    out = {}
    for fleet in args.fleets:
        p = predict(fleet, sees, looking=not args.transient)
        out[fleet] = p
        print(f'{fleet:10s} {p["sends"]:4d} maps of a bay sent, {p["regressions"]:2d} of them over a '
              f'fresh reading; {p["stale_after_exchange"]:2d} stale entries after; '
              f'delivered: {" ".join(p["delivered"]) or "none"}' +
              (f'; secret held by {" ".join(p["leak"])}' if p['leak'] else ''))
        print('           maps (t1 t2 t3): ' + ' '.join(f'{a}:{m}' for a, m in p['maps_after_exchange'].items()))
        for h, d in p['hauls'].items():
            print(f'           {h}: starts {d["map_at_start"]}, enters {" ".join(d["entered"]) or "no bay"}; '
                  f'{"; ".join(d["story"]) or "straight there"}; '
                  f'{"delivered" if d["delivered"] else "not delivered"}')
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
