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
Checks what the domain assumes of the area, before any run.

    check_layout.py

1. Every way a ground vehicle drives (its coverage route, its flock path and
   every edge of the roadmap) keeps CLEAR metres from every obstacle.
2. The domain treats "g stands at particle b" as a fact a hail at particle b
   decides. That needs the three particles of a ground vehicle to be further
   apart than the radio's range, with margin, whenever a hail can happen: from
   the earliest moment the task can be found to HAIL_UNTIL, after the last hail
   of any run. Later the particles close on the end of the route, where the
   paper's would meet.
3. Every site lies on the aerial vehicle's route, and is found after the last
   fault any seed has, so that the policy's model holds when it is computed.
4. In the flock, both ground vehicles stay within radio range of the aerial
   vehicle on the two passes, and the aerial vehicle's own path passes every
   site within R_DETECT, so that the flock finds the task when a ground
   vehicle has left it.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402
import worlds as W  # noqa: E402

CLEAR = 0.75
MARGIN = 1.0
HAIL_UNTIL = 420.0


def box_distance(p, o):
    cx, cy, sx, sy, _ = o
    dx = max(abs(p[0] - cx) - sx / 2, 0.0)
    dy = max(abs(p[1] - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def path_clearance(path):
    worst = float('inf')
    for a, b in zip(path, path[1:]):
        n = max(2, int(math.dist(a, b) / 0.05))
        for k in range(n + 1):
            p = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            worst = min(worst, min(box_distance(p, o) for o in L.OBSTACLES))
    return worst


def found_at(site):
    """Seconds after the start at which the aerial vehicle, on its route, first
    comes within R_DETECT of the site."""
    route, s = L.ROUTE['u'], 0.0
    while s <= L.length(route):
        if math.dist(L.at(route, s)[0], site) <= L.R_DETECT:
            return s / L.V_UAV
        s += 0.02
    return None


def main():
    ok = True

    def check(good, text):
        nonlocal ok
        print(f'  {"ok  " if good else "FAIL"} {text}')
        ok &= good

    print('1. clearance')
    for g in L.UGVS:
        check(path_clearance(L.ROUTE[g]) >= CLEAR, f'{g} route keeps {path_clearance(L.ROUTE[g]):.2f} m')
        check(path_clearance(L.FLOCK[g]) >= CLEAR, f'{g} flock path keeps {path_clearance(L.FLOCK[g]):.2f} m')
    _, edges = L.roadmap()
    worst = min(path_clearance([a, b]) for a, b in edges)
    check(worst >= CLEAR, f'{len(edges)} roadmap edges keep {worst:.2f} m')

    print('2. particles apart')
    first = min(found_at(s) for s in L.SITES)
    for g in L.UGVS:
        closest = float('inf')
        t = first
        while t <= HAIL_UNTIL:
            ps = [L.particle(g, b, t) for b in (1, 2, 3)]
            for i in range(3):
                for j in range(i + 1, 3):
                    closest = min(closest, math.dist(ps[i], ps[j]))
            t += 0.5
        check(closest >= L.R + MARGIN,
              f'{g}: particles at least {closest:.2f} m apart from {first:.0f} to {HAIL_UNTIL:.0f} s '
              f'(range {L.R} m)')

    print('3. sites')
    last_fault = max(t for s in range(1, W.SEEDS + 1) for ts in W.scenario(s)['fault_times'].values() for t in ts)
    for site in L.SITES:
        t = found_at(site)
        check(t is not None and t > last_fault + 10.0, f'site {site}: found {t:.1f} s after the start '
              f'(last fault {last_fault:.1f} s)')

    print('4. the flock')
    worst, s = 0.0, 0.0
    ref = L.FLOCK['u']
    pass_start = L.length(ref[:4])
    while s <= L.length(ref):
        u = L.flock_point('u', s)
        if s >= pass_start:
            worst = max(worst, max(math.dist(u, L.flock_point(g, s)) for g in L.UGVS))
        s += 0.05
    check(worst <= L.R, f'on the passes, each ground vehicle at most {worst:.2f} m from the aerial one')
    for site in L.SITES:
        near = min(math.dist(L.flock_point('u', k * 0.05), site) for k in range(int(L.length(ref) / 0.05)))
        check(near <= L.R_DETECT, f'site {site}: the aerial vehicle\'s flock path passes {near:.2f} m from it')
    print('all checks pass' if ok else 'SOME CHECKS FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
