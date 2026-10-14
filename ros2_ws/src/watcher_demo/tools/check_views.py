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
Who can see the carrier, where, by straight lines against the walls.

    check_views.py [--svg FILE]

The domain declares, per action, who observes it. This checks the hall
against those declarations before any simulator runs: it walks every route
r1 can drive, a sample every 5 cm, and for each sample asks which of the three
observers -- r2 at its station, a watcher at the west post, a watcher at the
east post -- has a clear line to any point of r1's body. A wall blocks a line;
the robots do not, which is the cautious reading.

What must hold:
  - r2 sees both gates and both window spots;
  - the west post sees the west gate and the west window spot, and the east
    post the east ones;
  - no post sees anything of the corridor, of a bay near its drop, of the
    other hall, or of a back route once it has left its hall;
  - the drive to a window, and to a gate, is seen by the post on its own side
    and by no other.
Exits non-zero if any fails.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

BODY = 0.09


def seg_box(p, q, box):
    """Does the segment p-q cross the box (cx, cy, sx, sy)? Liang-Barsky."""
    cx, cy, sx, sy = box
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for pk, qk in ((-dx, p[0] - x0), (dx, x1 - p[0]), (-dy, p[1] - y0), (dy, y1 - p[1])):
        if abs(pk) < 1e-12:
            if qk < 0:
                return False
        else:
            t = qk / pk
            if pk < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
            if t0 > t1:
                return False
    return t1 > 1e-6 and t0 < 1 - 1e-6


WALLS = L.walls()


def sees(eye, p):
    for k in range(8):
        a = k * math.pi / 4
        q = (p[0] + BODY * math.cos(a), p[1] + BODY * math.sin(a))
        if not any(seg_box(eye, q, w) for w in WALLS):
            return True
    return False


OBSERVERS = {'r2': L.STATION[:2], 'post_w': L.post('w')[:2], 'post_e': L.post('e')[:2]}


def samples(path):
    nodes, _ = L.roadmap()
    pts = []
    for u, v in zip(path, path[1:]):
        a, b = nodes[u], nodes[v]
        n = max(1, int(math.dist(a, b) / 0.05))
        pts += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    pts.append(nodes[path[-1]])
    return pts


def seen_by(path):
    out = {o: 0 for o in OBSERVERS}
    for p in samples(path):
        for o, eye in OBSERVERS.items():
            out[o] += sees(eye, p)
    return out


def clear(path):
    """Every sample of the route at least 0.2 m from every wall."""
    worst = math.inf
    for p in samples(path):
        for cx, cy, sx, sy in WALLS:
            dx = max(abs(p[0] - cx) - sx / 2, 0.0)
            dy = max(abs(p[1] - cy) - sy / 2, 0.0)
            worst = min(worst, math.hypot(dx, dy))
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svg', default='')
    a = ap.parse_args()
    bad = []

    def expect(what, ok):
        print(f'  {"ok  " if ok else "FAIL"} {what}')
        if not ok:
            bad.append(what)

    print('the spots')
    for b in ('x', 'y'):
        s = L.bay_side(b)
        o = 'e' if s == 'w' else 'w'
        g = L.gate(b)
        expect(f'r2 sees gate {b}', sees(OBSERVERS['r2'], g))
        expect(f'post {s} sees gate {b}', sees(OBSERVERS[f'post_{s}'], g))
        expect(f'post {o} does not see gate {b}', not sees(OBSERVERS[f'post_{o}'], g))
    for s in ('w', 'e'):
        o = 'e' if s == 'w' else 'w'
        w = L.window(s)
        expect(f'r2 sees window {s}', sees(OBSERVERS['r2'], w))
        expect(f'post {s} sees window {s}', sees(OBSERVERS[f'post_{s}'], w))
        expect(f'post {o} does not see window {s}', not sees(OBSERVERS[f'post_{o}'], w))

    print('the routes: samples seen by r2 / west post / east post, and the least clearance')
    for s in ('w', 'e'):
        o = 'e' if s == 'w' else 'w'
        for b in ('x', 'y'):
            p = L.route_back(s, b)
            k = p.index(f'c_in_{s}')
            hall, rest = seen_by(p[:k + 1]), seen_by(p[k:])
            print(f'    back from window {s} to bay {b}: to the corridor {hall}, beyond {rest}, '
                  f'clearance {clear(p):.2f} m')
            expect(f'back from window {s} to bay {b}: up to the corridor unseen by post {o}', hall[f'post_{o}'] == 0)
            expect(f'back from window {s} to bay {b}: in the corridor and the bay unseen by either post',
                   rest['post_w'] == 0 and rest['post_e'] == 0)
            expect(f'back from window {s} to bay {b} clear of walls', clear(p) >= 0.2)
        p = L.route_to_window(s)
        v = seen_by(p)
        print(f'    to window {s}: {v}, clearance {clear(p):.2f} m')
        expect(f'drive to window {s} unseen by post {o}', v[f'post_{o}'] == 0)
        expect(f'drive to window {s} clear of walls', clear(p) >= 0.2)
    for b in ('x', 'y'):
        s = L.bay_side(b)
        o = 'e' if s == 'w' else 'w'
        for name, p in (('to gate', L.route_to_gate(b)), ('gate into bay', L.route_gate_in(b))):
            v = seen_by(p)
            print(f'    {name} {b}: {v}, clearance {clear(p):.2f} m')
            expect(f'{name} {b} unseen by post {o}', v[f'post_{o}'] == 0)
            expect(f'{name} {b} clear of walls', clear(p) >= 0.2)
    print('the corridor and the bays, on a grid')
    for name, (x0, y0, x1, y1) in (('corridor', (L.HALL[0] + 0.2, L.HALL[1] + 0.2, L.HALL[2] - 0.2, L.CORRIDOR_N - 0.75)),):
        n = 0
        for i in range(141):
            for j in range(11):
                p = (x0 + (x1 - x0) * i / 140, y0 + (y1 - y0) * j / 10)
                n += sees(OBSERVERS['post_w'], p) or sees(OBSERVERS['post_e'], p)
        expect(f'no post sees the {name} south of the baffles ({n} points seen)', n == 0)
    for b in ('x', 'y'):
        x0, y0, x1, y1 = L.bay_region(b)
        n = 0
        drop = L.roadmap()[0][f'drop_{L.bay_side(b)}']
        for i in range(21):
            for j in range(21):
                p = (x0 + 0.3 + (x1 - x0 - 0.6) * i / 20, y0 + 0.3 + (y1 - y0 - 0.6) * j / 20)
                if math.dist(p, drop) < 1.0:
                    n += sees(OBSERVERS['post_w'], p) or sees(OBSERVERS['post_e'], p)
        expect(f'no post sees within 1 m of the drop in bay {b} ({n} points seen)', n == 0)
    if a.svg:
        write_svg(a.svg)
    print('all views as declared' if not bad else f'{len(bad)} failed')
    return 1 if bad else 0


def write_svg(path):
    """A plan of the hall: walls, the routes and the three observers."""
    x0, y0, x1, y1 = L.HALL
    k = 60

    def X(x):
        return (x - x0 + 0.4) * k

    def Y(y):
        return (y1 - y + 0.4) * k
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{(x1 - x0 + 0.8) * k:.0f}" '
           f'height="{(y1 - y0 + 0.8) * k:.0f}">']
    for cx, cy, sx, sy in WALLS:
        out.append(f'<rect x="{X(cx - sx / 2):.1f}" y="{Y(cy + sy / 2):.1f}" width="{sx * k:.1f}" '
                   f'height="{sy * k:.1f}" fill="#555"/>')
    nodes, edges = L.roadmap()
    for u, v in edges:
        out.append(f'<line x1="{X(nodes[u][0]):.1f}" y1="{Y(nodes[u][1]):.1f}" x2="{X(nodes[v][0]):.1f}" '
                   f'y2="{Y(nodes[v][1]):.1f}" stroke="#bbb" stroke-dasharray="4 3"/>')
    for name, (x, y) in OBSERVERS.items():
        out.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="7" fill="{"#1f60c8" if name == "r2" else "#c0392b"}"/>')
    out.append('</svg>')
    open(path, 'w').write('\n'.join(out))


if __name__ == '__main__':
    sys.exit(main())
