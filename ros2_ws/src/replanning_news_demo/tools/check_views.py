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
What the domain assumes about the residence, checked by casting rays.

    check_views.py [--png FILE]

  1. From the mouth of the west and of the east passage, a scan decides that
     passage's door, open and shut, whatever state the other doors are in.
  2. From the mouth of the middle passage nothing decides its door: the
     passage turns, and the scan is the same with the door open or shut.
  3. From the hatch, a scan decides the middle door.
  4. Every route keeps ROUTE_CLEAR from walls and furniture, with every door
     open; the crossings are checked with their own door open.
  5. The robots' start poses are clear.

--png draws the grid, the routes and the places.
"""

import argparse
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid as G  # noqa: E402
import layout as L  # noqa: E402

ROUTE_CLEAR = 0.30


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--png')
    a = ap.parse_args()
    ok = True
    states = [dict(zip('wme', bits)) for bits in itertools.product((True, False), repeat=3)]
    nodes, south, north = L.roadmap()

    for d, place in (('w', 'vw'), ('e', 've'), ('m', 'ks')):
        x, y = nodes[place]
        wrong = []
        for s in states:
            got = G.decide(d, G.scan(G.world_grid(s), x, y))
            want = 'open' if s[d] else 'shut'
            if got != want:
                wrong.append(f"{''.join(k.upper() if v else k for k, v in s.items())}->{got or 'nothing'}")
        print(f"{'ok  ' if not wrong else 'FAIL'} from {place}, door {d} decided in all 8 states"
              + (f": {wrong}" if wrong else ''))
        ok &= not wrong

    x, y = nodes['vm']
    seen = {G.decide('m', G.scan(G.world_grid(s), x, y)) for s in states}
    hits = {tuple(sorted(G.scan(G.world_grid(s), x, y))) == tuple(sorted(G.scan(G.world_grid(dict(s, m=not s['m'])), x, y)))
            for s in states}
    good = seen == {''} and hits == {True}
    print(f"{'ok  ' if good else 'FAIL'} from vm, the middle door is not decided (decisions {seen}, "
          f"scan unchanged by the door: {hits == {True}})")
    ok &= good

    g_open = G.world_grid({'w': True, 'm': True, 'e': True})
    for name, graph in (('south', south), ('north', north)):
        for u, v in graph:
            c = G.clearance(g_open, [nodes[u], nodes[v]])
            if c < ROUTE_CLEAR:
                print(f"FAIL {name} edge {u}-{v} passes {c:.2f} m from something")
                ok = False
    for d, places in L.CROSS.items():
        c = G.clearance(g_open, [nodes[p] for p in places])
        print(f"{'ok  ' if c >= ROUTE_CLEAR else 'FAIL'} crossing {d}: {c:.2f} m clear")
        ok &= c >= ROUTE_CLEAR
    for r, (x, y, _) in L.START.items():
        c = G.clearance(g_open, [(x, y), (x + 1e-3, y)])
        print(f"{'ok  ' if c >= 0.25 else 'FAIL'} {r} starts {c:.2f} m clear")
        ok &= c >= 0.25

    if a.png:
        draw(a.png, g_open, nodes, south + north)
    print('all views and routes as the domain assumes' if ok else 'SOMETHING IS WRONG')
    sys.exit(0 if ok else 1)


def draw(path, g, nodes, edges):
    from PIL import Image, ImageDraw
    img = Image.fromarray(((~g[::-1]) * 255).astype('uint8')).convert('RGB')
    k = 4
    img = img.resize((img.width * k, img.height * k), Image.NEAREST)
    dr = ImageDraw.Draw(img)

    def px(p):
        return ((p[0] - G.X0) / G.RES * k, (G.NY - (p[1] - G.Y0) / G.RES) * k)
    for u, v in edges:
        dr.line([px(nodes[u]), px(nodes[v])], fill=(60, 120, 220), width=3)
    for d, places in L.CROSS.items():
        dr.line([px(nodes[p]) for p in places], fill=(220, 60, 60), width=3)
    for n, p in nodes.items():
        x, y = px(p)
        dr.ellipse([x - 6, y - 6, x + 6, y + 6], fill=(240, 160, 0))
        dr.text((x + 8, y - 8), n, fill=(0, 0, 0))
    img.save(path)
    print('drawn', path)


if __name__ == '__main__':
    main()
