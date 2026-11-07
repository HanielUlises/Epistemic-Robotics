#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
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
How the courier's belief reports fare under other trusts and other porters.

    sensitivity.py [--out sensitivity.json]

trust      each act planned again with the listeners' trust ALPHA from 1 to 6:
           the predicted expected time of each formulation's policy, and the
           belief reports the perspective courier makes on each night.
west       each formulation's policy, planned once at the stated frequencies,
           charged its predicted time when the west loop is walked on another
           share of nights (the four rare loops keep 5 % between them).

The times are tools/plan.py's predictions under ePlanSys's parallel groups.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402


def predicted(team, tree, freq):
    rows = P.report(team, tree)
    z = sum(freq.values())
    return sum(freq[w] * t for w, t, *_ in rows) / z


def frequencies(west, rare=0.05):
    f = {}
    for w in R.worlds():
        if w == R.ROSTERED:
            f[w] = 1.0 - west - rare
        elif w == ('a', 'b'):
            f[w] = west
        else:
            f[w] = rare / 4
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    a = ap.parse_args()
    stated = {w: R.frequency(w) for w in R.worlds()}
    out = {'trust': {}, 'west': {}}
    alpha0 = R.ALPHA
    for alpha in range(1, 7):
        R.ALPHA = alpha
        out['trust'][alpha] = {}
        for act in sorted(R.FLEETS):
            row = {}
            for f in R.FORMULATIONS:
                P.Solo.solve.cache_clear()
                team, tree = P.plan(act, f)
                row[f] = round(predicted(team, tree, stated), 1)
                if f == 'perspective':
                    row['said'] = {R.name(w): s for w, _, _, _, s in P.report(team, tree)}
            out['trust'][alpha][act] = row
            print(f"trust {alpha} act {act}: " + '  '.join(f'{f} {row[f]:.1f}' for f in R.FORMULATIONS))
    R.ALPHA = alpha0
    for act in sorted(R.FLEETS):
        trees = {f: P.plan(act, f) for f in R.FORMULATIONS}
        out['west'][act] = {}
        for k in range(0, 9):
            west = 0.05 * k
            fr = frequencies(west)
            out['west'][act][round(west, 2)] = {f: round(predicted(*trees[f], fr), 1) for f in R.FORMULATIONS}
        print(f'west act {act}: ' + '  '.join(f"{w}: " + '/'.join(f'{v:.0f}' for v in row.values())
                                               for w, row in out['west'][act].items()))
    if a.out:
        json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
