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
How the formulations fare as the porter's habits change.

    sensitivity.py [--out FILE.json]

Each formulation's policy is planned once, from its own ranks, as the runs
plan it; it is then charged its predicted time (tools/plan.py) in every night,
and the expectation taken under porter frequencies other than the stated ones:
the west loop on a share w of nights, the four rare loops on r between them,
the rostered loop on the rest, and at two the reopening on a share o. The
bound is the policy each courier would choose knowing those frequencies
exactly, planned anew at every point.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan as P  # noqa: E402
import ranks as R  # noqa: E402


def weights(act, w, r, o=8 / 9):
    out = {}
    for lp, p in R.worlds(act):
        f = (1 - w - r) if lp == R.ROSTERED else w if lp == ('a', 'b') else r / 4
        if p is not None:
            f *= o if p == 'reopen' else 1 - o
        out[(lp, p)] = f
    return out


def expected(act, tree, wts):
    return sum(wts[x] * P.timeline(tree, x)[0] for x in R.worlds(act))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    a = ap.parse_args()
    trees = {(act, f): P.Team(act, f).tree() for act in (1, 2, 3) for f in R.FORMULATIONS}
    result = {}
    for act in (1, 2, 3):
        rows = []
        for w in [0.0, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4]:
            for r in (0.02, 0.05, 0.1, 0.2):
                if w + r >= 0.95:
                    continue
                wts = weights(act, w, r)
                row = {'w': w, 'r': r}
                for f in R.FORMULATIONS:
                    row[f] = round(expected(act, trees[(act, f)], wts), 1)
                row['bound'] = round(expected(act, P.Team(act, 'knowledge', wts).tree(), wts), 1)
                rows.append(row)
        result[act] = rows
        for row in rows:
            if row['r'] == 0.05:
                best = min(R.FORMULATIONS, key=lambda f: row[f])
                print(f"act {act} w={row['w']:.3f} r={row['r']:.2f}: " +
                      '  '.join(f'{f[:5]} {row[f]:6.1f}' for f in R.FORMULATIONS) + f"  bound {row['bound']:6.1f}  best {best}")
    if a.out:
        json.dump(result, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
