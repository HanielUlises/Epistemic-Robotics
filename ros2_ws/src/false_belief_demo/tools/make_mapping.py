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
Writes the map from the planner's ground action names to classical actions.

    make_mapping.py --out pddl/false-belief-mapping.json

`relocate_mover_t1_t3` is dispatched as `(relocate mover t1 t3)` and
`report_mover_picker_t1_t3` as `(report mover picker t1 t3)`. Every grounding
plank produces is listed, applicable or not, since the planner reads the map
before it knows which it will use.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

DURATION = {'go-dock': 60.0, 'relocate': 180.0, 'report': 4.0, 'look': 60.0, 'fetch': 120.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    agents = list(L.ROBOTS)
    bays = list(L.BAYS)
    table = {}
    for i in agents:
        table[f'go-dock_{i}'] = {'action': f'(go_dock {i})', 'duration': DURATION['go-dock']}
        for b in bays:
            table[f'look_{i}_{b}'] = {'action': f'(look {i} {b})', 'duration': DURATION['look']}
            table[f'fetch_{i}_{b}'] = {'action': f'(fetch {i} {b})', 'duration': DURATION['fetch']}
        for a in bays:
            for b in bays:
                if a == b:
                    continue
                table[f'relocate_{i}_{a}_{b}'] = {'action': f'(relocate {i} {a} {b})',
                                                  'duration': DURATION['relocate']}
                for j in agents:
                    if i != j:
                        table[f'report_{i}_{j}_{a}_{b}'] = {
                            'action': f'(report {i} {j} {a} {b})', 'duration': DURATION['report']}
    with open(args.out, 'w') as fh:
        json.dump(dict(sorted(table.items())), fh, indent=2)
        fh.write('\n')
    print(f'{args.out}: {len(table)} ground actions')


if __name__ == '__main__':
    main()
