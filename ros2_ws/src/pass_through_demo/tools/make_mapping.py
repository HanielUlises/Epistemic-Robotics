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

    make_mapping.py --out pddl/pass-through-mapping.json

The epistemic planner names a ground action by its schema and its arguments
joined with underscores, `share-shut_east_carrier_t3`; the executor dispatches
classical actions, `(tell_shut east carrier t3)`. This is the table between
them, for every agent and bay in layout.py, so the two vocabularies cannot
name different things.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    agents = list(L.ROBOTS)
    bays = sorted(L.TUNNEL_BAYS)
    table = {}
    for i in agents:
        for t in bays:
            table[f'survey_{i}_{t}'] = {'action': f'(survey {i} {t})', 'duration': 5.0}
            table[f'cross_{i}_{t}'] = {'action': f'(cross {i} {t})', 'duration': 30.0}
            for j in agents:
                table[f'share-open_{i}_{j}_{t}'] = {
                    'action': f'(tell_open {i} {j} {t})', 'duration': 2.0}
                table[f'share-shut_{i}_{j}_{t}'] = {
                    'action': f'(tell_shut {i} {j} {t})', 'duration': 2.0}
    with open(args.out, 'w') as fh:
        json.dump(dict(sorted(table.items())), fh, indent=2)
        fh.write('\n')
    print(f'{args.out}: {len(table)} ground actions')


if __name__ == '__main__':
    main()
