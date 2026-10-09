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

    make_mapping.py --out pddl/stale-maps-mapping.json

The epistemic planner names a ground action by its schema and its arguments
joined with underscores, `send-open_r6_r2_t3`; the executor dispatches
classical actions, `(send_open r6 r2 t3)`. This is the table between them,
for every robot and bay in layout.py. The doubting floor's actions are not in
it: that floor is solved and traced, not run.
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

    table = {}
    for t in L.BAYS:
        table[f'stage_{t}'] = {'action': f'(stage {t})', 'duration': 20.0}
        table[f'clear_{t}'] = {'action': f'(clear {t})', 'duration': 20.0}
        for i in L.ROBOTS:
            table[f'cross_{i}_{t}'] = {'action': f'(cross {i} {t})', 'duration': 90.0}
            for j in L.ROBOTS:
                if i == j:
                    continue
                table[f'send-open_{i}_{j}_{t}'] = {'action': f'(send_open {i} {j} {t})', 'duration': 3.0}
                table[f'send-blocked_{i}_{j}_{t}'] = {
                    'action': f'(send_blocked {i} {j} {t})', 'duration': 3.0}
    with open(args.out, 'w') as fh:
        json.dump(dict(sorted(table.items())), fh, indent=2)
        fh.write('\n')
    print(f'{args.out}: {len(table)} ground actions')


if __name__ == '__main__':
    main()
