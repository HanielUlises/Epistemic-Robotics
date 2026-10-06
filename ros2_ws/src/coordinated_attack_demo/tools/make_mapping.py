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

    make_mapping.py --out pddl/coordinated-attack-mapping.json

    make_mapping.py --domain positions --out pddl/coordinated-attack-positions-mapping.json

`ack2_north_south_s1` is dispatched as `(radio_ack2 north south s1)`, and
`lift_s1` as `(lift south north s1)`: one joint action for both robots. In the
positions domain the signal names its listener, `signal_south_north_s1` as
`(signal_to south north s1)`, and the sighting is `(sight south north)`.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

DURATION = {'read-order': 60.0, 'radio': 4.0, 'go-view': 60.0, 'signal': 6.0, 'sight': 6.0,
            'lift': 90.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--domain', choices=('coordinated-attack', 'positions'),
                    default='coordinated-attack')
    args = ap.parse_args()
    positions = args.domain == 'positions'

    agents = list(L.ROBOTS)
    stands = sorted(L.STANDS)
    table = {}
    for i in agents:
        table[f'go-view_{i}'] = {'action': f'(go_view {i})', 'duration': DURATION['go-view']}
        for s in stands:
            table[f'read-order_{i}_{s}'] = {'action': f'(read_order {i} {s})',
                                            'duration': DURATION['read-order']}
            if not positions:
                table[f'signal_{i}_{s}'] = {'action': f'(signal {i} {s})',
                                            'duration': DURATION['signal']}
            for j in agents:
                if i == j:
                    continue
                for kind in ('tell', 'ack', 'ack2', 'ack3'):
                    table[f'{kind}_{i}_{j}_{s}'] = {'action': f'(radio_{kind} {i} {j} {s})',
                                                    'duration': DURATION['radio']}
                if positions:
                    table[f'signal_{i}_{j}_{s}'] = {'action': f'(signal_to {i} {j} {s})',
                                                    'duration': DURATION['signal']}
    for s in stands:
        table[f'lift_{s}'] = {'action': f'(lift {agents[0]} {agents[1]} {s})',
                              'duration': DURATION['lift']}
    if positions:
        for i in agents:
            for j in agents:
                if i != j:
                    table[f'sight_{i}_{j}'] = {'action': f'(sight {i} {j})',
                                               'duration': DURATION['sight']}
    with open(args.out, 'w') as fh:
        json.dump(dict(sorted(table.items())), fh, indent=2)
        fh.write('\n')
    print(f'{args.out}: {len(table)} ground actions')


if __name__ == '__main__':
    main()
