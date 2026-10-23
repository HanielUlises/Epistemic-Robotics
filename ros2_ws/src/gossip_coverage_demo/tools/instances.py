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
The canonical instances of epddl-workspace/gossip-coverage/instances, one per
situation of the paper, written from the geometry each describes.

    instances.py [--out DIR]

hardware_case1    the paper's Fig. 10: three robots, robot 3 finds a task
                  that needs two, and every robot must learn of it (Case I).
hardware_case2    the paper's Fig. 11, its second task: three robots, robot 1
                  finds a task that needs all three while the other two may be
                  busy with the first task (at most two of them).
sim_case1         the paper's Fig. 5: five robots, robot 3 finds a task that
                  needs three; everyone must learn of it.
sim_case2         eight robots, a task for three found by robot 1, at most
                  two of the others busy (Case II).

Who may call whom is the neighbour relation of the Voronoi partition of the
particles' positions, which are given here in metres of the paper's 50 m area
(of its 4 by 5.5 m laboratory for the hardware instances).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gossip import epddl as P  # noqa: E402
from gossip.epistemic import near_relation  # noqa: E402

# where each robot's particle is when the task is found
GEOMETRY = {
    'hardware_case1': {'r3': (3.2, 4.4), 'r1': (1.0, 4.6), 'r2': (2.6, 1.6)},
    'hardware_case2': {'r1': (1.2, 4.8), 'r2': (3.0, 2.2), 'r3': (0.8, 1.4)},
    'sim_case1': {'r3': (12.0, 30.0), 'r4': (22.0, 28.0), 'r2': (6.0, 18.0), 'r5': (36.0, 22.0),
                  'r1': (4.0, 6.0)},
    'sim_case2': {'r1': (25.0, 40.0), 'r2': (15.0, 34.0), 'r3': (35.0, 33.0), 'r4': (8.0, 22.0),
                  'r5': (24.0, 24.0), 'r6': (42.0, 20.0), 'r7': (14.0, 8.0), 'r8': (36.0, 8.0)},
}

SPEC = {
    'hardware_case1': dict(finder='r3', k=2, case=1),
    'hardware_case2': dict(finder='r1', k=3, case=2, may_be_busy=['r2', 'r3'], max_busy=2),
    'sim_case1': dict(finder='r3', k=3, case=1),
    'sim_case2': dict(finder='r1', k=3, case=2, may_be_busy='others', max_busy=2),
}


def instance(name):
    geo = GEOMETRY[name]
    spec = dict(SPEC[name])
    finder = spec.pop('finder')
    # the others by distance from the finder: the order the planner sees
    order = [finder] + sorted((r for r in geo if r != finder),
                              key=lambda r: (geo[r][0] - geo[finder][0]) ** 2 + (geo[r][1] - geo[finder][1]) ** 2)
    if spec.get('may_be_busy') == 'others':
        spec['may_be_busy'] = order[1:]
    return P.Instance(order, finder, spec.pop('k'), spec.pop('case'), near=near_relation(geo), name=name, **spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(P.WORKSPACE, 'instances'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for name in GEOMETRY:
        open(os.path.join(a.out, f'{name}.epddl'), 'w').write(instance(name).text())
        print(os.path.join(a.out, f'{name}.epddl'))


if __name__ == '__main__':
    main()
