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
The Gazebo arena: the paper's laboratory (Section 7) at twice its size.

Bramblett and Bezzo ran two or three ROSbots in a 4 by 5.5 m space with
convex obstacles, with a sensing and communication range of 1 m. Here three
TurtleBot3 Burgers, whose top speed is 0.22 m/s, run in an 8 by 11 m space
with ranges of 2 m: the same proportions, at a scale at which a run lasts a
few minutes. Five crates of 0.5 m stand in it at places the seed draws,
unknown to the robots until they see them; the base is at the south wall.

Every parameter of the coverage that has a length is the kinematic
simulator's (env.SIM) scaled by the ratio of the sensing ranges, 2/5: the
turn term, the penalty Delta of eq. (10) and the gain term. The cells are
0.25 m.

Case I is the paper's Fig. 10: one task, which needs two robots or all
three. Case II is its Fig. 11: two tasks, one for two robots and one for
three, so that the three robots are needed by two tasks. A task is done when
the robots it needs have stood at it together for SPEC.work seconds; the
two-robot task of Case II takes WORK_LONG, so that the robots doing it are
away from their particles for a while (the paper's tasks take no time, and it
names tasks of other lengths as future work).
"""

import dataclasses
import json
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gossip import env as E  # noqa: E402

LAMBDA = 2.0 / 5.0
SPEC = dataclasses.replace(
    E.SIM, width=8.0, height=11.0, cell=0.25, sense=2.0, radio=2.0, v_max=0.20, v_particle=0.13,
    noise=0.0, r_task=0.7, work=6.0, home=(4.0, 0.9), home_radius=1.2, dt=0.1,
    penalty=E.SIM.penalty * LAMBDA, heading=E.SIM.heading * LAMBDA, gain=E.SIM.gain / LAMBDA,
    rho_o=0.4, meet=0.45, resync=10.0, limit=900.0, slot=0.35, inside=1.0)

ROBOTS = ('r1', 'r2', 'r3')
CAP = {'r1': (0.12, 0.38, 0.80), 'r2': (0.90, 0.42, 0.08), 'r3': (0.20, 0.62, 0.30)}
CRATE = 0.5
WORK_LONG = 40.0      # s, the two-robot task of Case II
N_CRATES = 5
CASES = {1: 'one task, for two robots or all three (the paper\'s Fig. 10)',
         2: 'a task for two robots and a task for three (the paper\'s Fig. 11)'}


def start_poses():
    h = SPEC.home
    return {r: (h[0] - 0.6 + 0.6 * i, h[1] - 0.25, math.pi / 2) for i, r in enumerate(ROBOTS)}


def scenario(seed, case):
    """Crates and tasks of one run, from its seed. In Case II the seed's
    draws are screened, in the kinematic simulator under the paper's own
    procedure, until the three-robot task is found while the two-robot task
    is being done: the situation in which the same robots are needed by two
    tasks, which the paper's rules are not guaranteed to handle. The world
    is then the same for every formulation."""
    if case != 2:
        return draw(seed, case, 0)
    from gossip import sim as S
    for sub in range(200):
        scn = draw(seed, case, sub)
        out = S.run(area(scn), len(ROBOTS), case, 'proposed', seed)
        t1, t2 = out['tasks']['t1'], out['tasks']['t2']
        if t1['found_t'] and t2['found_t'] and t1['done_t'] and t1['found_t'] + 5.0 < t2['found_t'] < t1['done_t']:
            scn['draw'] = sub
            return scn
    raise RuntimeError(f'no draw of seed {seed} has the conflict')


def draw(seed, case, sub):
    rng = random.Random(seed * 7919 + case * 104729 + sub)
    cell = SPEC.cell
    nx, ny = int(round(SPEC.width / cell)), int(round(SPEC.height / cell))
    ks = [rng.choice((2, 3))] if case == 1 else [2, 3]
    # in Case II the task for two robots is long (an inspection, say): its
    # robots are away from their particles for a while
    work = [SPEC.work] if case == 1 else [WORK_LONG, SPEC.work]
    tasks = []
    while len(tasks) < len(ks):
        p = (round(rng.uniform(1.0, SPEC.width - 1.0) / cell) * cell + cell / 2,
             round(rng.uniform(4.0, SPEC.height - 1.0) / cell) * cell + cell / 2)
        if math.dist(p, SPEC.home) < 4.0 or any(math.dist(p, t['pos']) < 3.0 for t in tasks):
            continue
        tasks.append({'id': f't{len(tasks) + 1}', 'pos': p, 'k': ks[len(tasks)], 'work': work[len(tasks)]})
    crates = []
    while len(crates) < N_CRATES:
        # a crate covers two by two cells, on the grid
        # at least a metre from every wall: no sliver behind a crate
        ix, iy = rng.randrange(4, nx - 5), rng.randrange(8, ny - 5)
        c = ((ix + 1) * cell, (iy + 1) * cell)
        if math.dist(c, SPEC.home) < 2.0 or any(math.dist(c, t['pos']) < 1.0 for t in tasks):
            continue
        if any(abs(c[0] - d[0]) < 1.2 and abs(c[1] - d[1]) < 1.2 for d in crates):
            continue
        crates.append(c)
    return {'seed': seed, 'case': case, 'tasks': tasks, 'crates': crates, 'about': CASES[case]}


def occupancy(scn):
    cell = SPEC.cell
    nx, ny = int(round(SPEC.width / cell)), int(round(SPEC.height / cell))
    occ = np.zeros((nx, ny), dtype=bool)
    half = CRATE / 2
    for (cx, cy) in scn['crates']:
        for ix in range(int(round((cx - half) / cell)), int(round((cx + half) / cell))):
            for iy in range(int(round((cy - half) / cell)), int(round((cy + half) / cell))):
                occ[ix, iy] = True
    return occ


def area(scn):
    return E.Area(SPEC, occupancy(scn), [dict(t, pos=tuple(t['pos'])) for t in scn['tasks']], seed=scn['seed'])


def load(run_dir):
    return json.load(open(os.path.join(run_dir, 'world.json')))
