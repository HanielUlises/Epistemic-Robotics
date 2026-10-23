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
The area, its obstacles and tasks, and what a range sensor sees in it.

An area is a grid of square cells over a rectangle whose lower left corner is
the origin. A cell is free or occupied; the robots know none of the occupied
ones at the start, and the walls of the rectangle are the only obstacles every
robot knows. Maps are kept per robot as int8 grids: UNKNOWN, FREE or OCC.

Spec holds every parameter of a scenario. SIM is the paper's simulations
(Section 6): a 50 by 50 m area, 1 m obstacles, a 5 m lidar, robots of 3 m/s;
the radio range is the 10 m of the flock method, which the paper does not
restate for its own.
"""

import dataclasses
import math

import numpy as np

UNKNOWN, FREE, OCC = -1, 0, 1


@dataclasses.dataclass
class Spec:
    width: float = 50.0
    height: float = 50.0
    cell: float = 1.0
    sense: float = 5.0          # m, range sensor
    radio: float = 10.0         # m, communication disk
    v_max: float = 3.0          # m/s, a robot's top speed
    v_particle: float = 2.7     # m/s, a particle's speed, below the top so that a robot can catch it
    noise: float = 0.2          # m/s, standard deviation of the process noise, per axis
    r_task: float = 1.5         # m, within this of a task a robot is at it
    work: float = 0.0           # s, a task is done this long after its last robot arrives
    home: tuple = (25.0, 1.5)
    home_radius: float = 2.5
    dt: float = 0.1
    penalty: float = 20.0       # m, Delta of eq. (10): a frontier point outside a particle's cell
    heading: float = 2.0        # m per radian of turn, the utility's heading term
    gain: float = 0.05          # m per square metre a particle would newly see from the point
    reselect: float = 1.0       # s, a particle chooses its frontier point again this often
    rho_o: float = 1.0          # m, the potential field's range of influence
    eta_g: float = 1.0          # attraction gain, eq. (6)
    eta_o: float = 5.0          # repulsion gain, eq. (7)
    meet: float = 1.0           # m, a seeker this close to a particle with nobody in range calls it empty
    resync: float = 10.0        # s, while all are in contact the common belief is reset at least this often
    limit: float = 900.0        # s, a run that has not ended by then is cut
    slot: float = 0.0           # m, robots stand this far from a task or the base, each at its own bearing


SIM = Spec()


class Area:
    """The true area: occupancy, the home base and the tasks."""

    def __init__(self, spec, occ, tasks, seed=None):
        self.spec = spec
        self.occ = occ.astype(bool)
        self.nx, self.ny = occ.shape
        self.tasks = tasks
        self.seed = seed
        self.vis = Visibility(spec)

    def cell_of(self, p):
        c = self.spec.cell
        return (min(self.nx - 1, max(0, int(p[0] / c))), min(self.ny - 1, max(0, int(p[1] / c))))

    def center(self, ix, iy):
        c = self.spec.cell
        return ((ix + 0.5) * c, (iy + 0.5) * c)

    def inside(self, p):
        return 0.0 <= p[0] < self.spec.width and 0.0 <= p[1] < self.spec.height

    def blocked(self, p):
        return not self.inside(p) or self.occ[self.cell_of(p)]

    def walls_map(self):
        """The map every robot starts with: nothing known."""
        return np.full((self.nx, self.ny), UNKNOWN, dtype=np.int8)

    def sense(self, p, known, seen):
        """A range sensor at p, against the true occupancy: marks in `known`
        the cells it sees, free or occupied, and in `seen` the same cells.
        Returns the indices of the cells seen."""
        ix, iy = self.cell_of(p)
        cells = self.vis.visible(ix, iy, self.occ)
        known[cells] = np.where(self.occ[cells], OCC, FREE)
        seen[cells] = True
        return cells

    def sees(self, p, q):
        """q within range of p and in its line of sight."""
        if math.dist(p, q) > self.spec.sense:
            return False
        return line_free(self, p, q, self.occ)


class Visibility:
    """For every cell offset within range, the cells a ray from the centre
    of the origin cell crosses before it reaches that offset: a cell is
    visible when none of those is occupied."""

    def __init__(self, spec):
        r = spec.sense / spec.cell
        n = int(math.ceil(r))
        offsets, rays = [], []
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if math.hypot(dx, dy) > r:
                    continue
                offsets.append((dx, dy))
                cross = []
                steps = max(4, int(4 * math.hypot(dx, dy)))
                for s in range(1, steps):
                    f = s / steps
                    cx, cy = int(math.floor(0.5 + f * dx)), int(math.floor(0.5 + f * dy))
                    if (cx, cy) not in ((0, 0), (dx, dy)) and (cx, cy) not in cross:
                        cross.append((cx, cy))
                rays.append(cross)
        width = max(1, max(len(c) for c in rays))
        self.offsets = np.array(offsets, dtype=np.int32)
        self.ray = np.zeros((len(rays), width, 2), dtype=np.int32)
        self.mask = np.zeros((len(rays), width), dtype=bool)
        for i, cross in enumerate(rays):
            for j, (cx, cy) in enumerate(cross):
                self.ray[i, j] = (cx, cy)
                self.mask[i, j] = True

    def visible(self, ix, iy, occ):
        nx, ny = occ.shape
        tx = ix + self.offsets[:, 0]
        ty = iy + self.offsets[:, 1]
        inside = (tx >= 0) & (tx < nx) & (ty >= 0) & (ty < ny)
        rx = np.clip(ix + self.ray[:, :, 0], 0, nx - 1)
        ry = np.clip(iy + self.ray[:, :, 1], 0, ny - 1)
        hit = (occ[rx, ry] & self.mask).any(axis=1)
        ok = inside & ~hit
        return (tx[ok], ty[ok])


def line_free(area, p, q, occ):
    """No occupied cell of `occ` on the segment p-q (sampled at a quarter cell)."""
    d = math.dist(p, q)
    n = max(1, int(4 * d / area.spec.cell))
    c = area.spec.cell
    for s in range(n + 1):
        f = s / n
        x, y = p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])
        ix, iy = int(x / c), int(y / c)
        if not (0 <= ix < area.nx and 0 <= iy < area.ny) or occ[ix, iy]:
            return False
    return True


def generate(spec, seed, n_obstacles=None, keep_clear=()):
    """An area of the paper's simulations: 10 to 20 obstacles of one cell at
    random, none within 4 m of the home base or of a point in keep_clear."""
    rng = np.random.default_rng(seed)
    nx, ny = int(round(spec.width / spec.cell)), int(round(spec.height / spec.cell))
    occ = np.zeros((nx, ny), dtype=bool)
    n = int(rng.integers(10, 21)) if n_obstacles is None else n_obstacles
    placed = 0
    clear = [spec.home] + list(keep_clear)
    while placed < n:
        ix, iy = int(rng.integers(0, nx)), int(rng.integers(0, ny))
        p = ((ix + 0.5) * spec.cell, (iy + 0.5) * spec.cell)
        if occ[ix, iy] or any(math.dist(p, q) < 4.0 for q in clear):
            continue
        occ[ix, iy] = True
        placed += 1
    return occ


def place_tasks(spec, occ, seed, requirements, min_home=20.0, min_apart=8.0):
    """Tasks at random free cells, at least min_home from the base and
    min_apart from each other, one per entry of `requirements` (the number of
    robots each needs)."""
    rng = np.random.default_rng(seed)
    nx, ny = occ.shape
    tasks = []
    tries = 0
    while len(tasks) < len(requirements):
        tries += 1
        ix, iy = int(rng.integers(1, nx - 1)), int(rng.integers(1, ny - 1))
        p = ((ix + 0.5) * spec.cell, (iy + 0.5) * spec.cell)
        if occ[ix, iy] or math.dist(p, spec.home) < min_home:
            continue
        if any(math.dist(p, t['pos']) < min_apart for t in tasks) and tries < 10000:
            continue
        # keep the cells around a task free, so that k robots fit within r_task
        if occ[max(0, ix - 1):ix + 2, max(0, iy - 1):iy + 2].any():
            continue
        tasks.append({'id': f't{len(tasks) + 1}', 'pos': p, 'k': int(requirements[len(tasks)])})
    return tasks


def case_requirements(case, n_robots, seed):
    """What the tasks of a run need. Case I: one task that needs between two
    robots and the whole team. Case II: two or three tasks of one to three
    robots each, needing no more robots in all than the team has (the
    paper's sum of N_tau at most N_a). Case III: tasks that need more robots
    in all than the team has, so that the same robots may be needed by two of
    them at once, which the paper's rules are not guaranteed to handle."""
    rng = np.random.default_rng(seed)
    if case == 1:
        return [int(rng.integers(2, n_robots + 1))]
    if case == 3:
        # more robots needed in all than the team has, as in the paper's own
        # experiment of Fig. 11 (a two-robot and a three-robot task, three robots)
        return {2: [1, 2], 3: [2, 3]}.get(n_robots, [3] * (n_robots // 3 + 1))
    while True:
        n = int(rng.integers(2, 4))
        ks = [int(rng.integers(1, min(3, n_robots) + 1)) for _ in range(n)]
        if sum(ks) <= n_robots:
            return ks
