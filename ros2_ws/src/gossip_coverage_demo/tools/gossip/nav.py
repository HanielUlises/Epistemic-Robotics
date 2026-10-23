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
Grid search, path smoothing and the potential field of Section 5.1.

A path is planned by A* over the cells a map does not hold occupied (unknown
cells count as free, as they do for a particle), eight-connected without
cutting a corner, and smoothed by keeping only the points between which the
line is clear. A robot or a particle follows its path by the composite field
of eqs. (6) to (8): attraction to the next point, repulsion from the occupied
cells it knows within rho_o.
"""

import heapq
import math

import numpy as np

SQ2 = math.sqrt(2.0)
MOVES = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2))


def astar(blocked, start, goal):
    """Cells from start to goal over the cells not blocked, or None."""
    nx, ny = blocked.shape
    if blocked[goal]:
        goal = nearest_free(blocked, goal)
        if goal is None:
            return None
    if start == goal:
        return [start]

    def h(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (SQ2 - 2) * min(dx, dy)
    g = {start: 0.0}
    came = {}
    heap = [(h(start), 0.0, start)]
    while heap:
        _, gc, c = heapq.heappop(heap)
        if c == goal:
            path = [c]
            while c in came:
                c = came[c]
                path.append(c)
            return path[::-1]
        if gc > g.get(c, math.inf):
            continue
        for dx, dy, w in MOVES:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or blocked[n]:
                continue
            if dx and dy and (blocked[c[0] + dx, c[1]] or blocked[c[0], c[1] + dy]):
                continue
            ng = gc + w
            if ng < g.get(n, math.inf):
                g[n] = ng
                came[n] = c
                heapq.heappush(heap, (ng + h(n), ng, n))
    return None


def nearest_free(blocked, c):
    nx, ny = blocked.shape
    for r in range(1, max(nx, ny)):
        best = None
        for dx in range(-r, r + 1):
            for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                n = (c[0] + dx, c[1] + dy)
                if 0 <= n[0] < nx and 0 <= n[1] < ny and not blocked[n]:
                    d = dx * dx + dy * dy
                    if best is None or d < best[0]:
                        best = (d, n)
        if best:
            return best[1]
    return None


def distance_field(blocked, source):
    """Path length in cells from source to every cell (Dijkstra, eight
    neighbours, no corner cutting); inf where unreachable."""
    nx, ny = blocked.shape
    dist = np.full((nx, ny), np.inf)
    if blocked[source]:
        source = nearest_free(blocked, source)
    dist[source] = 0.0
    heap = [(0.0, source)]
    while heap:
        d, c = heapq.heappop(heap)
        if d > dist[c]:
            continue
        for dx, dy, w in MOVES:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or blocked[n]:
                continue
            if dx and dy and (blocked[c[0] + dx, c[1]] or blocked[c[0], c[1] + dy]):
                continue
            nd = d + w
            if nd < dist[n]:
                dist[n] = nd
                heapq.heappush(heap, (nd, n))
    return dist


def clear(blocked, cell, p, q):
    """The segment p-q crosses no blocked cell."""
    d = math.dist(p, q)
    n = max(1, int(4 * d / cell))
    nx, ny = blocked.shape
    for s in range(n + 1):
        f = s / n
        ix, iy = int((p[0] + f * (q[0] - p[0])) / cell), int((p[1] + f * (q[1] - p[1])) / cell)
        if not (0 <= ix < nx and 0 <= iy < ny) or blocked[ix, iy]:
            return False
    return True


def smooth(blocked, cell, points):
    """Keep only the points between which the line is clear."""
    if len(points) <= 2:
        return list(points)
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not clear(blocked, cell, points[i], points[j]):
            j -= 1
        out.append(points[j])
        i = j
    return out


def plan(blocked, cell, p, goal):
    """A smoothed path of points from p to goal over the cells not blocked,
    or None. The first point is p and the last the goal."""
    nx, ny = blocked.shape
    s = (min(nx - 1, max(0, int(p[0] / cell))), min(ny - 1, max(0, int(p[1] / cell))))
    t = (min(nx - 1, max(0, int(goal[0] / cell))), min(ny - 1, max(0, int(goal[1] / cell))))
    if blocked[s]:
        s = nearest_free(blocked, s)
    cells = astar(blocked, s, t)
    if cells is None:
        return None
    pts = [tuple(p)] + [((c[0] + 0.5) * cell, (c[1] + 0.5) * cell) for c in cells[1:-1]]
    end = goal if not blocked[t] else ((cells[-1][0] + 0.5) * cell, (cells[-1][1] + 0.5) * cell)
    pts.append(tuple(end))
    return smooth(blocked, cell, pts)


def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


class Mover:
    """Takes a point to a moving or fixed goal over a map that it may learn
    as it goes: plans by A* when the straight line is not clear, follows the
    plan by the potential field, and plans again when the goal moves more
    than a metre or the map blocks the plan."""

    def __init__(self, spec):
        self.spec = spec
        self.path = None
        self.goal = None
        self.version = -1

    def reset(self):
        self.path, self.goal = None, None

    def waypoint(self, p, goal, blocked, version):
        cell = self.spec.cell
        if clear(blocked, cell, p, goal):
            self.path = None
            return goal
        replan = (self.path is None or self.goal is None or math.dist(goal, self.goal) > 1.0
                  or version != self.version)
        if not replan:
            # drop the points reached, and replan if the next is no longer clear
            while len(self.path) > 1 and math.dist(p, self.path[0]) < 0.4 * cell:
                self.path.pop(0)
            if not clear(blocked, cell, p, self.path[0]):
                replan = True
        if replan:
            path = plan(blocked, cell, p, goal)
            self.goal, self.version = tuple(goal), version
            self.path = path[1:] if path and len(path) > 1 else None
            if self.path is None:
                return goal
        # the furthest point of the path still in line of sight
        k = 0
        while k + 1 < len(self.path) and clear(blocked, cell, p, self.path[k + 1]):
            k += 1
        self.path = self.path[k:]
        return self.path[0]

    def step(self, p, goal, blocked, version, speed, occupied_near=None):
        """The velocity towards goal at most `speed`: attraction to the next
        waypoint, eq. (6), and repulsion from known occupied cells within
        rho_o, eq. (7)."""
        s = self.spec
        w = self.waypoint(p, goal, blocked, version)
        ax, ay = s.eta_g * (w[0] - p[0]), s.eta_g * (w[1] - p[1])
        rx = ry = 0.0
        if occupied_near is not None and len(occupied_near):
            d = np.hypot(occupied_near[:, 0] - p[0], occupied_near[:, 1] - p[1])
            near = (d < s.rho_o) & (d > 1e-6)
            for (ox, oy), r in zip(occupied_near[near], d[near]):
                mag = s.eta_o * (1.0 / r - 1.0 / s.rho_o) / (r * r)
                rx += mag * (p[0] - ox) / r
                ry += mag * (p[1] - oy) / r
        fx, fy = ax + rx, ay + ry
        n = math.hypot(fx, fy)
        if n < 1e-9:
            return 0.0, 0.0
        dist = math.dist(p, goal)
        v = min(speed, dist / s.dt) if dist < speed * s.dt * 1.5 else speed
        return v * fx / n, v * fy / n


def dilate(mask, cells=1):
    """The mask grown by `cells` in every direction, eight-connected."""
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        grown[1:, 1:] |= out[:-1, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        out = grown
    return out


def disk_sum(mask, radius):
    """For every cell, how many cells of `mask` lie within `radius` cells of
    it (zero outside the grid)."""
    nx, ny = mask.shape
    m = mask.astype(float)
    out = np.zeros_like(m)
    r = int(radius)
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            xs, xd = (slice(dx, nx), slice(0, nx - dx)) if dx >= 0 else (slice(0, nx + dx), slice(-dx, nx))
            ys, yd = (slice(dy, ny), slice(0, ny - dy)) if dy >= 0 else (slice(0, ny + dy), slice(-dy, ny))
            out[xd, yd] += m[xs, ys]
    return out


def occupied_points(known, cell, p, radius):
    """Centres of the occupied cells of `known` within radius of p."""
    nx, ny = known.shape
    r = int(math.ceil(radius / cell)) + 1
    ix, iy = int(p[0] / cell), int(p[1] / cell)
    x0, x1, y0, y1 = max(0, ix - r), min(nx, ix + r + 1), max(0, iy - r), min(ny, iy + r + 1)
    sub = np.argwhere(known[x0:x1, y0:y1] == 1)
    if not len(sub):
        return np.zeros((0, 2))
    return (sub + np.array([x0, y0]) + 0.5) * cell
