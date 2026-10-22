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
The area of the belief and empathy demonstration, in one place.

An open area 30 by 16 metres with crates and columns standing in it. Four
east-west streets cross it, at y = -6, -2, 2 and 6, and four north-south
avenues join them, at x = -14, -8, 8 and 14; every obstacle stands in a block
between them, at least 0.75 m from any centreline, so that a ground vehicle on
a street or an avenue meets none. The base is at the south edge, in the middle.

Three robots leave the base together: an aerial vehicle u and two ground
vehicles, g1 and g2. While they are still in contact they split the area, in
proportion to speed: the west strip (x < -7) to g1, the east strip (x > 7) to
g2, and the middle, twice as wide, to u, which flies three times as fast. This
is the partition of Bramblett and Bezzo's equation (3) with the weights set by
speed, computed once at the start and not again. Each robot then covers its
strip along a route every robot knows: the ground vehicles along the streets of
their strip, northwards lane by lane, and the aerial vehicle along the streets
of the middle, at 1.8 m, over the obstacles.

Out of contact, every robot keeps three belief particles for each ground
vehicle: where the vehicle is along its route if it has had no fault, one or
two. Particle b moves along the route at the speed SPEED[b - 1], 100, 60 and
20 per cent of the nominal speed as in the paper's simulations, from the
common start. A ground vehicle that has a fault does what the others expect of
it: it tracks its next particle (the paper's empathy). Its top speed after k
faults, CAP[k], is a little above its particle's, so that it can reach the
particle and hold it.

The task the aerial vehicle may find stands at one of four sites on the
streets of the middle strip; the seed decides which. It needs the aerial
vehicle and a ground vehicle with fewer than two faults within R_SITE of it
together for DELTA seconds.

Coordinates are metres in the world frame, which is every robot's odometry
frame. tools/check_layout.py checks what the domain assumes of this geometry.
"""

import heapq
import math

AREA = (-15.0, -8.0, 15.0, 8.0)
STREETS = (-6.0, -2.0, 2.0, 6.0)
AVENUES = (-14.0, -8.0, 8.0, 14.0)
BASE = (0.0, -7.3)
ROBOTS = ('u', 'g1', 'g2')
UGVS = ('g1', 'g2')
START = {'u': (0.0, -7.3, math.radians(90)),
         'g1': (-0.7, -7.3, math.radians(90)),
         'g2': (0.7, -7.3, math.radians(90))}
SIDE = {'g1': 'west', 'g2': 'east'}
STRIP = {'g1': (-15.0, -7.0), 'u': (-7.0, 7.0), 'g2': (7.0, 15.0)}
ALTITUDE = 1.8

SPEED = (0.16, 0.096, 0.032)      # m/s, particles 1, 2, 3
CAP = (0.20, 0.12, 0.05)          # m/s, a ground vehicle's top speed after 0, 1, 2 faults
DRIVE = (0.18, 0.11, 0.045)       # m/s, the speed it drives a planned path at
V_UAV = 0.5                       # m/s, the aerial vehicle, about three times a ground vehicle

R = 2.5                           # m, radio range, in the plane
R_DETECT = 2.2                    # m, a robot sees the task from this far: the half-swath of the
                                  # coverage, whose lanes are 4 m apart
R_SITE = 1.0                      # m, within this of the site, a robot is at the task
DELTA = 8.0                       # s, both at the task this long and it is done
HAIL_WINDOW = 8.0                 # s, how long the aerial vehicle follows a particle, calling
SITES = ((-4.0, 2.0), (4.0, 2.0), (4.0, 6.0), (-4.0, 6.0))

# (cx, cy, sx, sy, sz): crates, pallets and columns, each in a block.
OBSTACLES = [
    (-11.0, -4.0, 1.6, 1.2, 0.8), (-12.2, 0.0, 0.8, 0.8, 1.0), (-10.0, 4.2, 2.0, 0.8, 0.6),
    (11.0, -4.2, 1.2, 1.6, 0.8), (10.0, 0.3, 1.8, 0.9, 0.6), (12.0, 4.0, 0.8, 0.8, 1.0),
    (-5.0, -4.0, 1.5, 1.5, 1.0), (3.0, -4.5, 2.4, 0.8, 0.7), (-2.0, 0.0, 1.2, 2.0, 1.2),
    (5.0, -0.2, 0.9, 0.9, 1.5), (-5.5, 4.0, 1.0, 1.4, 0.8), (2.2, 4.2, 2.2, 1.0, 0.6),
    (-6.0, 7.3, 2.0, 0.6, 0.6), (6.5, -7.3, 2.0, 0.6, 0.6),
]

# The coverage routes, as polylines from each robot's start.
ROUTE = {
    'g1': [START['g1'][:2], (-1.0, -6.0), (-8.0, -6.0), (-14.0, -6.0), (-14.0, -2.0), (-8.0, -2.0),
           (-8.0, 2.0), (-14.0, 2.0), (-14.0, 6.0), (-8.0, 6.0)],
    'g2': [START['g2'][:2], (1.0, -6.0), (8.0, -6.0), (14.0, -6.0), (14.0, -2.0), (8.0, -2.0),
           (8.0, 2.0), (14.0, 2.0), (14.0, 6.0), (8.0, 6.0)],
    'u': [START['u'][:2], (-6.0, -6.0), (6.0, -6.0), (6.0, -2.0), (-6.0, -2.0), (-6.0, 2.0), (6.0, 2.0),
          (6.0, 6.0), (-6.0, 6.0)],
}

# The flock: the team sweeps the whole area in contact, in one loop that never
# doubles back: east along the south band, north, west along the north band,
# south, and east again to the middle. The aerial vehicle flies between the two
# ground vehicles, each on a street 2 m from it. The paths have the same number
# of segments, and a robot holds the point of its own path at the same segment
# and fraction as the flock's reference, the aerial vehicle's path.
FLOCK = {
    'u': [START['u'][:2], (0.0, -6.0), (0.0, -4.0), (14.0, -4.0), (14.0, 4.0), (-14.0, 4.0), (-14.0, -4.0),
          (0.0, -4.0)],
    'g1': [START['g1'][:2], (-1.2, -6.0), (1.5, -6.0), (14.0, -6.0), (14.0, 2.0), (-14.0, 2.0), (-14.0, -6.0),
           (-1.5, -6.0)],
    'g2': [START['g2'][:2], (0.0, -6.0), (0.0, -2.0), (14.0, -2.0), (14.0, 6.0), (-14.0, 6.0), (-14.0, -2.0),
           (-1.5, -2.0)],
}


# ─────────────────────────────────────────────────────────── polylines ──

def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def at(path, s):
    """The point at arc length s along a polyline, clamped to its ends, and
    the heading of the segment it lies on."""
    s = max(0.0, s)
    last = len(path) - 2
    for k, (a, b) in enumerate(zip(path, path[1:])):
        d = math.dist(a, b)
        if s <= d or k == last:
            f = min(1.0, s / d) if d > 0 else 0.0
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f), math.atan2(b[1] - a[1], b[0] - a[0])
        s -= d
    return path[-1], 0.0


def project(path, p):
    """The arc length of the point of a polyline nearest p, and its distance."""
    best, best_d, run = 0.0, float('inf'), 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        d2 = dx * dx + dy * dy
        f = 0.0 if d2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d2))
        q = (a[0] + dx * f, a[1] + dy * f)
        d = math.dist(q, p)
        if d < best_d:
            best, best_d = run + f * math.sqrt(d2), d
        run += math.sqrt(d2)
    return best, best_d


# ─────────────────────────────────────────────────────────── particles ──

def particle(g, b, t):
    """Where particle b of ground vehicle g stands t seconds after the start."""
    return at(ROUTE[g], SPEED[b - 1] * max(0.0, t))[0]


def intercept(p, t, g, b, speed=V_UAV, horizon=600.0, step=0.2):
    """The first point, and the time, at which a vehicle at p at time t, flying
    straight at `speed`, can be where particle b of g is then."""
    k = 0.0
    while k <= horizon:
        q = particle(g, b, t + k)
        if math.dist(p, q) <= speed * k:
            return q, t + k
        k += step
    return particle(g, b, t + horizon), t + horizon


# ─────────────────────────────────────────────────────── the ground roadmap ──

def roadmap():
    """Nodes and edges a ground vehicle drives on: every crossing of a street
    and an avenue, the sites, and the way from the base to the first street."""
    nodes = {}

    def node(p):
        key = (round(p[0], 3), round(p[1], 3))
        nodes[key] = key
        return key
    for y in STREETS:
        for x in AVENUES:
            node((x, y))
    for s in SITES:
        node(s)
    for p in ((-1.0, -6.0), (1.0, -6.0), (0.0, -6.0), BASE, START['g1'][:2], START['g2'][:2]):
        node(p)
    edges = set()
    for y in STREETS:
        xs = sorted(k[0] for k in nodes if k[1] == y)
        for a, b in zip(xs, xs[1:]):
            edges.add(((a, y), (b, y)))
    for x in AVENUES:
        ys = sorted(k[1] for k in nodes if k[0] == x and k[1] in STREETS)
        for a, b in zip(ys, ys[1:]):
            edges.add(((x, a), (x, b)))
    for p in (START['g1'][:2], START['g2'][:2]):
        edges.add((p, BASE))
    edges.add((BASE, (0.0, -6.0)))
    node((0.0, -2.0))
    edges.add(((0.0, -6.0), (0.0, -2.0)))
    return list(nodes), sorted(edges)


def ground_path(p, goal):
    """A shortest path on the roadmap from a point on or near it to a node."""
    nodes, edges = roadmap()
    goal = (round(goal[0], 3), round(goal[1], 3))
    # Join p to the nearest point of the nearest edge, and to both its ends.
    best, best_d = None, float('inf')
    for a, b in edges:
        s, d = project([a, b], p)
        if d < best_d:
            best, best_d = (a, b, at([a, b], s)[0]), d
    a, b, q = best
    graph = {}

    def link(x, y):
        w = math.dist(x, y)
        graph.setdefault(x, []).append((w, y))
        graph.setdefault(y, []).append((w, x))
    for x, y in edges:
        if {x, y} != {a, b}:
            link(x, y)
    q = (round(q[0], 4), round(q[1], 4))
    link(a, q)
    link(q, b)
    dist, prev, heap = {q: 0.0}, {}, [(0.0, q)]
    while heap:
        d, x = heapq.heappop(heap)
        if x == goal:
            break
        if d > dist.get(x, float('inf')):
            continue
        for w, y in graph.get(x, []):
            if d + w < dist.get(y, float('inf')):
                dist[y], prev[y] = d + w, x
                heapq.heappush(heap, (d + w, y))
    out, x = [goal], goal
    while x != q:
        x = prev[x]
        out.append(x)
    out.reverse()
    if math.dist(p, q) > 0.05:
        out.insert(0, (round(p[0], 4), round(p[1], 4)))
    return out


def flock_arc(r, s):
    """The arc length along robot r's own flock path of its place in the flock
    when the reference has gone s metres along the aerial vehicle's path: the
    same segment, at the same fraction."""
    ref, own = FLOCK['u'], FLOCK[r]
    run = 0.0
    for k, (a, b) in enumerate(zip(ref, ref[1:])):
        d, mine = math.dist(a, b), math.dist(own[k], own[k + 1])
        if s <= d or k == len(ref) - 2:
            return run + mine * max(0.0, min(1.0, s / d))
        s -= d
        run += mine
    return run


def flock_point(r, s):
    return at(FLOCK[r], flock_arc(r, s))[0]


def side(p):
    return 'west' if p[0] < 0 else 'east'
