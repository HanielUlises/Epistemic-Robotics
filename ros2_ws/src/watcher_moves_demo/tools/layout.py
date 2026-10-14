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
The hall of the watcher that moves, in one place.

The hall of watcher_demo, with the posts nearer the block and a passage behind
the north wall joining them, by which the watcher changes posts out of
everyone's sight. What follows is watcher_demo's description, still true but
for that.


From west to east: bay X, the west hall, a solid block holding r2's booth, the
east hall, bay Y. Under all of it, behind a wall, runs the south corridor,
where r1, the carrier, starts before two boards: the order board, whose lit
slot names the bay, and the duty board, whose lit slot names the post the
watcher r3 has today, west or east.

Each bay opens onto its hall by a gate, and onto the corridor by a door. The
booth has a window onto each hall. Each hall has a watcher's post near its
north wall and an entry from the corridor, behind which a baffle stands in
the corridor so that nothing of the corridor can be seen from the post.

So r2, in the booth, sees through its windows both gates and both window
spots; a watcher at the west post sees the west gate and the west window spot,
and nothing of the east hall, the corridor or the bays' insides; the east post
the same, mirrored. tools/check_views.py casts the rays and checks it.

Coordinates are metres in the world frame, which is also every robot's odometry
frame.
"""

import math

HALL = (-7.0, -4.6, 7.0, 3.0)
WALL = 0.1
CORRIDOR_N = -2.6                       # the corridor's north wall
BAY_INNER = 4.2                         # |x| of each bay's wall onto its hall
GATE = (1.2, 2.0)                       # y extent of the gate in that wall
BAY_DOOR = (4.8, 5.6)                   # |x| extent of the bay's door onto the corridor
ENTRY = (2.0, 2.8)                      # |x| extent of the hall's entry from the corridor
BAFFLE = (1.6, 3.2, -3.45)              # |x| extent and y of the baffle under the entry
BLOCK = 1.2                             # |x| of the block's faces onto the halls
BOOTH = (-0.7, 0.7)                     # y extent of the booth inside the block
WINDOW = (-0.2, 0.6)                    # y extent of each booth window
START = (0.0, -3.6, math.radians(-90))  # r1, facing the boards
BOARD_Y = -4.5                          # face of both boards
ORDER_SLOT_X = (-0.45, 0.45)            # order board: slot X, slot Y
DUTY_SLOT_X = (-1.35, 1.35)             # duty board: west post, east post
POST_Y = BOARD_Y + 0.25                 # where a lit slot's post stands
STATION = (0.0, 0.0, math.radians(90))  # r2, in the booth
WATCH_POST = (1.9, 2.4)                 # |x|, y of the watcher's post
PASSAGE_DOOR = (1.6, 2.2)               # |x| extent of each hall's door onto the passage
PASSAGE = (3.1, 3.9, 2.5)               # y extent of the passage behind the north wall, and |x| of its ends
WATCH_YAW = math.radians(-90)


def side_sign(s):
    return -1.0 if s == 'w' else 1.0


def bay_side(b):
    return 'w' if b == 'x' else 'e'


def post(s):
    return (side_sign(s) * WATCH_POST[0], WATCH_POST[1], WATCH_YAW)


def gate(b):
    """Where r1 stands in the gate of bay b."""
    return (side_sign(bay_side(b)) * BAY_INNER, sum(GATE) / 2)


def window(s):
    """Where r1 stands before the booth window on side s."""
    return (side_sign(s) * (BLOCK + 0.4), sum(WINDOW) / 2 - 0.1)


def box_around(p, h):
    return (p[0] - h, p[1] - h, p[0] + h, p[1] + h)


def gate_region(b):
    return box_around(gate(b), 0.3)


def window_region(s):
    return box_around(window(s), 0.3)


def bay_region(b):
    x0, x1 = -HALL[2], -BAY_INNER
    if b == 'y':
        x0, x1 = BAY_INNER, HALL[2]
    return (x0, CORRIDOR_N, x1, HALL[3])


def hall_region(s):
    x0, x1 = -BAY_INNER, -BLOCK
    if s == 'e':
        x0, x1 = BLOCK, BAY_INNER
    return (x0, CORRIDOR_N, x1, HALL[3])


def slot_region(x):
    return (x - 0.13, POST_Y - 0.13, x + 0.13, POST_Y + 0.13)


def order_slot_region(b):
    return slot_region(ORDER_SLOT_X[0] if b == 'x' else ORDER_SLOT_X[1])


def duty_slot_region(s):
    return slot_region(DUTY_SLOT_X[0] if s == 'w' else DUTY_SLOT_X[1])


def walls():
    """(cx, cy, sx, sy) boxes."""
    x0, y0, x1, y1 = HALL
    t = WALL
    out = [((x0 + x1) / 2, y0 - t / 2, x1 - x0 + 2 * t, t),
           (x1 + t / 2, (y0 + y1) / 2, t, y1 - y0), (x0 - t / 2, (y0 + y1) / 2, t, y1 - y0)]

    def box(xa, ya, xb, yb):
        out.append(((xa + xb) / 2, (ya + yb) / 2, abs(xb - xa), abs(yb - ya)))
    # The north wall, cut by the two doors onto the passage.
    d0, d1 = PASSAGE_DOOR
    for a_, b_ in ((x0 - t, -d1), (-d0, d0), (d1, x1 + t)):
        box(a_, y1, b_, y1 + t)
    # The passage: a closed room behind the north wall, joining the doors.
    py0, py1, px = PASSAGE
    box(-px - t, py1, px + t, py1 + t)
    for sgn in (-1, 1):
        box(sgn * px, y1 + t, sgn * (px + t), py1)
    # The corridor's north wall, from the west wall to the east, cut by each
    # bay's door and each hall's entry.
    cuts = sorted([(-BAY_DOOR[1], -BAY_DOOR[0]), (-ENTRY[1], -ENTRY[0]), ENTRY, BAY_DOOR])
    xs = [x0] + [c for cut in cuts for c in cut] + [x1]
    for a, b in zip(xs[::2], xs[1::2]):
        box(a, CORRIDOR_N, b, CORRIDOR_N + t)
    for s in (-1, 1):
        # each bay's wall onto its hall, with the gate
        box(s * BAY_INNER, CORRIDOR_N + t, s * (BAY_INNER + t), GATE[0])
        box(s * BAY_INNER, GATE[1], s * (BAY_INNER + t), y1)
        # the baffle under the entry
        box(s * BAFFLE[0], BAFFLE[2] - t / 2, s * BAFFLE[1], BAFFLE[2] + t / 2)
    # The block: solid but for the booth, and the booth's two windows.
    bx = BLOCK
    box(-bx, CORRIDOR_N + t, bx, BOOTH[0])
    box(-bx, BOOTH[1], bx, y1)
    for s in (-1, 1):
        box(s * bx, BOOTH[0], s * (bx - t), WINDOW[0])
        box(s * bx, WINDOW[1], s * (bx - t), BOOTH[1])
    return out


def roadmap():
    """Named places, and straight segments clear of every wall."""
    n = {'start': START[:2], 'station': STATION[:2]}
    for s in ('w', 'e'):
        k = side_sign(s)
        b = 'x' if s == 'w' else 'y'
        mid = (BAFFLE[2] + CORRIDOR_N) / 2
        ent = sum(ENTRY) / 2
        n.update({
            f'c_in_{s}': (k * (BAFFLE[0] - 0.35), -3.95),       # corridor, east (inner) end of the baffle
            f'pk_in_{s}': (k * (BAFFLE[0] - 0.35), mid),        # the pocket between baffle and wall
            f'pk_{s}': (k * ent, mid),                          # under the entry
            f'en_{s}': (k * ent, CORRIDOR_N + 0.45),            # just inside the hall
            f'win_{s}': window(s),
            f'ga_{s}': (k * (BAY_INNER - 0.8), sum(GATE) / 2),  # before the gate
            f'gate_{s}': gate(b),
            f'in_g_{s}': (k * (BAY_INNER + 0.9), sum(GATE) / 2),  # in the bay, past the gate
            f'c_d_{s}': (k * sum(BAY_DOOR) / 2, -3.95),         # corridor, under the bay's door
            f'bd_{s}': (k * sum(BAY_DOOR) / 2, CORRIDOR_N + 0.5),  # in the bay, past its door
            f'drop_{s}': (k * (sum(BAY_DOOR) / 2 + 0.4), -1.5),  # where the load is left
        })
    e = []
    for s in ('w', 'e'):
        e += [('start', f'c_in_{s}'), (f'c_in_{s}', f'pk_in_{s}'), (f'pk_in_{s}', f'pk_{s}'), (f'pk_{s}', f'en_{s}'),
              (f'en_{s}', f'win_{s}'), (f'en_{s}', f'ga_{s}'), (f'ga_{s}', f'gate_{s}'), (f'gate_{s}', f'in_g_{s}'),
              (f'c_in_{s}', f'c_d_{s}'), (f'c_d_{s}', f'bd_{s}'), (f'bd_{s}', f'drop_{s}'),
              (f'in_g_{s}', f'drop_{s}')]
    e.append(('c_in_w', 'c_in_e'))
    # The watcher's way between its posts.
    for s in ('w', 'e'):
        k = side_sign(s)
        n[f'post_{s}'] = (k * WATCH_POST[0], WATCH_POST[1])
        n[f'door_{s}'] = (k * sum(PASSAGE_DOOR) / 2, HALL[3] + 0.05)
        n[f'pass_{s}'] = (k * sum(PASSAGE_DOOR) / 2, sum(PASSAGE[:2]) / 2)
        e += [(f'post_{s}', f'door_{s}'), (f'door_{s}', f'pass_{s}')]
    e.append(('pass_w', 'pass_e'))
    return n, e


def route_watcher(frm, to):
    """The watcher's way from post frm to post to, through the passage."""
    return [f'post_{frm}', f'door_{frm}', f'pass_{frm}', f'pass_{to}', f'door_{to}', f'post_{to}']


# Routes, as places to drive through after the robot's current one.
def route_to_gate(b):
    s = bay_side(b)
    return ['start', f'c_in_{s}', f'pk_in_{s}', f'pk_{s}', f'en_{s}', f'ga_{s}', f'gate_{s}']


def route_gate_in(b):
    s = bay_side(b)
    return [f'gate_{s}', f'in_g_{s}']


def route_to_window(s):
    return ['start', f'c_in_{s}', f'pk_in_{s}', f'pk_{s}', f'en_{s}', f'win_{s}']


def route_back(s, b):
    """From the window on side s, unseen, to the bay b's drop."""
    d = bay_side(b)
    path = [f'win_{s}', f'en_{s}', f'pk_{s}', f'pk_in_{s}', f'c_in_{s}']
    if d != s:
        path.append(f'c_in_{d}')
    return path + [f'c_d_{d}', f'bd_{d}', f'drop_{d}']


def length(path):
    nodes, _ = roadmap()
    return sum(math.dist(nodes[u], nodes[v]) for u, v in zip(path, path[1:]))


CARRIER_PLACES = ('start', 'win_w', 'win_e', 'ga_w', 'ga_e')


def shortest(a, b, banned=('post_w', 'post_e', 'door_w', 'door_e', 'pass_w', 'pass_e')):
    """The shortest way over the roadmap from a to b, avoiding the watcher's own."""
    nodes, edges = roadmap()
    adj = {k: [] for k in nodes}
    for u, v in edges:
        if u in banned or v in banned:
            continue
        d = math.dist(nodes[u], nodes[v])
        adj[u].append((v, d))
        adj[v].append((u, d))
    best, prev, todo = {a: 0.0}, {}, {a}
    while todo:
        u = min(todo, key=best.get)
        todo.remove(u)
        if u == b:
            break
        for v, d in adj[u]:
            if best[u] + d < best.get(v, math.inf):
                best[v], prev[v] = best[u] + d, u
                todo.add(v)
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1]


def route_between(a, b):
    """The carrier's drive from place a to place b, both among CARRIER_PLACES."""
    return shortest(a, b)
