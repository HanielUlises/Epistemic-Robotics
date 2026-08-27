# The warehouse scenario

The floor plan is the one from the RoboticsAcademy [multi-robot Amazon
warehouse exercise](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/):
shelf blocks with aisles between them, a cross corridor at each end, and
loading docks on the east wall. Two robots, a pallet to move, a dock to bring
it to.

What that exercise asks for is a centralised task planner that assigns the
jobs. What it never asks is how a robot comes to *know* where the pallet is,
or what a robot should do about a part of the floor it has never seen. Both
questions are the subject here, and both are answered on the map rather than
around it.

Twenty-four metres by fourteen, at ten centimetres a cell: 240 x 140.

| | |
| --- | --- |
| ![No route: the east corridor was never observed](out/unknown-is-not-free.png) | ![The same query after it was](out/after-sensing.png) |

**Left:** the grey band is the east corridor, which nobody has looked into.
Unknown is not free, so the least fixed point excludes it and there is no
route to the dock at all. **Right:** the same query on the same warehouse
after a sensing action resolved that corridor. Nothing else changed.

## The two stages and the three snapshots

| grid | what it is |
| --- | --- |
| `stage_a` | the east corridor has never been observed: every cell of it is `-1` |
| `stage_b` | the same warehouse after a sensing action settled it |

| snapshot | what the robots know |
| --- | --- |
| `docks` | nothing in dispute: one world, both docks grounded on the map |
| `pallet` | the pallet is in the bay at aisle 2 or the one at aisle 3, and r1 cannot tell which: the zone has one extent in `w0` and another in `w1` |
| `fleet` | the same disagreement, plus r2, whose relation is the discrete partition — r2 has been down the aisles and tells the worlds apart |

## The cases

```
case                    goal   known  disputed sensing  region   iters  path   look
unknown-is-not-free     625    625    0        0        3834     98     0      0
after-sensing           625    625    0        0        25916    301    195    0
ontic-pallet            192    0      384      0        25916    254    59     0
epistemic-pallet        192    0      384      1072     25916    208    53     1
second-robot-knows      192    192    0        192      25916    254    120    0
safety-behind-the-link  625    625    0        0        25916    301    270    0
```

Four things worth reading off that table.

**`unknown-is-not-free` against `after-sensing`.** Same robot, same dock, same
625 goal cells. The winning region is 3834 cells in the first and the whole
free floor in the second, and the route goes from nothing to 195 cells. The
difference is one corridor's worth of cells having been looked at.

**`ontic-pallet` against `epistemic-pallet`.** Same map, same snapshot, same
zone; the goals differ in whether the robot has to *know* it arrived. The
ontic query drives 59 cells to the bay the designated world puts the pallet
in and settles nothing — 384 cells stay in dispute and the robot could not say
which bay it is standing in. The epistemic query stops 53 cells out, at the
one place from which the question is decidable, and sends the robot there
instead.

**`second-robot-knows`.** The same epistemic goal for r2, who can tell `w0`
from `w1`. Nothing is in dispute, 192 goal cells are already known to be goal
cells, and no sensing waypoint is produced. Knowing already is cheaper than
finding out, and the planner does not spend a look on it.

**`safety-behind-the-link`.** The safety constraint is `free ∧ link_up`, a
formula evaluated against the model rather than a mask over the map. With the
link up it is every free cell; take the link down — the test does — and the
safe set is empty, so there is no route rather than a longer one.

## Running it

```bash
colcon build --packages-select epistemic_msgs mu_path_planner warehouse_scenario
bash scenarios/warehouse/run_demo.sh
```

That writes the floor and the snapshots, answers every case offline, puts the
same cases to a running `mu_path_planner_node` over topics, and draws both.
The driver answers each case twice — once through the node and once in
process — and fails if the two disagree.

| executable | what it does |
| --- | --- |
| `make_maps` | writes `stage_a` and `stage_b` as PGM plus YAML, and the three snapshots as JSON |
| `run_offline` | resolves every case and takes the fixed point, with no ROS in the way |
| `scenario_driver` | publishes each case's map, snapshot and query, and checks the answer against the offline one |
| `render_scenario` | draws every result over the floor it was computed on |

The scenario is also asserted rather than looked at:

```bash
colcon test --packages-select warehouse_scenario
```

## What the wire changed, twice

Two defects showed up only because the same question was asked both ways, and
both are fixed in `mu_path_planner`:

**A pose landed in a different cell depending on where the map came from.**
`OccupancyGrid.resolution` is a float32, so `x = 2.0 m` over `0.1 m/cell`
arrives as 19.9999997 cells and floors to 19, while the same map built with a
double gives exactly 20. `GridInfo::cell_at` now snaps a quotient that is
within a relative millionth of an integer, which is far below half a cell and
well above the float32 error — and the error is relative, so the tolerance is
too: a fixed one placed the robot at y = 1.6 m correctly and left the one at
y = 12.4 m a cell short.

**Answers were read as answers to the wrong question.** A planner left running
from an earlier session answers these queries too, and every subscriber reads
its answers as its own. The node now stamps a path with the stamp of the query
it answers, and publishes on `/mu_planner/status` which map and which snapshot
it is currently holding — by content hash, so a driver can wait until the
planner has actually taken what it just sent instead of sleeping and hoping.

## The domain over the same warehouse

`epddl-workspace/robot-warehouse/` is this warehouse at the other altitude:
zones rather than cells, and knowledge rather than routes. The pallet is in
one of two bays, `pickup` has a modal precondition — a robot may lift the
pallet only where it *knows* the pallet is — and the goal asks both that the
pallet reach the dock and that r2 know which bay it came out of.

```bash
bash epddl-workspace/robot-warehouse/validate.sh
```

The solution branches at depth 7 with two leaves, and the division of labour
is not written anywhere in the domain: r1 walks to bay 2 while r2 walks to
bay 3, and whichever of them finds the pallet carries it to the dock. Note
what does *not* appear in the plan — the announcement. `pickup` is public, so
the fleet's seeing it happen is already enough for r2 to learn which bay the
pallet was in, and the planner works that out rather than spending an action
on saying so.

Two plans are rejected in the same run, and both rejections are the point: a
sequence that inspects and then picks up regardless is not applicable, because
after the inspection the model still designates two worlds; and a plan that
picks up without looking fails on the modal precondition. Drop that conjunct
and the domain is the classical one, in which a robot reaches for a pallet it
has no reason to believe is there.
