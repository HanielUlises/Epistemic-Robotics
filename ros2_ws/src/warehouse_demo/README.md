# warehouse_demo

The warehouse, executed by ePlanSys.

The floor is AWS RoboMaker's small warehouse, the world the RoboticsAcademy
multi-robot Amazon warehouse exercise runs on, launched unmodified with a
TurtleBot3 spawned at shipping. The mission is to bring the pallet to receiving
*and* to know which aisle it came out of. Which aisle that is, nobody knows at
the start — so the plan is not a sequence but a policy, and which branch of it
runs is decided by what the robot sees.

```bash
ros2 launch warehouse_demo warehouse_demo_launch.py            # gazebo + rviz
ros2 launch warehouse_demo warehouse_demo_launch.py gui:=false # topics only
```

## What runs the mission

Nothing in this package decides anything about the mission. The chain is:

| | |
| --- | --- |
| `epddl-workspace/robot-warehouse` | the domain and the instance: two worlds, `pickup` only where the robot *knows* the pallet is, goal `delivered ∧ Kw r1 (pallet-at bay2)` |
| **plank** | grounds that into the task: 66 ground actions, 2 worlds, both designated |
| **the epistemic planner**, as ePlanSys's plan solver plugin | searches it into a policy: 17 nodes, one of them branching on what `inspect_r1_bay2` observes |
| **`EpistemicBTBuilder`** | renders the policy as a behaviour tree — a knowledge guard, the PlanSys2 action subtree, the DEL update, and a switch on the outcome |
| **the PlanSys2 executor** | dispatches each node on the *classical* action named in `pddl/warehouse-mapping.json` |
| the action nodes here | drive, look, handle — the things a planning stack cannot supply |
| **`slam_toolbox`** | builds the map the robot has actually seen |
| **`plansys2_epistemic_perception`** | classifies the two bays over that map and reports the event that fired to the epistemic state |
| **`mu_path_planner`** | the least fixed point over the same map: how to cross a floor that is still half unobserved |

The one thing worth watching is the order. `look_into` does not decide what was
seen: it faces the bay, holds the robot there, and ends when the *model* says
`(Kw r1 pallet-at_bay2)`. What put that knowledge in the model was perception
reading the occupancy grid. If this package decided it, the demonstration would
be a puppet show.

## The two vocabularies

`pddl/warehouse.pddl` is the classical half: where the robot is, what it holds.
`epddl/` is the epistemic half: what it knows. Neither can say what the other
says — no predicate can hold "r1 knows whether" — and
`pddl/warehouse-mapping.json` is what joins them: `inspect_r1_bay2` on the
epistemic side is `(look_into r1 bay2)` here.

## What the drive action does, and does not

`goto_zone` asks `mu_path_planner` for a route and follows it. It does not plan
one. That division is the point of having both planners: ePlanSys decides which
zone to be in and what has to be known there; the µ-calculus decides how to
cross a floor that SLAM has only half filled in — and refuses, while the way
there is unobserved, to cross it at all. Early in a run the answer is no route,
because the robot has not yet measured the floor it would drive over; the same
question put again a few metres later is answered. The log shows exactly that:

```
to lane: at (0.02, -0.01), asked 'lane', 0 legs, last answer was no route
to lane: at (0.31, 0.06), asked 'lane', 51 legs
...
unloading at the dock at dock_north
mission complete: the policy reached its goal
```

## Going to look, when there is no route

The fixed point will not route across cells nobody has measured. Early in a run
almost nothing has been measured, so the honest answer to "how do I get to the
service lane" is that there is no known way — and standing still is not a
reply to that.

So when the destination comes back unreachable, the drive picks a **frontier**:
a cell that is known free, has room for the robot and the planner's inflation
around it, and has unmeasured floor within sensor range. It asks the planner
for a route to that instead, drives there, and asks for the destination again.
The map grows, and at some point the route it wanted exists.

Three details are load-bearing, and each of them was a robot going nowhere:

- **The target must border the unknown.** Score by closeness to the goal alone
  and the robot walks to whichever corner of the measured region points at the
  destination and stops: the map stops growing and the goal stays unreachable.
- **The score rewards travelling.** Subtracting how far the frontier is over
  known floor is what gets the robot out of a dead end — when the direct way is
  shut, going somewhere else and opening it up scores better than creeping half
  a metre at the blockage and choosing the same corner again.
- **A chosen frontier is committed to until it is reached.** A route to the
  frontier succeeds, which clears the no-route flag, which makes the next cycle
  ask for the destination, fail, and choose a *different* frontier. Left alone
  the robot alternates between two of them and travels nowhere.

None of this decides anything epistemic. It is how the robot earns the map that
the µ-calculus planner, and then perception, are entitled to reason about.

The zone geometry the drive publishes on `/epistemic/state` is geometry only:
one world, six boxes plus wherever it is currently going to look, and the live
pose. What is *not* known lives in ePlanSys's
epistemic state, where the policy and perception both reach it.

## Timing, and one parameter that matters

A bay here is visible from the service lane thirty to sixty seconds before the
robot reaches it, and the epistemic state will not accept an observation from a
robot it does not yet believe is in the bay. Perception keeps offering the
reading until the executor has applied the drive — `applicability_retries`, set
to 240 in `params/warehouse_demo.yaml`. The default of 40 grids is a few
seconds, which fits a building of small rooms and not a warehouse aisle.
