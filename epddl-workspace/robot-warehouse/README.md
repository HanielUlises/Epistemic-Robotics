# robot-warehouse

The RoboticsAcademy [multi-robot Amazon warehouse
exercise](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/)
as an epistemic planning domain, on the floor of the world that exercise runs
on.

The exercise gives two robots a floor of racks, a pallet to pick up and a dock
to bring it to, and asks for a task planner that assigns the jobs. What it
never says is how a robot comes to know *where* the pallet is: a centralised
planner is simply told, and the robots drive to a coordinate. Here that is the
whole problem.

The same warehouse is laid out cell by cell in
[`scenarios/warehouse/`](../../scenarios/warehouse), where the µ-calculus
planner routes between the zones this domain names. This says which zone to be
in and what must be known there; that one says how to cross the floor to it
without driving through a part of the building nobody has measured.

## The floor, as a graph

The zones and the edges between them are the AWS warehouse's own.

```
   dock_north          receiving, under the north wall
       |
   corridor            the west corridor, the length of the building
       |
   dock_south -------- lane          the service lane, along the rack fronts
                        |  \
                      bay2  bay3     two aisles between the rack rows
```

There is no edge from `bay2` to `bay3`. That is not a simplification: the rack
rows span the east half of the building and stop 0.15 m short of the east wall,
which is nothing at all to a robot 0.21 m across. An aisle has one mouth, on
the lane, and a robot that looks into one and finds it empty comes back out the
way it came in. The plan pays for that, and you can see it pay.

## Running it

```bash
bash validate.sh
```

Needs [plank](https://github.com/a-burigana/plank) for parsing, type checking,
grounding and validation, and the epistemic planner for the search. Both are
found on `PATH`, or through `PLANK` and `EPISTEMIC_PLANNER`.

The whole thing prints in about half a second, which is right at a terminal and
useless in front of a camera. `PACE` puts a pause between the steps of the
trace:

```bash
PACE=0.8 bash validate.sh    # about a minute, and readable while it runs
```

The output stays inside 100 columns, so it will not wrap on a normal terminal.

It runs four things, in order.

### 1. One agent

r1 comes on shift at shipping. The pallet is in one of two aisles and r1 does
not know which, so the initial model designates two worlds. The goal asks for
both halves of the mission at once:

```
(delivered and [Kw. r1] pallet-at_bay2)
```

The first conjunct is ontic and could in principle be had by luck. The second
is not about the warehouse at all, it is about r1, and no amount of moving the
pallet satisfies it. So the plan has to sense, and having sensed it has to
branch — because what the inspection returns is a fact about the warehouse, not
something the planner gets to choose:

```
go_r1_dock_south_lane      [public-ontic]
go_r1_lane_bay2            [public-ontic]
inspect_r1_bay2            [semi-private-sensing]
  +-- e-inspect-empty   (not pallet-at_bay2)
      go_r1_bay2_lane  go_r1_lane_bay3  pickup_r1_bay3
      go_r1_bay3_lane  go_r1_lane_dock_south
      go_r1_dock_south_corridor  go_r1_corridor_dock_north
      unload_r1_dock_north
  +-- e-inspect-found   (pallet-at_bay2)
      pickup_r1_bay2
      go_r1_bay2_lane  go_r1_lane_dock_south
      go_r1_dock_south_corridor  go_r1_corridor_dock_north
      unload_r1_dock_north
```

Two leaves. A plan with more than one leaf is not a sequence; it is a policy,
and the branch taken is decided by what the robot finds. The empty branch runs
eight actions to the found branch's six, and the two extra are exactly the trip
back out to the lane and in again that the missing `bay2 -- bay3` edge forces.

### 2. Two agents

Same warehouse, and now r2 is waiting at receiving. The goal moves:

```
(delivered and [Kw. r2] pallet-at_bay2)
```

r2 has to know which bay the pallet came out of. r2 never leaves receiving —
it sits at the far end of the building and sending it is no cheaper — and r2
never looks into anything. It ends up knowing anyway, and the returned policy is the
same shape as the one-agent one.

The trace prints the model after every action, and that is where the two-agent
case becomes visible rather than merely argued:

```
inspect_r1_bay2   [semi-private-sensing]
   2 world(s), 2 designated   (both outcomes still open; they split at the branch)
 +-- e-inspect-empty   (not pallet-at_bay2)
     ^ seeing this is where r1 came to know
     go_r1_bay2_lane   [public-ontic]
        2 world(s), 1 designated
        * r1 knows it is not: pallet-at_bay2
        * r1 knows it is there: pallet-at_bay3
          r2 does not know whether: pallet-at_bay2
     ...
     pickup_r1_bay3   [public-ontic]
        1 world(s), 1 designated
        * r2 knows it is not: pallet-at_bay2
        ^ this is where r2 came to know, from a public-ontic action
```

r1 comes to know from the inspection. r2 does not -- it watches r1 inspect a
bay and does not see what was found, which is what semi-private means -- and
goes on not knowing for two more actions. What settles it for r2 is the
**pickup**: watching the pallet come out of `bay3` says which bay it was in.

Three things are worth noticing about it.

**Nothing was announced.** The domain offers `report-pallet-at`, a public
announcement, and the planner declines to spend an action on it. `pickup` is
public, so the fleet seeing r1 lift the pallet out of a bay is already enough:
r2 watching the pickup succeed in `bay3` learns that the pallet was in `bay3`.
The information travels on an action taken for another reason, and the planner
works that out rather than paying for it twice.

**The sensing is still semi-private.** The fleet sees r1 inspect a bay; it does
not see what r1 found. So the inspection alone does not settle anything for r2,
and it is the pickup that does.

**The sensing action does not settle anything by itself.** At the
`inspect_r1_bay2` line the model still designates two worlds. An inspection
splits the situation in two; which half is real is decided by the warehouse,
not by taking the action, and the designated set only shrinks at the branch
below it. That is why the plan has to be a policy.

### 3 and 4. Two plans that must be rejected

Both rejections are the point of the domain.

**A sequence, where a policy is needed.** Everything about it is right except
its shape: it drives to `bay2`, inspects, and then picks up regardless. After
the inspection the model still designates two worlds, and picking up in `bay2`
is applicable in only one of them — so `plank` reports it not applicable. A
robot running that plan is reaching for a pallet that is somewhere else half
the time.

**Picking up without looking.** `pickup` carries a modal precondition: a robot
may lift the pallet only where it *knows* the pallet is.

```
(at-ag_r1_bay2 and bay_bay2 and pallet-at_bay2
                and [r1] pallet-at_bay2
                and not carrying_r1)
```

Drop `[r1] pallet-at_bay2` from that conjunction and the domain is the
classical one, in which a robot that happens to be standing in the right bay
picks up a pallet it has no reason to believe is there, and a plan that never
inspects anything is valid.

## The files

| file | what it is |
| --- | --- |
| `warehouse-domain.epddl` | the domain: driving, inspecting, handling, reporting |
| `warehouse-lib.epddl` | the action types the domain draws on |
| `instances/problem_1.epddl` | one agent; the instance the ROS demo executes |
| `warehouse-problem.epddl` | two agents, and the knowing is somebody else's |
| `validate.sh` | grounds, solves and reads out both, then the two rejections |

The reader is
[`scenarios/warehouse/tools/show_plan.py`](../../scenarios/warehouse/tools/show_plan.py).
The model, the events and the plan are the tools' own -- plank grounds the
task, the planner searches it. The trace is not: following the model through
the plan needs the product update, and that much the reader computes.

It computes it *against* the planner rather than instead of it. At every node
the action's designated event must be applicable, and at every leaf the goal
must hold; if they do not, the reader prints the disagreement and exits
non-zero instead of a plausible-looking state. So the trace is checked by the
thing that produced the plan, and a drift between this file's semantics and
Aletheia's shows up as a failure rather than as a nice picture.

## Executing it

`instances/problem_1.epddl` is the instance the ROS demo runs, through
ePlanSys on PlanSys2:

```bash
ros2 launch warehouse_demo warehouse_demo_launch.py
```

The epistemic action names in the policy are joined to the classical durative
actions the executor runs by
[`warehouse-mapping.json`](../../ros2_ws/src/warehouse_demo/pddl/warehouse-mapping.json):
`inspect_r1_bay2` on this side is `(look_into r1 bay2)` there.
