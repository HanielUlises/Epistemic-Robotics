# Epistemic-Robotics

Multi-agent task planning under partial observability, grounded in Dynamic Epistemic Logic. Each agent maintains a Kripke model of the environment: a set of possible worlds, a per-agent accessibility relation, and a designated subset standing for the situation as far as that agent can tell. Actions are epistemic events applied by product update rather than state transitions, and a goal may require that an agent reach a zone or that it *know* it has reached it. Route planning is the winning region of a µ-calculus fixed point over the occupancy graph rather than a geometric shortest path.

Validation is in simulation. Robots are URDF descriptions under ROS 2 and Gazebo; hardware is out of scope.

## Fixed points over partial maps

A cell that has never been observed is excluded from the reachability computation, since unknown is not free. The least fixed point therefore halts at the mouth of an unobserved corridor and reports no route. A sensing action resolves those cells, and the same computation then runs to completion.

| | |
| --- | --- |
| ![Reachability halted at an unobserved corridor](docs/img/sensing-before.png) | ![Reachability completing after the corridor is resolved](docs/img/sensing-after.png) |

**Figure 1.** Least fixed point before and after a sensing action. Cells marked `?` are unobserved.

When two goals must both be reached, the plan is a tree rather than a sequence: a shared approach followed by a branch, each subtree the winning region of its own reachability formula.

<p align="center">
  <img src="docs/img/branching-plan.png" alt="A shared approach followed by a branch toward two goals" /><br>
  <sub><b>Figure 2.</b> Branching plan over two targets.</sub>
</p>

The dual computation bounds where an agent may go rather than where it can arrive. The greatest fixed point retains the cells that remain within the known-free region under every step; its boundary is the exploration frontier, and the frontier is where a further sensing action is worth spending.

<p align="center">
  <img src="docs/img/safe-region.png" alt="The greatest fixed point and its frontier" /><br>
  <sub><b>Figure 3.</b> Safe known region and exploration frontier.</sub>
</p>

## A warehouse to run it on

The world the RoboticsAcademy [multi-robot Amazon warehouse
exercise](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/)
runs on — AWS RoboMaker's small warehouse — restated so that what the robots do
not know is part of the map. Not a floor plan that resembles it: the grid is
rasterised from the collision meshes Gazebo uses for that world, and the demo
launches that world unmodified, so the racks the planner drives around are the
racks the laser hits.

What the exercise asks for is a centralised task planner that assigns the jobs;
it is simply told where the pallet is. Here that is the whole problem. A part
of the floor nobody has measured is unknown rather than free, and which aisle
holds the pallet is a disagreement between two worlds one robot can tell apart
and another cannot.

```bash
ros2 launch warehouse_demo warehouse_demo_launch.py   # the mission, executed
bash scenarios/warehouse/run_demo.sh                  # cells: routes and where to look
bash epddl-workspace/robot-warehouse/validate.sh      # zones: what must be known
```

The first drives the policy in Gazebo through ePlanSys on PlanSys2, with
SLAM Toolbox building the map the µ-calculus planner routes over. The second
runs six questions past that planner, twice each — once in process and once
over ROS topics — and fails if the two answers differ. The third grounds the
warehouse domain with plank, solves it with Aletheia into a branching policy,
and prints the pointed model, the goal, the actions and the plan for one agent
and then for two — followed by two plans that must be rejected.

Written up in [`scenarios/warehouse/README.md`](scenarios/warehouse/README.md)
(the map and the routes) and
[`epddl-workspace/robot-warehouse/README.md`](epddl-workspace/robot-warehouse/README.md)
(the domain and the policies).

## Components

| Component | Role |
| --- | --- |
| [Aletheia](https://github.com/HanielUlises/Aletheia) | Epistemic planner. Heuristic search over pointed Kripke models under S5 and KD45 frames. |
| [plank](https://github.com/a-burigana/plank) | EPDDL parsing, type checking and grounding. Produces the tasks the planner reads. |
| [eplansys](https://github.com/ePlanSys/eplansys) | Execution layer on ROS 2, built on PlanSys2. Runs a policy and holds the epistemic state. |
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | 2D mapping. Each robot's occupancy grid. |
| [Nav2](https://github.com/ros-navigation/navigation2) | Navigation. Executes the ontic actions of a plan. |

This repository holds what is specific to the work: the simulated fleet and its worlds, the collaborative layer that tracks which regions each robot has observed and reconciles two maps when a link is restored, µ-calculus route planning, the warehouse scenario, and the experiments. The EPDDL domains and instances are under `epddl-workspace/`. Papers and the project site are on the `gh-pages` branch.
