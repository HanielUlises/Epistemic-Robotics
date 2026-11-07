# belief_revision_demo

Belief revision in a changing ward. A hospital floor at night: four fire doors,
A to D from west to east, close four lobbies through a band of linen rooms
between the hall and the corridor. Each lobby turns once, so its door is seen
only from a bend inside it. At one in the morning the night porter walks a
loop, north through one door and south through another, shutting both behind
him; at two he walks it back and opens them, unless he is held up elsewhere.
No robot sees him. The couriers must cross the band to the wards, and in the
third act come back.

A courier knows every loop is possible; it also has an idea of which is
likely. The porter's rostered loop is C-D; on some nights he walks the west
loop A-B; on rare nights another. Those habits are a Spohn ranking of the
worlds, carried in every world of the epistemic state as a grade atom. Belief
is a formula over knowledge and the grades, and revision is what a look or a
report does to knowledge: it removes worlds, and the least grade among those
left says what is now believed. Nothing in the domain changes a grade; every
relation stays an equivalence; plank, Aletheia and ePlanSys run it unchanged.

| act | team | what the act shows |
| --- | --- | --- |
| 1 | courier | a surprise revised by its explanation: B found shut, so the porter walked the west loop, so A is shut too, unseen |
| 2 | courier, second courier | a report of a shut door revises the listener the same way: it turns away from a lobby nobody looked into |
| 3 | both, there and back | the porter's walk at two turns what was known about shut doors into belief, and a second surprise is revised again |

| formulation | the ranking |
| --- | --- |
| `explanation` | the porter's habits: rostered loop 0, west loop 1, the rest 2; at two, reopening 0, held up 1 |
| `door-by-door` | minimal change (Dalal): how many doors differ from what is expected, as a map of independent cells ranks them |
| `full-meet` | the rostered picture believed and everything else equal: a KD45 belief that falls back on knowledge |
| `knowledge` | no belief beyond knowledge; the policy minimises the worst time |

A courier enters a lobby only if it does not believe its door shut, seeks
along the hall or corridor only if it does not believe every door ahead shut,
and crosses only a door it knows to be open; the executor checks each against
the epistemic state before dispatching it. Without grades, a KD45 belief has
no policy at all (Aletheia exhausts the search): its first surprise leaves the
courier believing nothing consistent.

Page: <https://hanielulises.github.io/Epistemic-Robotics/belief_revision.html>.

## Layout

- `tools/layout.py` is the ward in one place: walls, rooms, the band with its
  Z-shaped lobbies, the doors' leaves and their swing, floors, furniture,
  starts, beds, the roadmap and the porter's loops.
- `tools/furniture.py` reads the furniture's collision meshes (Open Robotics'
  hospital models from Gazebo Fuel, CC BY 4.0, and AWS RoboMaker's residential
  models, MIT-0); `tools/grid.py` rasterises the ward for a state of the
  doors, casts rays, and holds the rule a robot decides a door by.
- `tools/check_views.py` checks, in all sixteen states of the doors, that each
  door is decided right from both bends of its lobby, that from the hall and
  the corridor a laser gets fewer returns off any door than the rule needs,
  and every route's clearance.
- `tools/ranks.py` holds the worlds, the four rankings and the reference
  semantics of belief; `tools/model.py` writes the domain, the twelve problems
  and the KD45 control into `epddl-workspace/belief-revision/`.
- `tools/plan.py` plans: each courier on its own beliefs, minimising its
  expected time with each world weighted EPS ** rank (knowledge: its worst
  time), the team's policy composed in the order ePlanSys dispatches it, and
  written in Aletheia's format. It is the planner the AletheiaPlanSolver
  plugin calls, and the plugin validates what it returns.
- `tools/policy_check.py` replays every policy through the product update of
  the grounded task and checks, at every node, that each courier's belief
  read off the model's grades is the one the ranks give.
- `tools/sensitivity.py` charges each policy under other porter frequencies.
- `tools/make_pddl.py` writes the executor's vocabulary and the mapping;
  `tools/make_world.py` writes a run's world.
- `src/drive_node.cpp` drives a robot between the places an action names;
  `src/wait_node.cpp` runs looks, reports and the porter's round;
  `src/porter_plugin.cpp` walks the porter, a Gazebo actor;
  `scripts/porter_node.py` sends him on his rounds and swings the doors;
  `scripts/door_node.py` decides doors from a robot's laser at a bend;
  `scripts/mission.py` plans and executes; `scripts/recorder.py` records.

## Running it

```sh
bash tools/validate.sh                                  # the floor, twelve problems, every policy, Aletheia, KD45
python3 tools/make_world.py --act 2 --seed 2 --out RUN
ros2 launch belief_revision_demo revision_launch.py dir:=RUN act:=2 formulation:=explanation
bash tools/run_one.sh 2 2 explanation RUN 1             # one run in its own lane and process session
SEEDS_1="1 2 3 4 5 6" bash tools/batch.sh BATCH 7       # nights under the four formulations
python3 tools/analyse.py BATCH                          # runs.csv and analysis.json
python3 tools/sensitivity.py                            # other porter frequencies
```

Runs use ROS domain 60 + lane and Gazebo port 11460 + lane.

## The executor patch

PlanSys2's executor, as ePlanSys 0.3 ships it, keeps an action executor
requesting performers every second once its behavior tree node has been
ticked and left, since `ExecuteAction::halt()` does nothing. A performer that
answers such a stale request is held from the action actually due, which then
times out after 30 s: under the load of seven lanes, 6 of 52 runs failed so.
`patches/plansys2_executor-stale-requests.patch` makes an executor that is no
longer ticked stop asking. The runs use it from a workspace of its own:

```sh
mkdir -p ~/eplansys_patch_ws/src && cp -r ~/eplansys_ws/src/eplansys/plansys2_executor ~/eplansys_patch_ws/src/
(cd ~/eplansys_patch_ws/src && patch -p1 < .../belief_revision_demo/patches/plansys2_executor-stale-requests.patch)
(cd ~/eplansys_patch_ws && colcon build --packages-select plansys2_executor)
```

`tools/run_one.sh` sources it after ePlanSys (`EXECUTOR_PATCH_WS`). The hospital
models are read from `HOSPITAL_MODELS` (default
`~/plansys2_ws/src/ThirdParty/fuel_hospital`), the residential ones from
`HOUSE_MODELS`.
