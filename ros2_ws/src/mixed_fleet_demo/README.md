# mixed_fleet_demo

Belief revision in a mixed fleet. The ward of `belief_revision_demo`, a week
later: a hospital floor at night, four fire doors, A to D from west to east,
each across the jog of a Z-shaped lobby through a band of linen rooms, so that
a door is seen only from a bend inside its lobby. At one in the morning the
night porter walks a loop through the band, north through one door and south
through another, and shuts both. No robot sees him.

Three robots of three makers share the floor, and each ranks the six loops the
porter may have walked its own way: a Spohn ranking, carried in every world of
the epistemic state as that robot's grade atom.

| robot | maker | ranking | starts in | delivers to |
| --- | --- | --- | --- | --- |
| `c` | the hospital | the porter's habits: rostered C-D 0, west A-B 1, the rest 2 | the day room (west) | ward 4 (north-east) |
| `d` | a disinfection contractor | door by door: doors differing from the rostered picture (Dalal) | the pharmacy (east) | ward 2 |
| `e` | a linen contractor | none: every loop 0, it believes only what it knows | the treatment room | ward 1 (north-west) |

Every robot routes itself on its own beliefs. A robot that finds a door shut
reports it, and every robot revises on the report by its own ranking, so the
same evidence leaves them believing different things, and each knows what the
others believe. The courier may also report a belief, "I believe door A is
shut": a robot that takes its word raises its rank of every world the report
rules out by its trust, 3. That is the one action that changes a grade; it is
an EPDDL event with one assignment per grade atom, and its guard is
second-order: the courier may say it only if it believes it, does not know it,
and believes the listener does not believe it.

| act | fleet | who takes the courier's word |
| --- | --- | --- |
| 1 | `c`, `d` | `d` |
| 2 | `c`, `e` | `e` |
| 3 | `c`, `d`, `e` | `d` and `e` |

| formulation | what the courier says |
| --- | --- |
| `egocentric` | it takes the others to rank as it does, so its beliefs are never news to them: nothing |
| `relevance` | every belief the guard allows, as soon as it allows it |
| `perspective` | a belief when telling it lowers the fleet's expected time, predicted on the beliefs the others do hold |

`alone`, the policy in which nobody reports a belief, is also planned;
`validate.sh` checks it is the egocentric policy in every act.

Page: <https://hanielulises.github.io/Epistemic-Robotics/mixed_fleet.html>.

## Layout

- `tools/layout.py` is the ward (from `belief_revision_demo`), with the day
  room's start and wards 3 and 4 added to the roadmap.
- `tools/ranks.py` holds the worlds, the robots and their rankings, the trust
  and the porter's frequencies; `tools/kripke.py` the reference semantics: an
  epistemic model with a ranking per robot, and the three updates the actions
  need (restrict, refine, shift).
- `tools/plan.py` plans: each robot routes itself on its own cell and ranks;
  the courier's belief reports are chosen per formulation, under
  `perspective` by a rollout of the policy without reports, each world of its
  cell weighted EPS ** rank. The team's policy is written in Aletheia's format;
  the script is the planner the AletheiaPlanSolver plugin calls.
- `tools/model.py` writes the EPDDL (`epddl-workspace/mixed-fleet/`);
  `tools/policy_check.py` replays every policy through plank's grounding and
  the repository's product update, and checks every robot's cell, grades and
  belief at every node against `tools/kripke.py`.
- `tools/make_pddl.py` writes the executor's vocabulary and the mapping;
  `tools/make_world.py` writes a run's world; `tools/check_views.py` checks the
  floor.
- `src/drive_node.cpp` drives a robot between the places an action names, each
  robot giving way to the ones after it in the fleet; `src/wait_node.cpp` runs
  looks and reports, a belief report going on `/radio` as `c:a~shut>d`;
  `src/porter_plugin.cpp` walks the porter; `scripts/porter_node.py` sends him
  on his round and swings the doors; `scripts/door_node.py` decides doors from
  a robot's laser at a bend; `scripts/mission.py` plans and executes;
  `scripts/recorder.py` records.

## Running it

```sh
bash tools/validate.sh                                   # floor, problems, every policy, Aletheia
python3 tools/plan.py --act 3 --formulation perspective --show
bash tools/run_one.sh 3 2 perspective RUN 1              # act 3, the west-loop night, lane 1
SEEDS_1="$(seq -s ' ' 1 12)" SEEDS_2="$(seq -s ' ' 1 12)" SEEDS_3="$(seq -s ' ' 1 12)" bash tools/batch.sh BATCH 6
python3 tools/analyse.py BATCH                           # runs.csv and analysis.json
```

Runs use ROS domain 40 + lane and Gazebo port 11440 + lane, the executor patch
of `belief_revision_demo` (`EXECUTOR_PATCH_WS`), and the hospital models from
`HOSPITAL_MODELS`.

## A plank quirk

plank compiles several conditional effects (`when`) on one atom into the
conjunction of their assignments, so an atom one `when` adds and another
deletes loses the addition, and an atom several `when`s add needs all their
conditions. `tools/model.py` writes the grade shift with one `iff` per grade
atom instead, an assignment plank compiles as written; `tools/policy_check.py`
found the first version's grades missing after a belief report.
