# replanning_news_demo

A courier robot must carry a resident's medicine across a residence at night,
from the nurses' station in the south half to the resident's room in the
north, through one of three fire doors, any of which may be shut. At least one
is open, and nobody knows which. The courier crosses only a door it knows to be
open. The west and east doors can be seen from the mouths of their passages;
the middle passage turns twice, and its door can be seen only through a hatch
in the kitchenette beside it, where a scout waits.

While the policy runs, news arrives from outside the plan, and is announced
into ePlanSys's epistemic state with `epistemic_state/announce`. The
`CheckBeliefUnchanged` node in the running action fails on the next tick, the
executor halts the action, and the mission plans again with the in-process
epistemic solver, which plans from the model the state publishes.

| act | team | the news | its kind |
| --- | --- | --- | --- |
| 1 | courier | the east door is shut | a literal |
| 2 | courier, scout | the middle and east doors hang on one circuit | a relation between doors |
| 3 | courier, scout, cleaner | the cleaner has checked the east door (its message breaks off) | what one robot knows |

| formulation | at the news |
| --- | --- |
| `replan` | the news is announced; the policy is interrupted and planned again |
| `null` | a tautology is announced: interrupted and planned again from an unchanged model |
| `keep` | the news is heard and not announced; the policy runs on |
| `classical` | POPF, replanning over the valuations consistent with the news and the doors found shut |

Page: <https://hanielulises.github.io/Epistemic-Robotics/replanning_news.html>.

## Layout

- `tools/layout.py` is the residence in one place: walls, the band of linen
  rooms with the three passages and the kitchenette's hatch, the doors' leaves,
  floors, the AWS RoboMaker residential furniture, starts and the roadmap.
- `tools/furniture.py` reads the furniture's collision meshes and cuts them at
  a TurtleBot3's height; `tools/grid.py` rasterises the residence for a state
  of the doors, casts rays, and holds the rule a robot decides a door by.
- `tools/check_views.py` checks, in all eight states of the doors, that each
  straight passage's mouth decides its door, that the hatch decides the middle
  door, that the middle passage's mouth does not, and every route's clearance.
- `tools/model.py` writes the three problems and the same problems restricted
  by their news, with explicit Kripke models, into
  `epddl-workspace/replanning-news/instances/`; `tools/worlds.py` holds the
  runs' worlds and the news's timing.
- `tools/policy_check.py` checks Aletheia's policies against the domain's
  semantics: before the news, kept after it, and planned after it; that every
  message is sent where the speaker knows at every world; and replays the
  counterexample in which news breaks a policy.
- `tools/lazy.py` commits a policy's drives late: a drive of the courier is
  moved past another robot's look or message that does not need it, into its
  branches, and dropped where the courier's next action drives elsewhere. The
  result is a solution of the same depth (`tools/policy_check.py` checks it);
  the mission applies it to every epistemic policy.
- `tools/make_pddl.py` writes the executor's vocabulary, the mapping and the
  classical domain into `pddl/`; `tools/make_world.py` writes a run's world.
- `src/drive_node.cpp` drives over the roadmap from wherever the robot stands
  and stops the base when cancelled; `src/wait_node.cpp` runs the looks and
  the radio messages; `scripts/door_node.py` decides doors from each robot's
  laser; `scripts/news.py` says and announces the news; `scripts/mission.py`
  plans, executes and plans again; `scripts/recorder.py` records the run.

## Running it

```sh
bash tools/validate.sh                              # views, the six problems, every policy checked, POPF
python3 tools/make_world.py --act 3 --seed 4 --out RUN
ros2 launch replanning_news_demo news_launch.py dir:=RUN act:=3 formulation:=replan
bash tools/run_one.sh 3 4 keep RUN 1                # one run in its own lane and process session
SEEDS_3="1 2 3" bash tools/batch.sh BATCH 6         # seeds under the four formulations
python3 tools/analyse.py BATCH                      # runs.csv and analysis.json
```

Runs use ROS domain 50 + lane and Gazebo port 11450 + lane. The furniture is
read from the AWS small house world's installed models; `HOUSE_MODELS` names
another directory.

## Findings on the way

- Depth is not time. AO* minimises the worst branch, so it may start a drive
  before the look or message that decides whether the drive is needed. In a
  first batch, executing the policies as returned, planning again on the
  cleaner's message was 18.9 s slower than keeping the policy on average in
  act 3, and about 128 s slower in the worlds where the east door is open;
  committing late removed the loss.

- ePlanSys's `AletheiaPlanSolver` plans from the task file and not from the
  model the state publishes, so a replan with it forgets what the mission
  learned; the in-process `EpistemicPlanSolver` is used.
- `CheckBeliefUnchanged` takes its baseline at its first tick: news that lands
  while no action runs becomes the next action's baseline and interrupts
  nothing. Every run here delivers its news mid-drive.
- A halted action gets no terminal status on `/action_execution_info`.
