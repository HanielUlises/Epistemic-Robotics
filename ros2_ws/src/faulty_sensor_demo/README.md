# faulty_sensor_demo

A faulty sensor in the fleet. A hospital floor at night: four fire doors, A to
D from west to east, each near the north end of a straight passage through a
band of linen rooms five metres deep. On his round the night porter shut two
of the four; nobody on the floor saw which.

Scouts read the doors. A scout at a passage's mouth looks down it at the door
from more than five metres; a scout that has driven in to the mark before the
door reads it from 1.2 m. A laser may be short-sighted: it gets no return from
beyond 2.5 m, and the door rule counts a ray that returns nothing as a ray
that passed, as an occupancy grid does. Such a laser reads every door open
from a mouth and reads it right from the mark. So a reading "shut" is never
wrong, and a reading "open" from a mouth is as good as the sensor that took
it. No robot knows which lasers are short-sighted, its own included.

Bed-movers carry a bed from a room to a ward. A passage is as wide as a bed is
long: a bed-mover that has driven in cannot turn round, and if the door is
shut it backs out at a crawl. It drives in only when it is sure enough that
the door is open.

| formulation | an error belongs to | a bed-mover drives in when |
| --- | --- | --- |
| `credulous` | a sensor | it believes the door open: open in the most plausible worlds |
| `readings` | a reading | it believes it firmly, to degree 2: open in every world within one rank of the most plausible |
| `sensors` | a sensor | it believes it firmly, to degree 2 |
| `contact` | a sensor | it knows it; no scout looks from a mouth |

Under `sensors` a world is the porter's round and the set of short-sighted
scouts, and its Spohn rank the sum of their grades, 1 for a laser of the old
series and 2 for one of the new. Under `readings` a world is the round and,
for each shut door, which of the first and the second look at it errs, and its
rank the number of looks that err. Every world carries its rank as a grade
atom; belief of degree k is a formula over knowledge and the grades.

| act | scouts | bed-movers |
| --- | --- | --- |
| 1 | `s` (old series), from the porters' lodge | `b`, from the treatment room to ward 3 |
| 2 | `s` and `t` (both old), from the lodge and the day room | `b`, to ward 2 |
| 3 | `s` (old) from the lodge and `u` (new) from the pharmacy | `b` to ward 2 and `g`, from the day room, to ward 4 |

Page: <https://hanielulises.github.io/Epistemic-Robotics/faulty_sensor.html>.

## Layout

- `tools/layout.py` is the ward: the passages, each robot's own line along the
  hall and where it stands at a mouth, the roadmap.
- `tools/ranks.py` holds the nights, the fleets, the two models of error and
  the frequencies; `tools/kripke.py` the reference semantics: an epistemic
  model with one ranking, belief of degree k, and the two updates the actions
  need (refine, restrict).
- `tools/plan.py` plans: a search over what is known and where the robots
  stand, in the groups ePlanSys dispatches, for the least expected time to the
  last delivery. The policy is written in Aletheia's format; the script is the
  planner the AletheiaPlanSolver plugin calls. Policies are kept in the
  system's temporary directory, keyed by the sources.
- `tools/model.py` writes the EPDDL (`epddl-workspace/faulty-sensor/`);
  `tools/policy_check.py` replays every policy through plank's grounding and
  the repository's product update, and checks every robot's cell and every
  world's grade at every node against `tools/kripke.py`, and on which nights
  a bed-mover drives into a shut passage.
- `tools/schedule.py` is ePlanSys's scheduler, ported; `tools/make_pddl.py`
  writes the executor's vocabulary and the mappings; `tools/make_world.py`
  writes a run's world; `tools/check_views.py` checks the floor.
- `src/drive_node.cpp` drives a robot between the places an action names, and
  backs a bed-mover out of a passage; `src/wait_node.cpp` takes readings and
  says them on `/radio`, as `s:b=open`; `scripts/door_node.py` reads doors
  from a robot's laser, short-sighted if the night says so;
  `scripts/mission.py` plans and executes; `scripts/recorder.py` records.

## Running it

```sh
bash tools/validate.sh                                   # floor, problems, every policy, the schedule
python3 tools/plan.py --act 2 --formulation sensors --show
python3 tools/plan.py --act 2 --formulation sensors --show --night AB-s
bash tools/run_one.sh 2 6 sensors RUN 1                  # act 2, night AB-s (tools/worlds.py), lane 1
bash tools/batch.sh BATCH 6                              # every distinct path of every policy, once
python3 tools/analyse.py BATCH                           # runs.csv and analysis.json
```

Runs use ROS domain 40 + lane and Gazebo port 11440 + lane, the executor patch
of `belief_revision_demo` (`EXECUTOR_PATCH_WS`), and the hospital models from
`HOSPITAL_MODELS`.
