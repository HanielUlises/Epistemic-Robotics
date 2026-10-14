# watcher_demo

A carrier, an ally and a watcher. The carrier r1 reads an order naming bay X
(west) or bay Y (east) and must take its load there; the ally r2, in a booth
between the bays, must come to know which bay; the watcher r3 must not. The
watcher stands at one of two posts, west or east, and sees what happens on its
own side and nothing on the other. Every way of telling the ally is on a side:
the gate of each bay, which the booth sees, and each of the booth's two
windows, where the carrier can show a lamp. A back route along the corridor is
seen by nobody.

The goal is `in-bay ∧ Kw_r2 dest-x ∧ ¬Kw_r3 dest-x`. No sequence of actions
meets it in more than two of the four worlds, since every informing action is
on a side and the watcher may be on it. The epistemic policy reads a duty board
naming the watcher's post, which nothing in the goal mentions, and tells the
ally on the side the watcher is not. A classical goal cannot say that a third
robot must not know:

| formulation | planner | what it does |
| --- | --- | --- |
| `epistemic` | ePlanSys, Aletheia (AO*) | reads the duty board; the gate if the watcher is on the other side, otherwise the far window and the back route |
| `direct` | POPF | the gate of its bay, the shorter way |
| `discreet` | POPF, gates ruled out | the east window and the back route, for either bay |

Runs are judged by what the ally's and the watcher's own lasers recorded
(`src/observer_node.cpp`), not by the executor or the planner's model.

Page: <https://hanielulises.github.io/Epistemic-Robotics/watcher.html>.

## Layout

- `tools/layout.py` is the hall in one place: walls, bays, halls, gates, the
  booth and its windows, the posts, the corridor and its baffles, the boards,
  the starts and the roadmap. Every other file reads its geometry from it.
- `tools/check_views.py` casts straight lines from r2's station and from each
  post to every route, and checks what the domain assumes each sees.
- The EPDDL is in `epddl-workspace/watcher/`: the domain, the problem, and the
  library `sightlines.epddl` with the quasi-private ontic action type.
- `tools/make_pddl.py` writes the executor's vocabulary for the policy, the
  classical domain, and the mapping, into `pddl/`.
- `tools/make_world.py` writes a seed's world: seed mod 4 picks the order and
  the post, and every robot's pose is perturbed.
- `src/drive_node.cpp` drives the carrier along a route of the roadmap, holding
  in a gate for three seconds, and reports the event of the bay it went to;
  `src/sense_node.cpp` reads the two boards from the carrier's laser and shows
  the lamp; `src/observer_node.cpp` records what r2's and r3's lasers saw.
- `scripts/mission.py` runs a formulation; `scripts/recorder.py` records the
  actions, the three robots' poses and the lamp.
- `launch/watcher_launch.py` runs one headless run.

## Running it

```sh
bash tools/validate.sh                         # views, policy, no sequence, plank world by world, POPF, opacity
python3 tools/make_world.py --seed 2 --out RUN
ros2 launch watcher_demo watcher_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 2 discreet RUN 2         # one run in its own lane and process session
bash tools/batch.sh 1 40 BATCH 2               # every seed under all three formulations
python3 tools/analyse.py BATCH                 # runs.csv and the paired comparison
```

Runs use ROS domain 60 + lane and Gazebo port 11360 + lane, so several run side
by side and none shares a simulator with another session's.

## Defects found on the way

- ePlanSys's executor checks the modal conjuncts of all of an action's
  designated events as one conjunction, so events needing `K r1 dest-x` and
  `K r1 ¬dest-x` can never be dispatched. Every event here asks
  `Kw r1 dest-x` beside its own fact instead, which says the same.
- plank stops with an unknown exception when a domain and one of its
  action-type libraries share a name.
- A label `y` in a parameter file is read back by YAML as a boolean.
- The drive node once iterated over the array of a destroyed parameter object;
  the batch it crashed is kept in `artifacts/watcher/discarded/`.
