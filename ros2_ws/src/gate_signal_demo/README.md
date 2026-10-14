# gate_signal_demo

Two robots, two bays and no radio. An order board tells r1, the carrier, which
bay its load goes to; r2 must take its own load to the other bay, and may
deliver only into a bay it knows r1 will not use. From a booth inside the
racking r2 sees two gates and, of r1's routes, nothing else. r1 can reach its
bay by a back route nobody sees, or by a longer route through the gate of
that bay.

The goal asks no one to know anything; the knowledge is in the precondition of
r2's delivery. The epistemic policy sends r1 through its gate, where it stops,
and r2, watching, learns the order and drives to the other bay. Every policy
of the domain does so, since r2 can come to know the order in no other way.
The classical teams plan each robot alone with POPF, and r1 takes the back
route:

| formulation | planner | r2 without the order |
| --- | --- | --- |
| `epistemic` | ePlanSys, Aletheia (AO*), one policy for both | learns it at the gate |
| `optimistic` | POPF, one stack per robot | plans as if both bays were free: bay X |
| `cautious` | POPF, one stack per robot | drives into bay X and looks; crosses to bay Y if r1 is there |

Page: <https://hanielulises.github.io/Epistemic-Robotics/gate_signal.html>.

## Layout

- `tools/layout.py` is the hall in one place: walls, bays, passages, gates,
  the booth and its slits, the order board, the starts and the roadmap.
  Every other file reads its geometry from it.
- `tools/check_views.py` casts r2's laser from its station and checks what the
  domain assumes it sees.
- The EPDDL is in `epddl-workspace/gate-signal/`: the domain, the problem, the
  library `gate.epddl` with the semi-private ontic action type, and the variant
  `gate-signal-look.epddl` in which r2 can also look into bay X.
- `tools/make_pddl.py` writes the executor's vocabulary for the policy, the
  classical domain each robot plans with, and the mapping, into `pddl/`.
- `tools/make_world.py` writes a seed's world: the order (bay X for odd seeds)
  and both starts, each robot namespaced and with a coloured cap.
- `src/drive_node.cpp` drives a robot along a route of the roadmap, reporting
  the event of the bay it went to; `src/sense_node.cpp` reads the order, watches
  the gates, looks into a bay and unloads, from the robot's own laser.
- `scripts/mission.py` runs a formulation; `scripts/recorder.py` records the
  actions and both robots' poses.
- `launch/gate_signal_launch.py` runs one headless run: one PlanSys2 for the
  policy, or one per robot, namespaced, for the classical teams.

## Running it

```sh
bash tools/validate.sh                         # views, both policies, POPF; no simulator
python3 tools/make_world.py --seed 1 --out RUN
ros2 launch gate_signal_demo gate_signal_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 1 cautious RUN 2         # one run in its own lane and process session
bash tools/batch.sh 1 20 BATCH 2               # every seed under all three formulations
python3 tools/analyse.py BATCH                 # runs.csv and the paired comparison
```

Runs use ROS domain 50 + lane and Gazebo port 11350 + lane, so several run side
by side and none shares a simulator with another session's.
