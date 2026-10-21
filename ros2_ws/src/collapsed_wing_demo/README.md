# collapsed_wing_demo

Three robots enter a building nobody has mapped. A victim lies in its north
wing, which the south half reaches by three ways: a west doorway, an east
doorway, and between them a middle passage that winds through collapsed rooms,
the longest way and one no spot sees the whole of. Rubble may block any of the
three; at least one is clear. Two scouts come in by the side entrances, s1 by
the east and s2 by the west, and each can reach its own doorway. The medic m
starts in the entrance hall.

Every robot builds its own map with slam_toolbox from its own laser. A scout at
its doorway decides it on that map (`scripts/doorway_node.py`, `tools/grid.py`)
and can say over the radio what it found. The medic crosses into the north wing
only where it knows the way is clear.

The goal is `with-victim m`, and each crossing requires `K m clear-<way>`.
Aletheia's policy, of depth 7 over the seven worlds, sends both scouts out at
once; s1 looks at the east doorway and says what it found; if the doorway is
blocked, s2 looks at the west. If both are blocked, s2 says the middle passage
is clear, which nobody has seen: it knows by elimination, from what it saw and
what it heard s1 say.

| formulation | planner | what it does |
| --- | --- | --- |
| `epistemic` | ePlanSys, Aletheia (AO*) | the policy above; the medic crosses only a way it was told is clear |
| `optimistic` | POPF | every way assumed clear: the medic takes the shortest, finds rubble with its own laser, backs out to the hall and replans |
| `gated` | POPF | the medic's crossing gated on a scout's check, written as a fact: the checks run one after another, and nothing can check the middle passage, so where it alone is clear there is no plan |

Runs are judged by where the medic ended, by its own odometry, and every
doorway decision is checked against the world.

Page: <https://hanielulises.github.io/Epistemic-Robotics/collapsed_wing.html>.

## Layout

- `tools/layout.py` is the building in one place: walls, the serpentine middle
  passage, clutter, rubble, the regions a scout's map must decide, starts,
  vantages and routes. `tools/worlds.py` holds the seven worlds and the
  formulations.
- `tools/check_views.py` casts the rays and checks, in every world, that each
  scout at its vantage decides its own doorway right and not the other, that
  nothing sees the rubble in the middle passage, and that every route keeps its
  distance from walls and clutter.
- The EPDDL is in `epddl-workspace/collapsed-wing/`.
- `tools/make_pddl.py` writes the executor's vocabulary, the two classical
  domains and the mapping into `pddl/`.
- `tools/make_world.py` writes a seed's world: the seed picks one of the seven
  worlds in turn and perturbs every robot's pose.
- `src/drive_node.cpp` drives a robot along its route; with `stop_when_blocked`
  it stops and fails when rubble lies on the route ahead. `src/wait_node.cpp`
  runs the inspections, the classical checks and the radio messages.
- `scripts/doorway_node.py` decides each doorway on its robot's SLAM map;
  `scripts/mission.py` runs a formulation; `scripts/recorder.py` records the
  actions, the poses and the radio.
- `launch/wing_launch.py` runs one headless run.

## Running it

```sh
bash tools/validate.sh                              # views, the policy in every world, POPF
python3 tools/make_world.py --seed 6 --out RUN
ros2 launch collapsed_wing_demo wing_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 6 gated RUN 1                 # one run in its own lane and process session
bash tools/batch.sh 1 21 BATCH 4                    # every seed under the three formulations
python3 tools/analyse.py BATCH                      # runs.csv and analysis.json
```

Runs use ROS domain 80 + lane and Gazebo port 11480 + lane.

## Defects found on the way

- The executor dispatches together the actions of the policy whose vocabulary
  actions neither write what the other reads. With each action touching only
  its own robot's fact, the medic's crossing was dispatched beside the message
  it waits for, and the knowledge check stopped it. The vocabulary now carries
  `(radio)`, which every message writes and every crossing reads.
