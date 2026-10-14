# lookout_board_demo

One robot must know whether a zone is blocked. It can see the zone through four
windows, one from each of four lookouts around the room the zone is in, but a
screen closes every window except one, and the robot does not know which. A
status board beside its start shows it: slot k is lit exactly when the view
from lookout k is clear. Nothing in the goal mentions the board.

The epistemic policy reads the board, drives to the lookout it shows lit, and
looks; after three dark slots it goes to lookout 4 without reading slot 4. The
classical formulations are given every action the epistemic one has, the board
included, and never read it:

| formulation | planner | what it does |
| --- | --- | --- |
| `epistemic` | ePlanSys, Aletheia (AO*) | reads the board, then the open lookout |
| `replan` | POPF, replanned after every look | nearest lookout not yet found screened, until one shows the zone |
| `conformant` | the shortest tour of the four lookouts | looks from all four, every time |

Page: <https://hanielulises.github.io/Epistemic-Robotics/lookout_board.html>.

## Layout

- `tools/layout.py` is the hall in one place: walls, windows and embrasures,
  screens, zone, lookouts, board, start and the roadmap the drive node plans
  over. Every other file reads its geometry from it.
- `tools/make_domain.py` writes the EPDDL domain for n lookouts into
  `epddl-workspace/lookout-board/`; `tools/make_pddl.py` writes the classical
  domain, the problems and the mapping into `pddl/`.
- `tools/make_world.py` writes a seed's world: which lookout is open and
  whether a crate stands in the zone, `(seed - 1) mod 8`, and the start.
- `src/drive_action_node.cpp` drives `goto` over the roadmap;
  `src/sense_action_node.cpp` serves `read_slot`, `check_view` and
  `look_zone`, ending on what the robot knows (epistemic) or on what the map
  decides (classical).
- `scripts/mission.py` gives the order in each formulation;
  `scripts/recorder.py` records regions, actions and pose.
- `launch/lookout_board_launch.py` runs one headless run.

## Running it

```sh
bash tools/validate.sh                         # views, policy, no sequence, POPF
python3 tools/make_world.py --seed 1 --out RUN
ros2 launch lookout_board_demo lookout_board_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 1 replan RUN 2           # one run in its own lane and process session
bash tools/batch.sh 1 32 BATCH 2               # every seed under all three formulations
python3 tools/analyse.py BATCH                 # runs.csv and the paired comparison
python3 tools/scaling.py --lookouts 2 3 4 5 6 7 8 --out scaling.json
```

Runs use ROS domain 50 + lane and Gazebo port 11350 + lane, so several run side
by side and none shares a simulator with another session's.
