# link_grades_demo

Two robots, r1 at the west dock and r2 at the east station, must both come to
know whether a corridor along the north side of the hall is blocked. Only a
robot at the corridor's mouth can decide it with its laser. Each robot builds
its own occupancy map from its own laser, and what one map holds reaches the
other only over a link that epistemic_comm's `comm_monitor` and `link_gate`
take down. The link has three grades: never down (`full`), down from 10 s into
the mission for 20 + 4 × seed s (`partial`), and down all mission (`none`).

The goal is `Kw_r1 blocked ∧ Kw_r2 blocked`. Five formulations are compared:

| formulation | planner | what it does |
| --- | --- | --- |
| `epistemic_open` | ePlanSys, Aletheia (AO*) | the link's state unknown to both: both robots go and look |
| `epistemic` | ePlanSys, Aletheia (AO*) | the link read before planning and given as common knowledge: up, r1 looks and shares; a share not acknowledged within the wait fails, and the mission replans |
| `visit` | POPF | the problem without `(link)`: both robots go and look |
| `report` | POPF | the problem with `(link)`: r1 looks and reports, fire and forget |
| `replan` | POPF | `(link)` when the monitor says it is up; the report waits for an acknowledgement and fails without one, and the mission removes `(link)` and replans |

With the link unknown, a planner that minimises the depth of the worst branch
never tries the link: the branch in which the share is lost is one step deeper
than looking without it. So `epistemic_open` coincides with `visit`, and
`epistemic`, whose recovery is a timeout and a replanning step, coincides with
`replan`; the comparison on time measures the assumptions about the link.

Runs are judged by each robot's own map, not by the executor: a run is backed
when, at its end, each robot's merged map (its own cells, and the other's where
it has none) decides the corridor correctly (`tools/grid.py`).

Page: <https://hanielulises.github.io/Epistemic-Robotics/link_grades.html>.

## Layout

- `tools/layout.py` is the hall in one place: walls, corridor, debris, the
  region a map must decide, the dock, the station, the spots and the roadmap.
- `tools/grid.py` is the occupancy grid every robot builds and the rule by
  which a grid decides the corridor; `tools/check_views.py` casts the rays and
  checks that only a robot at the mouth decides it.
- `tools/grades.py` holds the grades' schedule, the formulations, the common
  start and the acknowledgement's wait, shared by the share and the report.
- The EPDDL is in `epddl-workspace/link-grades/`: the domain, the library
  `link.epddl` with the `lossy-message` action type, and four instances.
- `tools/make_pddl.py` writes the executor's vocabulary, the classical domain
  and the mapping into `pddl/`.
- `src/drive_node.cpp` drives a robot along its route; `src/wait_node.cpp`
  runs the inspection, the share and the report, the last two waiting for an
  acknowledgement where the formulation asks for one; `scripts/mapper_node.py`
  builds each robot's grid, merges what arrives over the link, and writes the
  grid's decisions and a snapshot of it every 2 s.
- `scripts/mission.py` runs a formulation; `scripts/recorder.py` records the
  actions, both robots' poses and the monitor's events.
- `launch/link_launch.py` runs one headless run.

## Running it

```sh
bash tools/validate.sh                                   # views, the four epistemic tasks, POPF
python3 tools/make_world.py --seed 15 --out RUN
ros2 launch link_grades_demo link_launch.py dir:=RUN formulation:=epistemic_open grade:=partial seed:=15
bash tools/run_one.sh 15 replan partial RUN 1            # one run in its own lane and process session
bash tools/batch.sh 1 20 BATCH 5                         # four formulations, three grades
FORMS=epistemic_open bash tools/batch.sh 1 20 BATCH 5    # the fifth
python3 tools/analyse.py BATCH                           # runs.csv and analysis.json
```

Runs use ROS domain 70 + lane and Gazebo port 11470 + lane, so that several run
side by side and none shares a simulator with another session's.

## Defects found on the way

- ePlanSys's executor checks the modal conjuncts of all of an action's
  designated events as one conjunction, so a share whose delivered events need
  `K r1 blocked` and `K r1 ¬blocked` can never be dispatched. Every event asks
  `Kw r1 blocked` beside its own fact instead, which says the same.
- The classical replan removed `(link)` from the problem expert's state by
  matching its text, which the problem expert prints as `( link )`; the replan
  kept the link and tried the report again. The match now allows spaces inside
  the parentheses.
