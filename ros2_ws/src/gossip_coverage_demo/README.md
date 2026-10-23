# gossip_coverage_demo

Bramblett and Bezzo's "Epistemic planning for multi-robot systems in
communication-restricted environments" (Frontiers in Robotics and AI
10:1149439, 2023), recreated: its simulation study in a kinematic simulator,
the decision a robot makes when it finds a task as an EPDDL planning task that
Aletheia plans and ePlanSys executes, and its laboratory experiments in Gazebo
with three TurtleBot3 Burgers.

A team covers an unknown area and breaks radio contact to do it faster. When
all are in contact they share what they know, and from then on each robot
propagates the same particles, one per robot (the common belief), by the same
frontier coverage, and tracks its own: it stays where the others believe it to
be. A robot that finds a task needing several robots must tell them, by going
to where it believes them to be.

| formulation | what decides at a task found |
| --- | --- |
| `proposed` | the paper's framework: Algorithms 1 to 3 (auction, makespan, gossip auction), the rulesets of exclusion and of a task believed found by a teammate; a robot excluded is admitted again after 60 s |
| `ideal` | the same with a radio that reaches the whole area |
| `flock` | the team never leaves radio range: a formation whose centre explores the shared frontier |
| `epistemic` | a policy of `epddl-workspace/gossip-coverage`: the finder writes the planning task from what it knows, Aletheia plans it (`--strategy replan --subjective auto --consistent-beliefs`), and the policy is run as ePlanSys runs it: by `gossip/schedule.py`, a port of ePlanSys's `schedule_policy`, in the kinematic simulator, and by ePlanSys itself in Gazebo |

Runs are judged by mission time: every task done and every robot home, and in
Cases II and III the whole area seen.

## The library

`tools/gossip/` is shared by the simulator and the Gazebo mission:

- `env.py`: the area, the range sensor, the paper's scenario (`SIM`), the cases;
- `nav.py`: A* on the grid, path smoothing, the potential field of eqs. (6) to (8);
- `belief.py`: the common belief set, eqs. (9) to (14);
- `alloc.py`: Algorithms 1 to 3;
- `sim.py`: the simulator and the four formulations;
- `epddl.py`: the planning task as an EPDDL problem, plank and Aletheia;
- `epistemic.py`: the epistemic formulation: instance, policy, actions;
- `schedule.py`: the port of ePlanSys's scheduler.

## The kinematic study

`tools/study.py` runs the paper's Section 6: fifteen 50 by 50 m areas with 10
to 20 unknown obstacles, teams of 2, 3, 5 and 8, robots of 3 m/s with a 5 m
sensor, a 10 m radio. Case I has one task, Case II two or three needing no more
robots in all than the team has, Case III tasks needing more (the paper's
laboratory task mix). `tools/analyse.py study DIR` compares the procedure, the
ideal method and the flock with the paper's reported gaps.

## Gazebo

`tools/arena.py` is the paper's 4 by 5.5 m laboratory at twice its size: an 8
by 11 m arena with five crates of 0.5 m unknown to the robots, a 2 m range for
sensing and radio. Case I is its Fig. 10 (one task for two robots or all
three); Case II its Fig. 11 (a two-robot task that takes 40 s and a three-robot
task), placed so that the three-robot task is found while the two-robot task is
being done. `scripts/mission.py` runs the library with the Burgers as its
robots: positions from odometry, motions as velocity commands, the laser to stop
short. Under `epistemic` the instance is grounded by plank, loaded into
ePlanSys's epistemic state (`LoadTask`), planned by its planner through
`plansys2/AletheiaPlanSolver`, and run by its epistemic executor with parallel
dispatch; `src/relay_node.cpp` hands each action to the mission, which answers
with the event that occurred. The executor's vocabulary (`tools/make_pddl.py`,
`pddl/`) gives every action `(ok ?r)` of the robots it names, an arrival
`go_site_n` writes `(count_{n+1})`, and `do_task_k` reads every count below `k`.

Runs use ROS domain 90 + lane and Gazebo port 11490 + lane.

## Checks

`tools/validate.sh` needs no Gazebo: the arena's seeds; the four canonical
instances planned and replayed in every designated world by
`tools/check_policy.py`, with the paper's update rules as formulas and every
robot able to follow the policy on what it observes; the oblivious reading of
eq. (18) (`tools/oblivious.py`), under which no policy with consistent beliefs
exists; the executor's vocabulary; the scheduler; one kinematic run per method
and case. `tools/scaling.py` plans teams of two to eight.

## Running it

```sh
bash tools/validate.sh
python3 tools/study.py --out STUDY --cases 1 2 3 --methods ideal proposed flock epistemic
python3 tools/analyse.py study STUDY
python3 tools/study.py --one 3 5 2 epistemic        # one kinematic run, its events
python3 tools/scaling.py --out SCALE
python3 tools/make_world.py --seed 3 --case 2 --out RUN
ros2 launch gossip_coverage_demo gc_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 3 2 epistemic RUN 1           # one Gazebo run in its own lane
bash tools/batch.sh 1 12 BATCH 5                    # every seed, both cases, four formulations
python3 tools/analyse.py gazebo BATCH
```

Page: <https://hanielulises.github.io/Epistemic-Robotics/gossip_coverage.html>.

## Found on the way

- An arrival that did the task read only its own robot in the executor's
  vocabulary, so ePlanSys's scheduler moved it above the other arrivals, and its
  group waited forever; it now reads every count below its need.
- ePlanSys starts the next group of actions when all of the current one have
  finished, so an arrival answered at the site held the next calls for a drive;
  an arrival is answered when the robot sets out, and the task waits at the site.
- A launch `name=` renames every node of a process; the relay process hosts one
  action client per action, so it is launched without one.
- A common belief reset at every instant all robots were in contact started the
  particles facing north each time, and three robots in contact never left one
  another; it is reset when tasks, allocations or obstacles are shared, keeping
  each particle's heading and goal, or at the latest every 10 s.
- A particle sees through obstacles nobody has seen and could call covered a
  pocket its robot could not see; a cell at the edge of the robots' real map
  counts as seen in the belief only from beside it.
- Two policies can wait for each other; in contact, the task found first keeps
  its robots, and a wait is bounded, after which the members return to their
  particles and the finder plans again after a random back-off.
- SciPy in `/usr/local` does not load with the system NumPy that the run's
  `PYTHONNOUSERSITE=1` leaves; the library uses NumPy only where ROS runs it.
