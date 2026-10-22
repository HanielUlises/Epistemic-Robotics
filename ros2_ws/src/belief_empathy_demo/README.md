# belief_empathy_demo

Bramblett and Bezzo's "Epistemic Planning for Heterogeneous Robotic Systems"
([arXiv:2308.00579](https://arxiv.org/abs/2308.00579)), recreated on ePlanSys and
compared with the paper's own methods in Gazebo.

An aerial vehicle `u` and two ground vehicles, `g1` and `g2` (TurtleBot3
Burgers), leave a base together, split a 30 by 16 m area in proportion to speed
and cover it out of radio contact, each along a route every robot knows. The
radio reaches 2.5 m. Each robot keeps three belief particles for each ground
vehicle: where it is with no fault, one or two, at 100, 60 and 20 per cent of
the nominal speed, as in the paper. Faults happen at times the seed fixes, and a
vehicle with a fault goes to its next particle and holds it, which is the
paper's empathy. A vehicle with two faults can no longer do a task.

The aerial vehicle finds a task that needs it and a ground vehicle with fewer
than two faults at the site together. Only a hail at the right particle can
tell a ground vehicle of it.

| formulation | what decides | radio |
| --- | --- | --- |
| `epistemic` | Aletheia's policy over `belief-empathy.epddl`, computed when the task is found, run by ePlanSys's epistemic executor | 2.5 m |
| `particles` | the paper's procedure: least estimated makespan over the particles still held possible, estimated again after every hail | 2.5 m |
| `ideal` | the paper's ideal method: every robot hears every other, so the vehicle of least makespan is told at once | unlimited |
| `flock` | the paper's flock method: the team sweeps the area together in contact, at the speed of its slowest member | 2.5 m |

Runs are judged by when the task was done, by the simulator's poses.

## The variant with latent tool faults

`VARIANT=latent` runs a variant the paper's particles cannot represent. A drive
fault slows a vehicle, which tracks its next particle, and costs it nothing
else; a tool fault leaves speed and particle unchanged and is known to nobody,
the vehicle included, until it attempts the task, which then fails. At most two
faults of either kind, and at least one tool works: twelve patterns
(`belief-empathy-latent.epddl`). Five formulations:

| formulation | what decides |
| --- | --- |
| `epistemic` | Aletheia's policy, least in worst-case steps, no durations |
| `timed` | a policy from `tools/timed_policy.py`, a timed AND-OR search over the same grounded task that minimises expected time; computed beforehand, one per site, in `policies/`, and run through the same plugin and executor by `tools/fixed_policy.sh` |
| `published` | the paper's procedure: each vehicle's particles alone, no joint budget, a vehicle whose attempt fails dropped |
| `strong` | the procedure at its strongest: every pattern consistent with what u learned, the budget and the assumption that some tool works; one vehicle or both, by least mean makespan |
| `ideal` | the strong procedure knowing every vehicle's drive faults |

`tools/estimate.py` is the kinematic model and the procedures' decisions;
`tools/precheck.py` times every formulation in it before any run, and takes
apart the gap between Aletheia's policy and the strong procedure;
`tools/compare.py` does the same for the timed policy.

Page: <https://hanielulises.github.io/Epistemic-Robotics/belief_empathy.html>.

## Layout

- `tools/layout.py` is the area in one place: streets, obstacles, the base, the
  partition, every route, the particles and the intercept of a particle, the
  ground roadmap and the flock's paths. `tools/worlds.py` holds the six fault
  patterns, the seeds and the formulations.
- `tools/check_layout.py` checks that every way a ground vehicle drives clears
  the obstacles, that a vehicle's three particles stay further apart than the
  radio's range whenever a hail can happen, that every site is found after the
  last fault, and that the flock stays in contact.
- The EPDDL is in `epddl-workspace/belief-empathy/`.
- `tools/make_pddl.py` writes the executor's vocabulary and the mapping into
  `pddl/`. `tools/trace_policy.py` replays a policy through the product update
  in every world and evaluates the paper's update rules as formulas.
- `tools/make_world.py` writes a seed's Gazebo world: the obstacles, the site,
  two Burgers, and a quadrotor flown in the plane by `gazebo_ros_planar_move`
  with gravity off.
- `scripts/ugv_node.py` and `scripts/uav_node.py` are the robots: coverage,
  faults, the particle a vehicle holds, hails, and the requests every
  formulation drives them by. `scripts/world_node.py` is the radio and the task.
  `src/relay_node.cpp` hands an action of the executor to a robot's node.
- `scripts/mission.py` runs a formulation; `scripts/recorder.py` records the
  actions, the poses, the radio and the events.
- `launch/be_launch.py` runs one headless run.

## Running it

```sh
bash tools/validate.sh                       # the area, both policies in every world, the update rules, the oblivious reading
python3 tools/make_world.py --seed 11 --out RUN
ros2 launch belief_empathy_demo be_launch.py dir:=RUN formulation:=epistemic
bash tools/run_one.sh 11 particles RUN 1     # one run in its own lane and process session
bash tools/batch.sh 1 24 BATCH 6             # every seed under the four formulations
python3 tools/analyse.py BATCH               # runs.csv and analysis.json
python3 tools/scaling.py OUT --max 8         # the policy's search for teams of 2 to 8 ground vehicles
python3 tools/precheck.py OUT                # the variant in the kinematic model, before any run
VARIANT=latent bash tools/batch.sh 1 24 BATCH 6     # the variant under its five formulations
```

Runs use ROS domain 60 + lane and Gazebo port 11460 + lane.

## Found on the way

- ePlanSys's executor dispatches together consecutive actions of the policy
  only when the agents they name are disjoint. A drive to the site that also
  named the aerial vehicle, so that it would observe the drive, ran after the
  aerial vehicle's flight to the site instead of with it. The drive now names
  the ground vehicle alone, and the aerial vehicle is unsure of it like
  everyone else.
- An observer out of contact written as `Oblivious` makes the vehicle that is
  hailed believe a contradiction (see the EPDDL's README); it is `Unsure`.
- Detection within 1.2 m of a site was narrower than the 4 m between lanes
  assumes; in the flock a site on a street was then seen only by the ground
  vehicle driving it, and when that vehicle left with two faults the task was
  never found. Detection is now the coverage's half-width, 2.2 m, checked by
  `tools/check_layout.py`, and the batch was rerun.
- `layout.at()` found the last segment of a path by identity, `b is path[-1]`,
  and Python shares equal tuple constants, so a path that passed a point twice
  ended at its first visit. It compares indices now.
