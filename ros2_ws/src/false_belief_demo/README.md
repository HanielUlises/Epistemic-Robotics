# false_belief_demo

A crate is moved while the robot that stored it is away, and the robot that
moved it has to reason about where the other will look. The setting follows
the false-belief tasks of developmental psychology, the Sally-Anne test in
particular, as formalised in dynamic epistemic logic by Bolander: the move is
an action one robot does not observe, the model after it has that robot
believing something false, and the question is a belief about a belief
evaluated on that model.

On the pass-through floor, the picker stores a crate in bay `t1` and goes to
charge at a dock from which neither bay is in sight. The mover carries the
crate to `t3`. The picker is then to fetch it, and `fetch(picker, b)` requires
that the crate be in `b` and that the picker believe it is.

```
ros2 launch false_belief_demo false_belief_launch.py                  # told: a radio
ros2 launch false_belief_demo false_belief_launch.py floor:=untold    # no radio
ros2 launch false_belief_demo false_belief_launch.py floor:=doubt     # no radio, a doubting picker
bash tools/validate.sh                                                # six checks, no simulator
bash tools/record_demo.sh told /tmp/raw_told.mkv /tmp/run_told.log    # Xvfb recording
```

## The domain

`epddl/false-belief.epddl`, with two action types of its own in
`epddl/beliefs.epddl`.

The shift fixes its first two actions: the picker goes to its dock, a public
move, and the mover carries the crate from `t1` to `t3`. Two phase atoms hold
that order. The relocation has two events, the move and the mover's slot
passing with nothing moved, and three observability types:

| type | relation | for |
| --- | --- | --- |
| Fully | the identity | the mover |
| Oblivious | the move to the slot passing | the picker, on `told` and `untold` |
| Doubting | the two related | the picker, on `doubt` |

The slot passing carries the phase change, because the picker knows the
schedule: it knows the mover's slot has passed and does not know the mover
used it.

The report has two events, the report and the move as the picker takes it.
The picker relates the first to the second, so it takes the report as news
that the move happened, applied to the world it believes. Its precondition
includes `[mover] [picker] crate-at(t1)`: the mover reports only what it
believes the picker believes otherwise, which is the false-belief test as a
precondition.

`look(picker, b)` is semi-private sensing; `fetch(picker, b)` is public and
ontic. The three instances differ in two lines:

| instance | radio | doubts |
| --- | --- | --- |
| `told` | yes | no |
| `untold` | no | no |
| `doubt` | no | yes |

## What the logic says

After the shift's two actions, with the picker oblivious, the model has two
worlds and the picker's relation is no longer reflexive: the frame is KD45,
not S5. At the actual world

```
crate-at(t3) ∧ B_picker crate-at(t1) ∧ B_mover B_picker crate-at(t1) ∧ B_picker B_mover crate-at(t1)
```

The fourth conjunct is the answer to the Sally-Anne question: the picker will
look in `t1`.

An observation that a belief rules out leaves the believer with no world. If
the picker believes `¬ψ` and observes an event whose precondition entails `ψ`,
its relation from there is empty, and it believes every formula. Looking into
`t1` and finding it empty is such an observation, and so is a truthful public
announcement that the crate is in `t3`. Aletheia can keep such a state (its
default, as plank does), repair it, or refuse it; with
`--consistent-beliefs` it refuses it, and ePlanSys's planner plugin sets that
by default.

The report repairs the belief consistently: afterwards the picker believes the
crate is in `t3`, every world has a successor for each robot, and the location
is common belief. A doubting picker has no false belief to repair: after the
move it believes the crate in neither bay, its relation is an equivalence, and
a look into `t1` tells it the crate is in `t3`.

## What the planner returns

With consistent beliefs required:

| instance | policy |
| --- | --- |
| `told` | go-dock, relocate, report(mover, picker, t1, t3), fetch(picker, t3) |
| `untold` | none: the space is exhausted at depth 3 |
| `doubt` | go-dock, relocate, look(picker, t1), fetch(picker, t3) |

Without them, `untold` has a policy: look into `t1`, then fetch from `t3`, on
the picker's belief that the crate is in `t3`, which holds because the look
left it believing everything. `tools/validate.sh` checks all of this, and
`tools/trace.py` prints the beliefs after every update.

## On the floor

| action | performer | what it does |
| --- | --- | --- |
| `go-dock` | `go_dock_action` | drives the picker to its dock and logs which bays are in line of sight from there; refuses if one is |
| `relocate` | `relocate_action` | the mover drives in at the north mouth of `t1`, takes the crate onto its top plate, carries it to `t3` and sets it down; logs which bays are in sight of where the picker stands, and refuses if one is |
| `report` | `report_action` | puts the report on the picker's inbox and waits for it to arrive |
| `look` | `bay_action --kind look` | drives the picker to its mouth of the bay and reads its laser along the bay's axis: a crate reads at about three metres, an empty bay through to the far floor |
| `fetch` | `bay_action --kind fetch` | reads the bay the same way, refuses if it is empty, and otherwise takes the crate and carries it to the drop |

The crate rides on the robot that carries it, moved through
`gazebo_ros_state`: the Waffles have no gripper. On `untold` the planner has
no policy, and the mission runs what the picker would do on its own beliefs:
after the shift's two actions, fetch from `t1`. The executor checks the
picker's belief that the crate is there, which holds, and dispatches; the
performer reads `t1` empty and refuses. The mission reports the floor as the
expected outcome only if the epistemic state then says the crate is in `t3`,
the picker believes it in `t1`, and the mover believes the picker believes so.

The floor plan is `pass_through_demo`'s racking without the crate.
`tools/make_floorplan.py` checks, by casting rays over it, that the dock sees
neither bay, that from either mouth a robot sees along a bay to the far floor,
and that every place a robot is sent is reachable.

## The recorded runs

The three floors were recorded on a 3840 × 1080 Xvfb display, RViz on the
left half and Gazebo on the right, and composed into one film. Every figure
below is a line the runs wrote.

```
                        untold                        told                          doubt
plan                    none, exhausted at depth 3    4 nodes, 1 leaf               4 nodes, 1 leaf
go-dock(picker)         33 s; bays in sight: none     35 s; none                    36 s; none
relocate(mover, t1, t3) set down after 124 s          after 128 s                   after 129 s
  model after           2 worlds; B_picker t1         2 worlds; B_picker t1         2 worlds; B_picker neither
report                                                delivered; B_picker t3
look(picker, t1)                                                                    11.2 m: empty; K_picker t3
fetch(picker, b)        t1: 11.2 m, empty: refused    t3: 3.1 m, the crate          t3: 3.1 m, the crate
result                  the expected refusal          fetched, 131 s                fetched, 108 s
```

On the untold floor the mission then asked the epistemic state, and it
answered that the crate is in `t3`, the picker believes it in `t1`, the mover
believes the picker believes so, and the picker believes the mover believes it
in `t1`. Every node of the three runs exited cleanly at shutdown.

## What this does not show

* Belief revision. When an observation contradicts a belief, the logic used
  here has nothing to offer short of the empty relation, and the planner
  refuses it. Plausibility models (Baltag and Smets) would revise instead.
* The epistemic state applying such an observation. It tracks with Aletheia's
  default, which keeps the empty relation; the untold run never looks, so it
  never arises in a run.
* A physical carry. The crate is moved by the simulator once a robot is in
  position under it.

## Files

| file | contents |
| --- | --- |
| `epddl/false-belief.epddl` | the domain |
| `epddl/beliefs.epddl` | the relocation and the report |
| `epddl/told.epddl`, `epddl/untold.epddl`, `epddl/doubt.epddl` | the three floors, two lines apart |
| `tools/layout.py` | the floor, on pass_through_demo's |
| `tools/make_floorplan.py` | the floor plan and the three geometric claims |
| `tools/make_world.py` | the Gazebo world: the racking, the crate, the dock and the drop |
| `tools/make_mapping.py` | ground action names to the classical actions the executor dispatches |
| `tools/trace.py` | product update, first- and second-order beliefs, and where a robot is left with no world |
| `tools/validate.sh` | grounds, solves and traces; six checks |
| `tools/record_demo.sh` | one floor on an Xvfb display, RViz left and Gazebo right |
