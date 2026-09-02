# The hotel incident

The Open-RMF hotel demo, with a mission whose world changes underneath it.

```bash
bash scenarios/hotel/record_demo.sh                  # films it, captions and all
LEAK=l3_suite bash scenarios/hotel/record_demo.sh    # the other branch
```

Without the recorder:

```bash
ros2 launch hotel_rmf_demo hotel_rmf_launch.py
ros2 launch hotel_rmf_demo hotel_rmf_launch.py leak:=l3_suite
```

## What it is

A leak is in one of two suites, on two different floors, and nobody knows
which. Finding out means riding a lift and looking. Shutting the valve is quiet
work in a closed suite, so the moment it is done every agent still downstairs
believes something that was true when the belief formed and is false now.
Repairing that without the guest overhearing is the rest of the mission.

The domain is `epddl-workspace/hotel-incident/`; the instance the demo runs is
`instances/problem_2.epddl`. The world, the three fleets and the two lifts are
the stock `rmf_demos` hotel, unmodified.

## Two robots at once

The policy opens with `deploy`, a single epistemic action that moves both
robots. This is not decoration. A policy is a chain of product updates and the
executor renders it as a `Sequence`, so two consecutive move actions are two
robots moving one after the other; concurrency between nodes would leave the
order of the updates ambiguous. Robots move together only when one event moves
both of them.

The bridge turns that one action into one RMF task per robot and holds the
action open until every one reports done, failing as soon as any one fails.

## The film

`record_demo.sh` places Gazebo and RViz side by side, grabs the strip, and
then burns in captions built from the run's own log by `tools/captions.py`.
Without them the recording is two robots taking lifts: what the mission is
about happens in the model and is invisible on the floor.

`SPEED` (4 by default) speeds up the film and leaves the captions readable;
`GEOMETRY` is the screen region to grab, for a machine that is not two
1920&#215;1080 monitors side by side.

## Output

```
out/hotel-<leak>.mp4            captioned, sped up
out/hotel-<leak>-raw.mp4        the grab, unedited
out/hotel-<leak>-segments.txt   the caption table
out/hotel-<leak>.log            the whole run
```
