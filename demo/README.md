# Demo: the mu-calculus planner over the six-room world

A scripted run of `mu_path_planner` over `worlds/02_building_rooms.sdf`, with a
TurtleBot3 driving whatever route comes back. It exists because the planner in
this repository has no map source, no snapshot publisher and no launch file of
its own: those are what this directory supplies, and they are deliberately kept
outside the packages so that nothing here is a dependency of anything there.

## Running it

    ros2 launch is not involved. Source the workspace, then:

    ./demo/run_demo.sh out.mp4     # gazebo + rviz side by side, recorded
    REHEARSE=1 ./demo/run_demo.sh  # the same, without recording
    ./demo/smoke.sh                # headless, no GUI, for checking the logic

The recorder captures a 3840x1040 region across the first two monitors with
ffmpeg's x11grab; edit `run_demo.sh` if your desktop is arranged differently.

## What the script shows

The nine steps are chosen to separate what the robot can reach from what it can
know, which is the distinction the package is about:

| Step | What it asks | What comes back |
| --- | --- | --- |
| 1-2 | ontic goals, room2 then room5 | routes, driven |
| 3 | the target, in rooms nobody observed | no route: unknown is not free |
| 4-5 | room3 sensed, the same query again | a route, because the cells became free |
| 6-7 | an epistemic goal over a disputed target | sensing waypoints, not a route |
| 8-9 | the observation collapses the model | one world left, so the agent knows, and goes |

Steps 3 and 6 are the point. A cell nobody observed is excluded from the fixed
point rather than driven through, so the least fixed point halts at the mouth of
an unobserved room; and a goal whose extent differs between the worlds an agent
holds possible cannot be *known* to be reached, however easy it is to reach.

## The pieces

| File | What it does |
| --- | --- |
| `make_world.py` | Writes the demo world: the shipped world plus a robot, a camera and the state plugin |
| `world_grid.py` | Rasterises the world SDF into the OccupancyGrid the planner subscribes to |
| `demo_driver.py` | Publishes `/map` and the Kripke snapshot, then walks the nine steps |
| `follow_path.py` | Drives the route: shortcutting, a smoother, and a dynamic window over the laser |
| `demo.rviz` | Map, zones, route, smoothed route, sensing waypoints, robot |

## Notes for anyone changing it

Both controllers run on `/clock`. The GUI does not simulate at wall-clock
speed, and a controller ticking on wall time issues several commands per
simulated step: the velocity window opens faster than the base can accelerate
and the robot overshoots into whatever it was rounding.

`V_MAX` is 0.35 m/s. A burger's own top speed is 0.22, and commanding much past
that means the wheels saturate, the command is not tracked, and odometry
integrates a motion the robot did not make.

The map is inflated by 0.25 m in `demo_driver.py`. Raising it to 0.35 closes the
0.8 m doorway into room 3 and the planner then correctly reports no route at
all -- a fact about the inflation, not about the building.

Leftover processes from an earlier run are not harmless: a second planner
answers the same query with a second path, and a second follower fights the
first for `/cmd_vel`. Both scripts sweep before they start, and refuse to run if
the sweep did not take.
