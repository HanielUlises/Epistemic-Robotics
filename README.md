# Epistemic-Robotics

Autonomous robots operating in partially observable environments cannot plan over what they do not know. This project addresses that gap by grounding multi-agent task planning in Dynamic Epistemic Logic — a formal framework for reasoning about knowledge, ignorance, and how both evolve as agents perceive and communicate.

Each agent maintains a Kripke model of the world: a set of possible worlds, an accessibility relation encoding what the agent considers plausible, and a designated subset representing the actual situation as far as the agent can tell. Actions are not state transitions but epistemic events — product updates that refine the model. Navigation routes are winning regions of µ-calculus reachability formulas computed over the occupancy graph, not geometric shortest paths. The goal of a task may be that an agent reaches a zone, or that an agent *knows* it has reached it — these are different things, and the planner treats them differently.

Consider two robots, R₁ and R₂, assigned to retrieve an object from a corridor that may or may not be blocked. A classical planner assigns R₂ a route through the blocked corridor because it treats the world as globally known and generates the geometrically shortest path. When R₂ reaches the obstruction it simply fails. Under this framework, the initial epistemic state explicitly represents R₂'s uncertainty: two possible worlds, one where the corridor is free and one where it is not, both designated. The planner does not assign R₂ a route until R₁ — already near the corridor — executes a sensing action that collapses R₂'s belief state to a single world. The route follows from knowledge, not assumption.

The epistemic planner lives in [Aletheia](https://github.com/HanielUlises/Aletheia). The execution layer lives in [eplansys](https://github.com/ePlanSys/eplansys). This repository connects them to simulated robots: URDF descriptions running under ROS 2 and Gazebo. Hardware is out of scope.

## What is here

| | |
| --- | --- |
| `epddl-workspace/` | EPDDL domains and instances, with the tasks `plank` grounds from them. Twenty-one instances across muddy children, coin-in-the-box, active muddy child, doxastic depot and the box tasks. |
| `ros2_ws/src/mu_path_planner/` | µ-calculus reachability over the occupancy graph. A route as a winning region rather than a shortest path. |
| `ros2_ws/src/epistemic_slam/` | The collaborative layer over SLAM Toolbox: which regions each robot knows, and how two maps reconcile when a link comes back. |
| `ros2_ws/src/worlds/` | Gazebo worlds for the evaluation scenarios. |
| `ros2_ws/eplansys.repos` | Pulls eplansys and its dependencies into the workspace. |

The papers and the project site live on the `gh-pages` branch.

## Where the line falls

eplansys is a tool and it outlives this thesis. Anything reusable by someone who does not care about this research belongs there: the planner, the EPDDL front end, the epistemic state, the behavior tree nodes, and the bridge that turns an occupancy grid into epistemic atoms.

This repository is the thesis. The simulated fleet, the worlds those robots drive in, the collaborative SLAM layer, the µ-calculus route planning, and the experiments that measure whether any of it was worth doing.

## Building

```sh
cd ros2_ws
vcs import src < eplansys.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```