# Epistemic-Robotics

Research framework for multi-agent task planning under epistemic uncertainty, combining Dynamic Epistemic Logic (DEL) with ROS2-based robot execution.

The planning component (EHP — Epistemic Heuristic Planner) lives in a separate repository: [HanielUlises/Aletheia](https://github.com/HanielUlises/Aletheia). The execution layer (ePlanSys) lives in [HanielUlises/eplansys](https://github.com/HanielUlises/eplansys). This repository holds the ROS2 workspace, the µ-calculus path planner, and the DEL world manager that connect them.

---

## Repository structure

```
Epistemic-Robotics/
├── core/
│   └── planner/           # Standalone EHP binary (C++17, no ROS deps)
│       ├── include/       # action, bisimulation, formula, heuristic,
│       │                  # parser, product_update, search, state,
│       │                  # task, types, validator
│       └── src/
├── ros2_ws/
│   └── src/
│       ├── epistemic_msgs/        # ROS2 message definitions
│       ├── epistemic_state/       # EpistemicWorldManager node
│       ├── epistemic_slam/        # SLAM → DEL event bridge (TT-II)
│       └── mu_path_planner/       # µ-calculus path planner node
├── epddl-workspace/       # EPDDL domain/problem files and solved plans
└── lean/                  # Lean 4 formalisations (DEL, Kripke, planning)
```

---

## Core components

### `core/planner` — EHP (Epistemic Heuristic Planner)

Standalone C++ planner. Reads a ground JSON task (exported from plank/EPDDL) and writes a plan (linear or conditional JSON tree). No ROS2 dependency by design.

```bash
cd core/planner
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/ehp --task problem.json --plan out.json
```

Strategies: `--gbfs`, `--ehc`, `--conditional` (AO*). Default: auto-selected from task structure.

Heuristics: `--heuristic wc` (world count), `ug` (unsatisfied goal, default), `ed` (epistemic distance).

### `ros2_ws/src/mu_path_planner` — µ-calculus path planner

Computes backward reachability over the SLAM occupancy grid using least fixed-point iteration:

```
μZ. (at_goal ∨ (¬obstacle ∧ ⟨move⟩Z))
```

The `mu_calculus` library (`src/mu_calculus.cpp`) is independent of ROS2 and testable standalone via GTest. The ROS2 node wraps it, subscribing to `/map` and `/mu_planner/query`, publishing to `/mu_planner/path`.

When `require_epistemic_goal = true`, the formula lifts to:

```
μZ. (K_i(at_goal) ∨ (¬obstacle ∧ (⟨move⟩Z ∨ ⟨sense⟩Z)))
```

where `⟨sense⟩Z` introduces sensing waypoints derived from the current Kripke model.

### `ros2_ws/src/epistemic_state` — EpistemicWorldManager

Maintains the shared Kripke model M = (W, R₁..ₙ, V, W★). Applies DEL product updates triggered by EpistemicEvent messages. Publishes the current model as JSON on `/epistemic/state`.

### `ros2_ws/src/epistemic_slam` — SLAM bridge (TT-II)

Will convert LIDAR observations into EpistemicEvent messages, lifting sensor readings into private sensing actions. Currently a stub.

---

## Building the ROS2 workspace

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.zsh
```

Build order is handled by colcon via `package.xml` dependencies:
`epistemic_msgs` → `epistemic_state`, `mu_path_planner`, `epistemic_slam`.

---

## Running

```bash
# World manager
ros2 run epistemic_state epistemic_world_manager --ros-args -p num_agents:=2

# µ-calculus path planner
ros2 run mu_path_planner mu_path_planner_node

# Send a path query
ros2 topic pub /mu_planner/query epistemic_msgs/msg/MuPathQuery \
  "{agent_id: 0, goal_zone: 'zone_A', require_epistemic_goal: false}"
```

---

## Testing the µ-calculus planner standalone

The `mu_calculus` library has no ROS2 dependencies and can be tested without a ROS environment:

```bash
cd ros2_ws/src/mu_path_planner
cmake -B build -DBUILD_TESTING=ON
cmake --build build
./build/test_mu_calculus
```

---

## EPDDL workspace

The `epddl-workspace/` directory contains domain and problem files for the benchmark domains used in IEPC 2026 development:

- `muddy-children/` — classical DEL benchmark, multi-agent sensing
- `coin-in-the-box/` — public/private announcement
- `box-task/`, `box-task-2.0/` — multi-robot retrieval with partial observability
- `Active-Muddy-Child/` — active sensing variant

Solved plan trees (JSON) are included alongside each problem.

---

## Lean formalisations

`lean/Epistemic/` contains Lean 4 formalisations of the core theoretical objects: Kripke semantics, DEL product update, and the planning problem. These are auxiliary to the main implementation and are not required to build or run the ROS2 stack.

---

## Dependencies

| Component | Dependency |
|---|---|
| `core/planner` | C++17, nlohmann/json |
| `mu_calculus` lib | C++17, STL only |
| ROS2 packages | ROS2 Jazzy (or Humble), Nav2 |
| EHP planner (external) | [Aletheia](https://github.com/HanielUlises/Aletheia) |
| Execution layer (external) | [eplansys](https://github.com/HanielUlises/eplansys) |

---

## Status

This repository is under active development as part of a B.Eng. thesis at IPN-ESCOM on epistemic planning for multi-robot systems. The EHP planner participated in the IEPC 2026 smoke tests at ICAPS 2026.

Current state: ROS2 workspace bootstrapped, µ-calculus path planner operational, EpistemicWorldManager stub functional. SLAM bridge and full DEL integration are TT-II work.
