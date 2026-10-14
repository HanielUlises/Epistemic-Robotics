# watcher_moves_demo

The watcher of `watcher_demo`, which may now change posts and may know the
policy. The carrier r1 must still take its load into the bay its order names,
let the ally r2 know which, and not let the watcher r3 know. The hall is
watcher_demo's with the posts nearer the block and a passage behind the north
wall, by which the watcher changes posts out of everyone's sight
(`tools/layout.py`, checked by `tools/check_views.py`).

| formulation | planner | domain |
| --- | --- | --- |
| `v1` | ePlanSys, in the executor | `epddl-workspace/watcher/watcher.epddl`, the published policy |
| `shortest` | Aletheia, `--subjective auto` | `epddl-workspace/watcher-mobile/gates`, one change of post |
| `opaque` | Aletheia, `--subjective auto` | `epddl-workspace/watcher-mobile/opaque`, no gates |
| `direct`, `discreet` | POPF | the classical domain of watcher_demo |

The watcher's strategy (`scripts/watcher_node.py`) is `stay`, or `chase`:
the moment the carrier starts its first drive it leaves for the other post,
where a carrier that read the duty board would tell. Runs are judged by what
the ally's and the watcher's own lasers recorded (`src/observer_node.cpp`),
and by a watcher that knows the policy from the other runs of the batch
(`tools/analyse.py`).

## What it needs beyond watcher_demo

- Aletheia with subjective branching, the branch `feature/subjective-branching`
  (`~/aletheia-subjective`): `--subjective auto` makes each action branch on
  what its actor observes.
- ePlanSys's `plansys2_epistemic_executor`, `plansys2_epistemic_planner` and
  `plansys2_aletheia_plan_solver` built over it, with a parameter for
  subjective branching in the state node and in the plugin, in an overlay
  workspace (`~/watcher_overlay_ws`) sourced after `eplansys_ws`. The
  installed ePlanSys is left as it was.

## Running it

```sh
python3 tools/check_views.py                         # the sightlines, the passage and every drive
bash tools/run_one.sh 1 opaque chase RUN 1           # one run, in lane 1 (ROS domain 61)
bash tools/batch.sh 1 20 BATCH                       # every seed, five formulations, two strategies
python3 tools/analyse.py BATCH                       # runs.csv and analysis.json
```

The planning study, the regression of subjective branching on every other
domain, and the opacity checks are in `epddl-workspace/watcher-mobile/tools`.
