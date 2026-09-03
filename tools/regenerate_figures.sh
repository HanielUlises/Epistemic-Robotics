#!/usr/bin/env bash
# Copyright 2026 Haniel Ulises
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Regenerates every figure the project site derives from a plan or a run.
#
# Each fragment is an HTML block written straight into a page on the gh-pages
# branch, so the figures cannot drift from the planner output and the recorded
# runs they are computed from. The two commands that read a run check
# themselves first: `model_check` requires its labelling to reproduce every
# verdict the executor recorded, and `plan_trace --against` requires the models
# it reconstructs to be bisimilar to the models that were measured.
#
#     tools/regenerate_figures.sh <output directory>

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="${1:?usage: regenerate_figures.sh <output directory>}"
mkdir -p "$out"
cd "$here"

hotel_goal='(and safe (K porter safe) (not (Kw guest leak-at_l2_suite)))'
warehouse_goal='(and delivered (Kw r2 pallet-at_bay2))'

# ── one agent knows: lone-inspector, solved by Aletheia ──
python3 tools/plan_trace.py diagram \
  --task epddl-workspace/lone-inspector/out/problem_1.json \
  --plan epddl-workspace/lone-inspector/out/problem_1-plan.json \
  --title 'The plan Aletheia returned, and the model at every node' \
  --out "$out/lone-diagram.html"

python3 tools/plan_trace.py update \
  --task epddl-workspace/lone-inspector/out/problem_1.json \
  --plan epddl-workspace/lone-inspector/out/problem_1-plan.json \
  --step 1 --id lone-pu \
  --title 'One product update: the inspection that refines the partition' \
  --out "$out/lone-update.html"

python3 tools/plan_trace.py matrix \
  --task epddl-workspace/lone-inspector/out/problem_1.json \
  --plan epddl-workspace/lone-inspector/out/problem_1-plan.json \
  --relation r --extra '(K r crate-at_bay2)' \
  --title 'The extension of each formula, node by node' \
  --caption 'The same run read formula by formula. The inspection removes no world and two edges: \(|W|\) is unchanged and \(|R_{\texttt{r}}|\) falls from four to two, which is what carries \([\![K_{\texttt{r}}\,\texttt{crate-at}\_\texttt{bay2}]\!]\) from \(\varnothing\) to the designated world and makes the pick-up applicable.' \
  --out "$out/lone-matrix.html"

# ── one agent believes: remote-door, and the repair the search does not find ──
python3 tools/plan_trace.py update \
  --task epddl-workspace/remote-door/out/problem_1.json \
  --plan epddl-workspace/remote-door/repair-witness.json \
  --step 0 --id door-pu \
  --title 'One product update: the command the agent does not observe' \
  --out "$out/door-update.html"

python3 tools/plan_trace.py diagram \
  --task epddl-workspace/remote-door/out/problem_1.json \
  --plan epddl-workspace/remote-door/repair-witness.json \
  --title 'The repair sequence, and the model at every node' \
  --out "$out/door-diagram.html"

python3 tools/plan_trace.py matrix \
  --task epddl-workspace/remote-door/out/problem_1.json \
  --plan epddl-workspace/remote-door/repair-witness.json \
  --relation r --extra '(K r (not shut))' \
  --title 'The extension of each formula along the repair' \
  --caption 'The command splits one world into two. At the designated world \(\texttt{shut}\) is true and \(K_{\texttt{r}}\,\neg\texttt{shut}\) holds there as well: the box modality is satisfied at a world its argument is false at, which is a false belief exhibited rather than argued for. The look removes one edge, and \(K_{\texttt{r}}\,\texttt{shut}\) acquires the designated world.' \
  --out "$out/door-matrix.html"

# ── a fleet believes: hotel-incident, checked against the run it was executed on ──
python3 tools/plan_trace.py update \
  --task epddl-workspace/hotel-incident/out/problem_2.json \
  --plan epddl-workspace/hotel-incident/plan-problem-2.json \
  --step 2 --take e-inspect-dry --id hotel-pu \
  --title 'One product update: the report the guest does not receive' \
  --out "$out/hotel-update.html"

python3 tools/plan_trace.py diagram \
  --task epddl-workspace/hotel-incident/out/problem_2.json \
  --plan epddl-workspace/hotel-incident/plan-problem-2.json \
  --against scenarios/hotel/out/analysis-l3.json \
  --title 'The policy Aletheia returned, and the model at every node' \
  --out "$out/hotel-diagram.html"

python3 tools/plan_trace.py matrix \
  --task epddl-workspace/hotel-incident/out/problem_2.json \
  --plan epddl-workspace/hotel-incident/plan-problem-2.json \
  --take e-inspect-dry \
  --relation inspector --relation porter --relation guest \
  --title 'The extension of each subformula of the goal, along the dry branch' \
  --caption 'The branch the recorded run took. The model grows rather than shrinks, and the two agents that end the run holding a false belief are the two whose relation loses an edge count it does not recover.' \
  --out "$out/hotel-matrix.html"

# ── the two recorded runs: extensions, and the goal test as a refutation ──
python3 tools/model_check.py extensions \
  --analysis scenarios/hotel/out/analysis-l3.json \
  --agents inspector porter guest --formula "$hotel_goal" \
  --extra '(K inspector safe)' \
  --title 'The extension of each subformula of the goal, after every action' \
  --out "$out/hotel-ext.html"

python3 tools/model_check.py cnf \
  --analysis scenarios/hotel/out/analysis-l3.json \
  --agents inspector porter guest --formula "$hotel_goal" \
  --title 'The goal test on the final model, as a propositional refutation' \
  --out "$out/hotel-cnf.html"

python3 tools/model_check.py extensions \
  --analysis scenarios/warehouse_rmf/out/analysis-bay2.json \
  --agents r1 r2 --formula "$warehouse_goal" \
  --extra '(Kw r1 pallet-at_bay2)' \
  --title 'The extension of each subformula of the goal, after every action' \
  --out "$out/wh-ext.html"

python3 tools/model_check.py cnf \
  --analysis scenarios/warehouse_rmf/out/analysis-bay2.json \
  --agents r1 r2 --formula "$warehouse_goal" \
  --title 'The goal test on the final model, as a propositional refutation' \
  --out "$out/wh-cnf.html"
