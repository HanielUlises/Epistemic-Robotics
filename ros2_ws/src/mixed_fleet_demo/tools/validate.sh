#!/bin/bash
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
#
# Everything about belief revision in a mixed fleet that needs no simulator.
#
#   1. the floor: tools/check_views.py, in all sixteen states of the doors;
#   2. the problems: tools/model.py writes the domain and the three problems,
#      plank grounds them;
#   3. the policies: tools/policy_check.py builds each act's policy under each
#      formulation and replays it through the product update, checking every
#      robot's cell, grades and belief at every node against the reference;
#   4. the egocentric courier's policy is the one in which nobody reports a
#      belief, in every act;
#   5. Aletheia, unmodified, plans act 1 itself with consistent beliefs;
#   6. the expectations tools/plan.py predicts.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/mixed-fleet")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/watcher_overlay_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
OUT=${OUT:-$(mktemp -d)}

echo "== 1. the floor"
python3 "$HERE/check_views.py" | tail -2

echo "== 2. the problems"
python3 "$HERE/model.py" --out "$EPDDL"
for act in 1 2 3; do
  "$PLANK" export -d "$EPDDL/mixed-fleet.epddl" -p "$EPDDL/instances/act$act.epddl" -l "$LIB" -o "$OUT/task$act" > /dev/null 2>&1
done
echo "  three problems grounded: $(python3 -c "import json,sys; print(', '.join(str(len(json.load(open(sys.argv[i]))['actions'])) for i in (1,2,3)))" "$OUT/task1/act1.json" "$OUT/task2/act2.json" "$OUT/task3/act3.json") ground actions"

echo "== 3. the policies"
python3 "$HERE/policy_check.py" --formulations egocentric relevance perspective alone --out "$OUT" | sed 's/^/  /'

echo "== 4. the egocentric courier"
python3 - "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import plan as P
for act in (1, 2, 3):
    same = P.plan(act, 'egocentric')[1] == P.plan(act, 'alone')[1]
    print(f'  act {act}: the egocentric policy {"is" if same else "is NOT"} the policy without belief reports')
PY

echo "== 5. Aletheia"
timeout 600 "$PLANNER" --task "$OUT/task1/act1.json" --plan "$OUT/aletheia-act1.json" --conditional --consistent-beliefs \
    > "$OUT/aletheia-act1.log" 2>&1 || true
echo "  act 1: $(grep -E 'Solution found|No solution|exhausted' "$OUT/aletheia-act1.log" | head -1)"

echo "== 6. the expectations"
python3 - "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import plan as P, ranks as R
for act in (1, 2, 3):
    print(f'  act {act}: ' + '  '.join(f'{f} {P.expected(*P.plan(act, f)):.1f} s' for f in R.FORMULATIONS))
PY
echo "  written to $OUT"
