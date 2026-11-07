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
# Everything about belief revision in a changing ward that needs no simulator.
#
#   1. the floor: tools/check_views.py, in all sixteen states of the doors;
#   2. the problems: tools/model.py writes the domain and the twelve problems,
#      plank grounds them, tools/plan.py plans each;
#   3. the policies: tools/policy_check.py replays each through the product
#      update and checks every belief against the ranks;
#   4. Aletheia, unmodified, on act 1 ranked by explanation and by knowledge,
#      and on the KD45 control, where it finds no policy;
#   5. the expectations tools/plan.py predicts.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/belief-revision")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/watcher_overlay_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
OUT=${OUT:-$(mktemp -d)}

echo "== 1. the floor"
python3 "$HERE/check_views.py" | tail -2

echo "== 2. the problems"
python3 "$HERE/model.py" --out "$EPDDL"
for act in 1 2 3; do
  for f in explanation door-by-door full-meet knowledge; do
    d="$OUT/act$act-$f"
    "$PLANK" export -d "$EPDDL/belief-revision.epddl" -p "$EPDDL/instances/act$act-$f.epddl" -l "$LIB" -o "$d" > /dev/null 2>&1
    mv "$d/act$act-$f.json" "$d/task.json"
    python3 "$HERE/plan.py" --task "$d/task.json" --plan "$d/policy.json" --act "$act" --formulation "$f" > /dev/null
  done
done
echo "  twelve problems grounded and planned"

echo "== 3. the policies"
python3 "$HERE/policy_check.py" "$OUT" | sed 's/^/  /'

echo "== 4. Aletheia"
for f in explanation knowledge; do
  "$PLANNER" --task "$OUT/act1-$f/task.json" --plan "$OUT/act1-$f/aletheia.json" --conditional --consistent-beliefs \
      > "$OUT/act1-$f/aletheia.log" 2>&1 || true
  echo "  act 1 $f: $(grep -E 'Solution found|No solution' "$OUT/act1-$f/aletheia.log" | head -1)"
done
"$PLANK" export -d "$EPDDL/kd45/belief-revision-kd45.epddl" -p "$EPDDL/kd45/act1-kd45.epddl" -l "$LIB" -o "$OUT/kd45" > /dev/null 2>&1
"$PLANNER" --task "$OUT/kd45/act1-kd45.json" --plan "$OUT/kd45/plan.json" --conditional --consistent-beliefs \
    > "$OUT/kd45/aletheia.log" 2>&1 || true
echo "  act 1, KD45 without ranks: $(grep -E 'Search space exhausted|Solution found|No solution' "$OUT/kd45/aletheia.log" | head -1)"

echo "== 5. the expectations"
python3 - "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import plan as P, ranks as R
for act in (1, 2, 3):
    print(f'  act {act}: ' + '  '.join(f'{f} {P.expected(act, f, P.Team(act, f).tree()):.1f} s' for f in R.FORMULATIONS))
PY
echo "  written to $OUT"
