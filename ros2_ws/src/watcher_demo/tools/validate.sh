#!/bin/bash
# Copyright 2026 Haniel Vásquez Morales
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
# Everything about the watcher that needs no simulator.
#
#   1. the views: tools/check_views.py;
#   2. the policy: plank grounds the task, Aletheia's AO* plans it;
#   3. no conformant plan: Aletheia's GBFS, which searches for one sequence
#      for all four worlds, exhausts;
#   4. world by world, plank's validator on every sequence a classical carrier
#      could follow and on each branch of the policy, over four copies of the
#      problem that each designate one world and leave every relation as it is;
#   5. the classical plans POPF returns;
#   6. opacity against a watcher that knows the policy (tools/opacity.py), under
#      the domain as declared and under watcher-sightlines.epddl.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/watcher")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

echo "== 1. the views"
python3 "$HERE/check_views.py" | tail -1

echo "== 2. the policy"
"$PLANK" export -d "$EPDDL/watcher.epddl" -p "$EPDDL/instances/problem_1.epddl" -l "$LIB" "$EPDDL/sightlines.epddl" \
  -o "$OUT" > /dev/null
"$PLANNER" --task "$OUT/problem_1.json" --plan "$OUT/plan.json" --conditional --consistent-beliefs --timeout 60 2>&1 \
  | grep -E 'Loaded|Solution|leaves'

echo "== 3. one sequence for all four worlds"
"$PLANNER" --task "$OUT/problem_1.json" --plan "$OUT/seq.json" --gbfs --no-helpful --timeout 60 2>&1 \
  | grep -E 'exhausted|Solution found|No solution'

echo "== 4. world by world"
for d in x y; do for w in west east; do
  python3 - "$EPDDL/instances/problem_1.epddl" "$OUT/$d-$w.epddl" "$d" "$w" <<'PY'
import sys
src, dst, d, w = sys.argv[1:]
s = open(src).read()
lit = ('(dest-x)' if d == 'x' else '(not (dest-x))') + ' ' + ('(west r3)' if w == 'west' else '(not (west r3))')
anchor = '      ([C. All] (carrier r1))'
assert anchor in s
open(dst, 'w').write(s.replace(anchor, '      ' + lit + '\n' + anchor, 1))
PY
done; done
v() {
  "$PLANK" validate -d "$EPDDL/watcher.epddl" -p "$OUT/$1.epddl" -l "$LIB" "$EPDDL/sightlines.epddl" -a $2 2>&1 \
    | grep -E '^(true|false)'
}
declare -A SEQ=(
  [classical-gate]="read-order_r1 gate_r1_r2_r3"
  [classical-window-w]="read-order_r1 to-win-w_r1 say-w_r1_r2_r3 back_r1"
  [classical-window-e]="read-order_r1 to-win-e_r1 say-e_r1_r2_r3 back_r1"
  [classical-silent]="read-order_r1 to-win-w_r1 back_r1"
  [policy-gate]="read-order_r1 peek_r1_r3 gate_r1_r2_r3"
  [policy-window-w]="read-order_r1 peek_r1_r3 to-win-w_r1 say-w_r1_r2_r3 back_r1"
  [policy-window-e]="read-order_r1 peek_r1_r3 to-win-e_r1 say-e_r1_r2_r3 back_r1"
)
printf '  %-20s %-7s %-7s %-7s %-7s\n' sequence x-west x-east y-west y-east
for k in classical-gate classical-window-w classical-window-e classical-silent policy-gate policy-window-w policy-window-e; do
  printf '  %-20s' "$k"
  for wd in x-west x-east y-west y-east; do printf ' %-7s' "$(v $wd "${SEQ[$k]}")"; done
  echo
done
echo "  the policy takes: gate in x-east and y-west, window-e in x-west, window-w in y-east"

echo "== 5. POPF"
python3 "$HERE/make_pddl.py" --out "$OUT/pddl" > /dev/null
cd "$HERE"
for o in x y; do for g in True False; do
  python3 -c "import make_pddl as P; open('$OUT/p.pddl', 'w').write(P.problem_deliver('$o', $g))"
  echo "  order $o, $([ $g = True ] && echo direct || echo discreet):" \
    $("$POPF" "$OUT/pddl/watcher-classical.pddl" "$OUT/p.pddl" 2>&1 | sed -n '/Solution Found/,$p' \
      | grep -oE '\((to_gate|to_window|signal|back_route)[^)]*\)' | tr '\n' ' ')
done; done

echo "== 6. against a watcher that knows the policy"
echo "  -- as declared"
python3 "$HERE/opacity.py" "$OUT/problem_1.json" "$OUT/plan.json" --agent r3 --secret dest-x | tail -2 || true
"$PLANK" export -d "$EPDDL/watcher-sightlines.epddl" -p "$EPDDL/instances/problem_sightlines.epddl" -l "$LIB" \
  "$EPDDL/sightlines.epddl" -o "$OUT/sl" > /dev/null
"$PLANNER" --task "$OUT/sl/problem_sightlines.json" --plan "$OUT/sl/plan.json" --conditional --consistent-beliefs --timeout 60 2>&1 \
  | grep -E 'Solution'
echo "  -- declared by the sightlines"
python3 "$HERE/opacity.py" "$OUT/sl/problem_sightlines.json" "$OUT/sl/plan.json" --agent r3 --secret dest-x | tail -1
