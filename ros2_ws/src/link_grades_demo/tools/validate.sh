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
# Everything about the link grades that needs no simulator.
#
#   1. the views: tools/check_views.py;
#   2. the policy with the link's state unknown (problem_open): plank grounds
#      the task, Aletheia's AO* plans it, and the policy never shares;
#   3. the policy with the link up and common knowledge (problem_up): r1
#      looks and shares, r2 does not move, and no branch is lost;
#   4. with the link down (problem_down), and after a lost share
#      (problem_lost): both robots look, or r2 looks;
#   5. the classical plans POPF returns, without (link) and with it.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/link-grades")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

echo "== 1. the views"
python3 "$HERE/check_views.py" | tail -1

policy() {
  # $1: the instance; prints the planner's verdict and the policy, and checks $2
  local p=$1 check=$2
  mkdir -p "$OUT/$p"
  "$PLANK" export -d "$EPDDL/link-grades.epddl" -p "$EPDDL/instances/problem_$p.epddl" \
      -l "$LIB" "$EPDDL/link.epddl" -o "$OUT/$p" > /dev/null
  "$PLANNER" --task "$OUT/$p/problem_$p.json" --plan "$OUT/$p/plan.json" --conditional --consistent-beliefs 2>&1 \
      | grep -E 'Solution found|validator' | sed 's/^/  /'
  python3 - "$OUT/$p/plan.json" "$check" <<'PY'
import json, sys
plan, check = json.load(open(sys.argv[1])), sys.argv[2]
acts, lines = set(), []
def walk(n, d=0):
    if n is None:
        return
    acts.add(n['action'])
    lines.append('    ' + '  ' * d + n['action'] + ('' if len(n['branches']) == 1 else f'  ({len(n["branches"])} branches)'))
    for b in n['branches']:
        walk(b['subtree'], d + (len(n['branches']) > 1))
walk(plan)
print('\n'.join(dict.fromkeys(lines)))
share = any(a.startswith('share') for a in acts)
r2 = 'goto-junction_r2' in acts
ok = {'open': not share and r2, 'up': share and not r2, 'down': not share and r2, 'lost': not share and r2}[check]
print('  ' + ('OK' if ok else 'FAIL') + f': share {"in" if share else "not in"} the policy, r2 {"drives" if r2 else "stays"}')
sys.exit(0 if ok else 1)
PY
}

echo "== 2. the link unknown"
policy open open
echo "== 3. the link up"
policy up up
echo "== 4. the link down, and after a lost share"
policy down down
policy lost lost

echo "== 5. POPF"
python3 "$HERE/make_pddl.py" --out "$OUT/pddl" > /dev/null
cd "$HERE"
for link in False True; do
  python3 -c "import make_pddl as P; open('$OUT/p.pddl', 'w').write(P.problem_start($link))"
  echo "  $([ $link = True ] && echo 'with (link):   ' || echo 'without (link):')" \
    $("$POPF" "$OUT/pddl/link-classical.pddl" "$OUT/p.pddl" 2>&1 | sed -n '/Solution Found/,$p' \
      | grep -oE '\((drive_dock|drive_station|inspect|report)[^)]*\)' | tr '\n' ' ')
done
