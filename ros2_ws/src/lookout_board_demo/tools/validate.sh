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
# The lookout board, checked without a simulator.
#
#     validate.sh
#
#   1. the hall's geometry gives each observation the meaning the domain gives
#      it, in all eight worlds (check_views.py);
#   2. the EPDDL domain grounds with plank, and Aletheia's policy reads the
#      board, goes to the lookout it shows lit, and goes to lookout 4 unread
#      when slots 1 to 3 are dark;
#   3. no sequence exists: Aletheia's GBFS, which returns sequences, finds none;
#   4. POPF plans the classical problems: the replanner's first plan goes to the
#      nearest lookout and does not read the board.
set -e
HERE=$(dirname "$(realpath "$0")")
WS=$(realpath "$HERE/../../../..")
LIB=${PLANK_LIBRARY:-$HOME/eplansys_ws/install/plansys2_epddl_grounder/share/plansys2_epddl_grounder/libraries/intermediate.epddl}
PLANK=${PLANK:-$HOME/plank/build/plank}
ALETHEIA=${ALETHEIA:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "1. views"
python3 "$HERE/check_views.py"

echo "2. the policy"
python3 "$HERE/make_domain.py" --lookouts 4 --out "$TMP/epddl"
cmp -s "$TMP/epddl/lookout-board.epddl" "$WS/epddl-workspace/lookout-board/lookout-board.epddl" ||
  { echo "epddl-workspace/lookout-board is not what make_domain.py writes"; exit 1; }
"$PLANK" export -d "$TMP/epddl/lookout-board.epddl" -p "$TMP/epddl/instances/problem_1.epddl" \
  -l "$LIB" -o "$TMP" > /dev/null
"$ALETHEIA" --task "$TMP/problem_1.json" --plan "$TMP/policy.json" --timeout 120 2>&1 |
  grep -E "Solution found|validator"
python3 - "$TMP/policy.json" <<'PY'
import json, sys
t = json.load(open(sys.argv[1]))
assert t['action'] == 'goto-board_r1', t['action']
node, k = t['branches'][0]['subtree'], 1
while True:
    assert node['action'] == f'read-{k}_r1', node['action']
    lit, dark = node['branches'][0]['subtree'], node['branches'][1]['subtree']
    assert lit['action'] == f'goto-l{k}_r1', lit['action']
    if k == 3:
        assert dark['action'] == 'goto-l4_r1', dark['action']
        break
    node, k = dark, k + 1
print('the policy reads slots 1 to 3, goes to the lit one, and to lookout 4 unread')
PY

echo "3. no sequence"
"$ALETHEIA" --task "$TMP/problem_1.json" --plan "$TMP/seq.json" --strategy gbfs --timeout 60 2>&1 |
  grep -q "No solution found" && echo "GBFS finds no sequence"

echo "4. the classical problems"
python3 "$HERE/make_pddl.py" --out "$TMP/pddl"
(cd "$TMP" && "$POPF" "$TMP/pddl/lookout-board.pddl" "$TMP/pddl/problem-replan.pddl" 2>&1) |
  grep -E "^[0-9.]+:" | tee "$TMP/replan.txt"
grep -q "(goto r1 start l4)" "$TMP/replan.txt" && ! grep -q read_slot "$TMP/replan.txt" &&
  echo "the replanner goes to the nearest lookout, l4, and does not read the board"
echo "all checks passed"
