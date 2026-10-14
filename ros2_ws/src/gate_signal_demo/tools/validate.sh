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
# The gate signal, checked without a simulator.
#
#     validate.sh
#
#   1. from its station r2 sees r1 in either gate and nothing of the back
#      routes, the bays' insides, the south corridor or r1's start
#      (check_views.py);
#   2. the EPDDL domain grounds with plank, and Aletheia's policy routes r1
#      through its gate on both branches and sends r2 to the other bay;
#   3. with r2 also able to look into bay X (gate-signal-look.epddl), the
#      policy Aletheia returns checks the bay and never uses a gate;
#   4. POPF plans r1's classical problem by the back route.
set -e
HERE=$(dirname "$(realpath "$0")")
WS=$(realpath "$HERE/../../../..")
EP=$WS/epddl-workspace/gate-signal
LIB=${PLANK_LIBRARY:-$HOME/eplansys_ws/install/plansys2_epddl_grounder/share/plansys2_epddl_grounder/libraries/intermediate.epddl}
PLANK=${PLANK:-$HOME/plank/build/plank}
ALETHEIA=${ALETHEIA:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "1. views"
python3 "$HERE/check_views.py"

plan() {  # domain out
  mkdir -p "$2"
  "$PLANK" export -d "$1" -p "$EP/instances/problem_1.epddl" -l "$LIB" "$EP/gate.epddl" -o "$2" > /dev/null
  "$ALETHEIA" --task "$2/problem_1.json" --plan "$2/policy.json" --timeout 300 2>&1 |
    grep -E "Solution found|validator"
}

echo "2. the policy"
plan "$EP/gate-signal.epddl" "$TMP/main"
python3 - "$TMP/main/policy.json" <<'PY'
import json, sys
t = json.load(open(sys.argv[1]))
assert t['action'] == 'read-order_r1', t['action']
for branch, other in zip(t['branches'], ('y', 'x')):
    seq, node = [], branch['subtree']
    while node:
        seq.append(node['action'])
        node = node['branches'][0]['subtree']
    assert seq[:2] == ['to-gate_r1', 'watch_r2_r1'], seq
    assert 'back-route_r1' not in seq and f'go-{other}_r2' in seq and f'drop-other-{other}_r2' in seq, seq
print('both branches: r1 through its gate, r2 watching, then to the other bay')
PY

echo "3. with a look into bay X"
plan "$EP/gate-signal-look.epddl" "$TMP/look"
python3 - "$TMP/look/policy.json" <<'PY'
import json, sys
def actions(n):
    return [] if not n else [n['action']] + [a for b in n['branches'] for a in actions(b['subtree'])]
acts = actions(json.load(open(sys.argv[1])))
assert 'look-x_r2_r1' in acts and not any(a.startswith(('to-gate', 'watch')) for a in acts), acts
print('the policy checks bay X and uses no gate')
PY

echo "4. r1's classical problem"
mkdir -p "$TMP/pddl"
python3 "$HERE/make_pddl.py" --out "$TMP/pddl"
python3 -c "import sys; sys.path.insert(0, '$HERE'); import make_pddl as P; open('$TMP/r1.pddl', 'w').write(P.problem_r1('x'))"
(cd "$TMP" && "$POPF" "$TMP/pddl/gate-signal-classical.pddl" "$TMP/r1.pddl" 2>&1) | grep -E "^[0-9.]+:" | tee "$TMP/r1.txt"
grep -q "(back_route r1 bay_x)" "$TMP/r1.txt" && ! grep -q to_gate "$TMP/r1.txt" &&
  echo "r1 alone takes the back route"
echo "all checks passed"
