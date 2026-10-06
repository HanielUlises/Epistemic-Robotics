#!/usr/bin/env bash
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
# The coordinated attack, grounded, solved and traced without a simulator.
#
#     tools/validate.sh [out directory, default /tmp/coordinated_attack_validate]
#
#   1. BEACON   the floor has a beacon. The planner must return a policy, and
#               every leaf of it must lift with the stand common knowledge.
#   2. RADIO    the floor has none. The planner must exhaust its space and
#               return no policy.
#   3. LADDER   the four radio messages applied by hand: one level of mutual
#               knowledge per delivered message, E^1 to E^4, and lift still
#               does not apply.
#   4. ALONE    the beacon lit with only south at a viewpoint: south knows, north
#               does not, and lift does not apply.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EPDDL="${HERE}/../epddl"
OUT="${1:-/tmp/coordinated_attack_validate}"
TRACE="${HERE}/trace.py"

PLANK="${PLANK:-$(command -v plank || echo "${HOME}/plank/build/plank")}"
PLANNER="${EPISTEMIC_PLANNER:-${HOME}/eplansys_ws/install/aletheia/bin/epistemic_planner}"
LIB="${PLANK_LIB:-${HOME}/plank/benchmarks/libraries/intermediate.epddl}"

section() {
  echo
  echo "############################################################"
  echo "# $1"
  echo "############################################################"
}

ground() {
  "${PLANK}" export -d "${EPDDL}/coordinated-attack.epddl" -p "${EPDDL}/$1.epddl" \
    -l "${LIB}" "${EPDDL}/lossy.epddl" -o "${OUT}/$1" > "${OUT}/$1.plank.log" 2>&1 \
    || { cat "${OUT}/$1.plank.log"; exit 1; }
}

solve() {
  "${PLANNER}" --task "${OUT}/$1/$1.json" --plan "${OUT}/$1/plan.json" \
    --timeout 240 2>&1 | tee "${OUT}/$1/search.log" | grep -E 'aostar|validator|No solution'
}

mkdir -p "${OUT}"
ground beacon
ground radio

section "1. BEACON: a policy, and common knowledge at every lift"
solve beacon
grep -q 'Solution found' ${OUT}/beacon/search.log
python3 "${TRACE}" --task ${OUT}/beacon/beacon.json --plan ${OUT}/beacon/plan.json \
  | tee ${OUT}/beacon/trace.log
grep -q 'goal holds' ${OUT}/beacon/trace.log

section "2. RADIO: no policy"
solve radio
grep -q 'exhausted' ${OUT}/radio/search.log
test "$(cat ${OUT}/radio/plan.json)" = "null"

section "3. LADDER: four delivered messages, four levels, no lift"
python3 "${TRACE}" --task ${OUT}/radio/radio.json --actions \
  read-order_south_s1 tell_south_north_s1 ack_north_south_s1 \
  ack2_south_north_s1 ack3_north_south_s1 lift_s1 | tee ${OUT}/radio/ladder.log
for k in 1 2 3 4; do grep -q "depth E\^${k}\$" ${OUT}/radio/ladder.log; done
grep -q 'lift_s1  NOT APPLICABLE' ${OUT}/radio/ladder.log

section "4. ALONE: the beacon lit with one robot watching"
python3 "${TRACE}" --task ${OUT}/beacon/beacon.json --actions \
  read-order_south_s1 go-view_south signal_south_s1 lift_s1 | tee ${OUT}/beacon/alone.log
grep -q 'lift_s1  NOT APPLICABLE' ${OUT}/beacon/alone.log

echo
echo "all four checks passed"
