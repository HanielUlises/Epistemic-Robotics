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
# The same floor with moves only the mover witnesses (coordinated-attack-
# positions), where who saw the beacon is no longer common knowledge:
#
#   5. SIGHT    the viewpoints see each other. A policy, and every leaf lifts
#               under C; every leaf has the robots sight each other.
#   6. BLIND    they do not. No policy, although the beacon is there.
#   7. UNSEEN   on the sight floor, the signal without a sighting: E^1, as
#               from one radio message, and lift does not apply.
#   8. AFTER    the signal and then the sighting: C. The sighting makes the
#               earlier signal common knowledge.
#
# The domain tools/scaled.py writes for n robots and m message levels:
#
#   9. TWO      at n = 2, m = 4 it agrees with the published floors: a beacon
#               policy of depth 5, and no radio policy.
#  10. FOUR     the beacon floor with four robots, as run in Gazebo: a policy,
#               and every leaf lifts under C over all four.
#  11. THREE    the radio floor with three robots and one level: no policy.
#  12. LADDER3  with three robots, E^3 costs six messages: the planner finds
#               no fewer, and the product update confirms each level and that
#               lift does not apply.
#  13. PROTOCOL protocols/radio-n4-m2.json, which the mission runs on the
#               four-robot radio floor, reaches E^2 and not C.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EPDDL="${HERE}/../epddl"
OUT="${1:-/tmp/coordinated_attack_validate}"
TRACE="${HERE}/trace.py"
export EPISTEMIC_PLANNER PLANK PLANK_LIB

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
  local domain="${2:-coordinated-attack}"
  "${PLANK}" export -d "${EPDDL}/${domain}.epddl" -p "${EPDDL}/$1.epddl" \
    -l "${LIB}" "${EPDDL}/lossy.epddl" "${EPDDL}/moves.epddl" -o "${OUT}/$1" \
    > "${OUT}/$1.plank.log" 2>&1 || { cat "${OUT}/$1.plank.log"; exit 1; }
}

# Writes and grounds the scaled floor: ground_scaled <robots> <levels> <floor>.
ground_scaled() {
  local d="${OUT}/scaled-n$1-m$2" name="$3-n$1-m$2"
  python3 "${HERE}/scaled.py" --robots "$1" --messages "$2" --out "$d" > /dev/null
  "${PLANK}" export -d "$d/coordinated-attack-m$2.epddl" -p "$d/${name}.epddl" \
    -l "${LIB}" "${EPDDL}/lossy.epddl" -o "$d/${name}" \
    > "$d/${name}.plank.log" 2>&1 || { cat "$d/${name}.plank.log"; exit 1; }
}

solve_scaled() {
  local d="${OUT}/scaled-n$1-m$2/$3-n$1-m$2"
  "${PLANNER}" --task "$d/$3-n$1-m$2.json" --plan "$d/plan.json" --timeout 240 \
    --strategy aostar --no-portfolio 2>&1 | tee "$d/search.log" | grep -E 'aostar|validator|No solution'
}

solve() {
  "${PLANNER}" --task "${OUT}/$1/$1.json" --plan "${OUT}/$1/plan.json" \
    --timeout 240 2>&1 | tee "${OUT}/$1/search.log" | grep -E 'aostar|validator|No solution'
}

mkdir -p "${OUT}"
ground beacon
ground radio
ground positions-sight coordinated-attack-positions
ground positions-blind coordinated-attack-positions

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

section "5. SIGHT: a policy, and a sighting on every branch"
solve positions-sight
grep -q 'Solution found' ${OUT}/positions-sight/search.log
python3 "${TRACE}" --task ${OUT}/positions-sight/positions-sight.json \
  --plan ${OUT}/positions-sight/plan.json | tee ${OUT}/positions-sight/trace.log
test "$(grep -c 'goal holds' ${OUT}/positions-sight/trace.log)" -ge 2
test "$(grep -c '^ *sight_' ${OUT}/positions-sight/trace.log)" -ge 2

section "6. BLIND: no policy, beacon or not"
solve positions-blind
grep -q 'exhausted' ${OUT}/positions-blind/search.log
test "$(cat ${OUT}/positions-blind/plan.json)" = "null"

section "7. UNSEEN: the signal without a sighting is one level"
python3 "${TRACE}" --task ${OUT}/positions-sight/positions-sight.json --actions \
  go-view_north read-order_south_s1 go-view_south signal_south_north_s1 lift_s1 \
  | tee ${OUT}/positions-sight/unseen.log
grep -q 'depth E\^1$' ${OUT}/positions-sight/unseen.log
grep -q 'lift_s1  NOT APPLICABLE' ${OUT}/positions-sight/unseen.log

section "8. AFTER: a sighting after the signal makes it common knowledge"
python3 "${TRACE}" --task ${OUT}/positions-sight/positions-sight.json --actions \
  go-view_north read-order_south_s1 go-view_south signal_south_north_s1 \
  sight_south_north lift_s1 | tee ${OUT}/positions-sight/after.log
grep -q 'depth C' ${OUT}/positions-sight/after.log
! grep -q 'NOT APPLICABLE' ${OUT}/positions-sight/after.log

section "9. TWO: the scaled domain at two robots agrees with the published one"
ground_scaled 2 4 beacon
ground_scaled 2 4 radio
solve_scaled 2 4 beacon
grep -q 'Solution found at depth 5' ${OUT}/scaled-n2-m4/beacon-n2-m4/search.log
solve_scaled 2 4 radio
grep -q 'exhausted' ${OUT}/scaled-n2-m4/radio-n2-m4/search.log

section "10. FOUR: four robots, one signal, C over all four"
ground_scaled 4 2 beacon
solve_scaled 4 2 beacon
grep -q 'Solution found' ${OUT}/scaled-n4-m2/beacon-n4-m2/search.log
python3 "${TRACE}" --task ${OUT}/scaled-n4-m2/beacon-n4-m2/beacon-n4-m2.json \
  --plan ${OUT}/scaled-n4-m2/beacon-n4-m2/plan.json | tee ${OUT}/scaled-n4-m2/trace.log
test "$(grep -c 'goal holds' ${OUT}/scaled-n4-m2/trace.log)" -eq 2
grep -q 'depth C' ${OUT}/scaled-n4-m2/trace.log

section "11. THREE: no radio policy for three robots"
ground_scaled 3 1 radio
solve_scaled 3 1 radio
grep -q 'exhausted' ${OUT}/scaled-n3-m1/radio-n3-m1/search.log

section "12. LADDER3: E^3 among three robots costs six messages"
python3 "${HERE}/ladder.py" --robots 3 --depth 3 --out ${OUT}/ladder | tee ${OUT}/ladder.log
grep -q '3 robots, E^3, branch e-here: 6 messages' ${OUT}/ladder.log
! grep -q 'PROBLEM' ${OUT}/ladder.log

section "13. PROTOCOL: the four-robot radio protocol reaches E^2, not C"
python3 "${HERE}/ladder.py" --replay "$(cd "${HERE}/.." && pwd)/protocols/radio-n4-m2.json" \
  --out ${OUT}/replay | tee ${OUT}/replay.log
test "$(grep -c 'lift_s[12]  NOT APPLICABLE' ${OUT}/replay.log)" -eq 2

echo
echo "all thirteen checks passed"
