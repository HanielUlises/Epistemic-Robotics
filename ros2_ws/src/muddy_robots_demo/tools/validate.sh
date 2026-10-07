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
# The muddy robots, grounded, solved and traced without a simulator.
#
#     tools/validate.sh [out directory, default /tmp/muddy_robots_validate]
#
# Four robots, the floors tools/muster.py writes:
#
#   1. PA       the hall has a public address. The planner must return a
#               policy, and every leaf of it must satisfy the goal at each of
#               its designated worlds: four leaves, the fifteen patterns with
#               a lamp lit.
#   2. SILENT   it has none. The planner must return no policy: the
#               relaxation proves the goal unreachable.
#   3. THREE    r1, r2 and r3 lit, with the announcement. "Some lamp is lit" is
#               E^2 before it and C after; the bell gives e-stay, e-stay,
#               e-leave; the worlds reachable go 16, 15, 11, 5, 1; and at the
#               end the three know they are faulty and r4 that it is not.
#   4. MUTE     the same lamps on the silent floor. The announcement does not
#               apply, every bell gives e-stay, the depth stays at E^2 and the
#               worlds at 16, and no robot ever learns its own lamp.
#   5. ONE      r2 alone lit. r2 knows from the announcement, and the first
#               bell gives e-leave.
#   6. DEPTH    for two to six robots: a policy of depth N with the public
#               address, and none without.
#
# And the floor the domain assumes, on the AWS small warehouse:
#
#   7. FLOOR    for three, four and five robots, every robot on the muster
#               ring sees every other, and every place a robot is sent is
#               reachable.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-/tmp/muddy_robots_validate}"
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

# Writes and grounds both floors for n robots: ground <robots>.
ground() {
  local d="${OUT}/n$1"
  python3 "${HERE}/muster.py" --robots "$1" --out "$d" > /dev/null
  for floor in pa silent; do
    "${PLANK}" export -d "$d/muddy-robots.epddl" -p "$d/${floor}-n$1.epddl" \
      -l "${LIB}" -o "$d/${floor}" > "$d/${floor}.plank.log" 2>&1 \
      || { cat "$d/${floor}.plank.log"; exit 1; }
  done
}

# Solves one floor: solve <robots> <floor>. The search log is kept beside it.
solve() {
  local d="${OUT}/n$1/$2"
  "${PLANNER}" --task "$d/$2-n$1.json" --plan "$d/plan.json" --timeout 240 \
    --strategy aostar --no-portfolio 2>&1 | tee "$d/search.log" \
    | grep -E 'Solution found|No solution' || true
}

# The sequence of one kind of token in a trace: tokens <log> <pattern>.
tokens() {
  grep -oE "$2" "$1" | tr '\n' ' ' | sed 's/ $//'
}

# Fails with the trace shown when a sequence is not the one expected:
# expect <what> <actual> <expected> <log>.
expect() {
  if [ "$2" != "$3" ]; then
    echo "FAILED: $1 is '$2', expected '$3'"
    cat "$4"
    exit 1
  fi
  echo "  $1: $2"
}

mkdir -p "${OUT}"
ground 4

section "1. PA: a policy, and the goal at every leaf"
solve 4 pa
grep -q 'Solution found at depth 4' "${OUT}/n4/pa/search.log"
python3 "${TRACE}" --task "${OUT}/n4/pa/pa-n4.json" --plan "${OUT}/n4/pa/plan.json" \
  | tee "${OUT}/pa-policy.log" | tail -1
test "$(grep -c 'goal holds' "${OUT}/pa-policy.log")" -eq 4
grep -q '4 leaves, 15 fault patterns covered' "${OUT}/pa-policy.log"

section "2. SILENT: no policy without the public address"
solve 4 silent
grep -q 'the relaxation proves the goal unreachable' "${OUT}/n4/silent/search.log"
# Not `! grep`: set -e never exits on a negated command.
if grep -q 'Solution found' "${OUT}/n4/silent/search.log"; then
  echo 'FAILED: a policy on the silent floor'; exit 1
fi

section "3. THREE: r1, r2 and r3 lit, with the announcement"
python3 "${TRACE}" --task "${OUT}/n4/pa/pa-n4.json" --faults r1 r2 r3 --bells 4 \
  > "${OUT}/three.log"
L="${OUT}/three.log"
expect 'depth of "some lamp lit"' "$(tokens "$L" '"some lamp lit": (E\^[0-9]+|C)' | sed 's/"some lamp lit": //g')" \
  'E^2 C C C C' "$L"
expect 'bells' "$(tokens "$L" 'e-(stay|leave)')" 'e-stay e-stay e-leave' "$L"
expect 'worlds reachable' "$(tokens "$L" '[0-9]+ worlds reachable' | sed 's/ worlds reachable//g')" \
  '16 15 11 5 1' "$L"
tail -2 "$L" | grep -qE 'r1 faulty +r2 faulty +r3 faulty +r4 clean' \
  || { echo 'FAILED: at the end the robots do not all know'; cat "$L"; exit 1; }
echo '  at the end: r1, r2, r3 faulty and r4 clean, each known to itself'

section "4. MUTE: the same lamps, and no public address"
python3 "${TRACE}" --task "${OUT}/n4/silent/silent-n4.json" --faults r1 r2 r3 --bells 4 \
  > "${OUT}/mute.log"
L="${OUT}/mute.log"
grep -q 'announce  NOT APPLICABLE' "$L"
expect 'depth of "some lamp lit"' "$(tokens "$L" '"some lamp lit": (E\^[0-9]+|C)' | sed 's/"some lamp lit": //g')" \
  'E^2 E^2 E^2 E^2 E^2' "$L"
expect 'bells' "$(tokens "$L" 'e-(stay|leave)')" 'e-stay e-stay e-stay e-stay' "$L"
expect 'worlds reachable' "$(tokens "$L" '[0-9]+ worlds reachable' | sed 's/ worlds reachable//g')" \
  '16 16 16 16 16' "$L"
! grep -qE ' (faulty|clean) ' "$L" \
  || { echo 'FAILED: a robot learnt its own lamp without the announcement'; cat "$L"; exit 1; }
echo '  no robot ever learns its own lamp'

section "5. ONE: r2 alone lit"
python3 "${TRACE}" --task "${OUT}/n4/pa/pa-n4.json" --faults r2 --bells 4 > "${OUT}/one.log"
L="${OUT}/one.log"
expect 'bells' "$(tokens "$L" 'e-(stay|leave)')" 'e-leave' "$L"
grep -A1 '^  announce' "$L" | grep -qE 'r2 faulty' \
  || { echo 'FAILED: r2 does not know from the announcement'; cat "$L"; exit 1; }
echo '  r2 knows from the announcement, and leaves at the first bell'

section "6. DEPTH: a policy of depth N with the public address, none without"
for n in 2 3 4 5 6; do
  [ "$n" -eq 4 ] || ground "$n"
  solve "$n" pa > /dev/null
  solve "$n" silent > /dev/null
  grep -q "Solution found at depth $n " "${OUT}/n$n/pa/search.log" \
    || { echo "FAILED: no policy of depth $n for $n robots"; cat "${OUT}/n$n/pa/search.log"; exit 1; }
  grep -q 'the relaxation proves the goal unreachable' "${OUT}/n$n/silent/search.log" \
    || { echo "FAILED: the silent floor for $n robots"; cat "${OUT}/n$n/silent/search.log"; exit 1; }
  echo "  $n robots: $(grep -oE 'Solution found at depth [0-9]+ +Expanded=[0-9]+' "${OUT}/n$n/pa/search.log"); silent: no policy"
done

section "7. FLOOR: the muster ring and the routes, for three to five robots"
for n in 3 4 5; do
  python3 "${HERE}/check_floor.py" --robots "$n" | tail -1
done

echo
echo "all seven checks passed"
