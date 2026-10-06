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
# The false-belief domain, grounded, solved and traced without a simulator.
#
#     tools/validate.sh [out directory, default /tmp/false_belief_validate]
#
#   1. FALSE     the shift's two actions: the crate is in t3, the picker
#                believes it in t1, the mover believes the picker believes
#                so, and the picker's relation is no longer reflexive.
#   2. TOLD      a policy, with the report before the fetch; every leaf
#                fetches with every robot's belief consistent.
#   3. UNTOLD    no policy when consistent beliefs are required.
#   4. COLLAPSE  on the untold floor, the picker looking into t1: it finds it
#                empty and is left with no world, believing everything.
#   5. VACUOUS   the same floor with consistent beliefs not required: the
#                planner returns a policy, and the trace shows it relies on
#                that collapse.
#   6. DOUBT     a policy that looks before it fetches; after the relocation
#                the picker believes the crate in neither bay, and its
#                relation stays reflexive.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EPDDL="${HERE}/../epddl"
OUT="${1:-/tmp/false_belief_validate}"
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
  "${PLANK}" export -d "${EPDDL}/false-belief.epddl" -p "${EPDDL}/$1.epddl" \
    -l "${LIB}" "${EPDDL}/beliefs.epddl" -o "${OUT}/$1" \
    > "${OUT}/$1.plank.log" 2>&1 || { cat "${OUT}/$1.plank.log"; exit 1; }
}

# solve <floor> <plan file> [planner flags]
solve() {
  local floor="$1" plan="$2"
  shift 2
  "${PLANNER}" --task "${OUT}/${floor}/${floor}.json" --plan "${OUT}/${floor}/${plan}" \
    --timeout 240 "$@" 2>&1 | tee "${OUT}/${floor}/${plan%.json}.log" \
    | grep -E 'aostar|validator|No solution'
}

mkdir -p "${OUT}"
ground told
ground untold
ground doubt

section "1. FALSE: the shift's two actions plant a false belief"
python3 "${TRACE}" --task "${OUT}/untold/untold.json" \
  --actions go-dock_picker relocate_mover_t1_t3 | tee "${OUT}/false.log"
grep -q 'crate t3   B_picker t1   B_mover t3   B_mover B_picker t1' "${OUT}/false.log" \
  || { echo "FAIL: expected the picker to believe t1 and the mover to know it"; exit 1; }
grep -q 'picker not reflexive' "${OUT}/false.log" \
  || { echo "FAIL: expected the picker's relation to lose reflexivity"; exit 1; }

section "2. TOLD: a policy, the report before the fetch, beliefs consistent"
solve told plan.json --consistent-beliefs
python3 "${TRACE}" --task "${OUT}/told/told.json" --plan "${OUT}/told/plan.json" \
  | tee "${OUT}/told.trace"
grep -q 'report_mover_picker_t1_t3' "${OUT}/told.trace" \
  || { echo "FAIL: expected the policy to report the move"; exit 1; }

section "3. UNTOLD: no policy with consistent beliefs"
if solve untold plan.json --consistent-beliefs | grep -q 'Solution found'; then
  echo "FAIL: expected no policy on the untold floor"; exit 1
fi

section "4. COLLAPSE: looking where the crate is believed to be"
python3 "${TRACE}" --task "${OUT}/untold/untold.json" \
  --actions go-dock_picker relocate_mover_t1_t3 look_picker_t1:e-empty | tee "${OUT}/collapse.log"
grep -q 'picker has no world' "${OUT}/collapse.log" \
  || { echo "FAIL: expected the picker to be left with no world"; exit 1; }

section "5. VACUOUS: the untold floor without consistent beliefs"
solve untold vacuous.json | grep -q 'Solution found' \
  || { echo "FAIL: expected a policy when consistent beliefs are not required"; exit 1; }
if python3 "${TRACE}" --task "${OUT}/untold/untold.json" --plan "${OUT}/untold/vacuous.json" \
    | tee "${OUT}/vacuous.trace"; then
  echo "FAIL: expected the trace to find a collapse in that policy"; exit 1
fi
grep -q 'leaves an agent without a consistent belief' "${OUT}/vacuous.trace"

section "6. DOUBT: a policy that looks first, and no false belief"
solve doubt plan.json --consistent-beliefs
python3 "${TRACE}" --task "${OUT}/doubt/doubt.json" --plan "${OUT}/doubt/plan.json" \
  | tee "${OUT}/doubt.trace"
grep -q 'look_picker_' "${OUT}/doubt.trace" \
  || { echo "FAIL: expected the policy to look before it fetches"; exit 1; }
grep -q 'crate t3   B_picker ?' "${OUT}/doubt.trace" \
  || { echo "FAIL: expected the picker to believe the crate in neither bay"; exit 1; }
grep -A2 'relocate_mover' "${OUT}/doubt.trace" | grep -q 'picker reflexive' \
  || { echo "FAIL: expected the picker's relation to stay reflexive"; exit 1; }

echo
echo "all six checks passed"
