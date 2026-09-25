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
# The pass-through problem, grounded and solved without a simulator.
#
#     tools/validate.sh [out directory, default /tmp/pass_through_validate]
#
# plank grounds the EPDDL, the planner the executor uses solves it, and the
# warehouse scenario's show_plan.py traces the model through every leaf of the
# policy -- product update by product update, with the check that every action
# the planner used applies and the goal holds at every leaf. It prints the
# knowledge state after each step, which is where the point of the domain is
# visible: after the second survey the two scouts know t2 is open only
# distributedly, and after the second exchange the carrier knows it outright.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EPDDL="${HERE}/../epddl"
OUT="${1:-/tmp/pass_through_validate}"
PLANK="${PLANK:-$(command -v plank || echo "${HOME}/plank/build/plank")}"
PLANNER="${EPISTEMIC_PLANNER:-${HOME}/eplansys_ws/install/aletheia/bin/epistemic_planner}"
LIB="${PLANK_LIB:-${HOME}/plank/benchmarks/libraries/intermediate.epddl}"
SHOW="${SHOW_PLAN:-${HERE}/../../../../scenarios/warehouse/tools/show_plan.py}"

mkdir -p "${OUT}"
"${PLANK}" export -d "${EPDDL}/pass-through.epddl" -p "${EPDDL}/pass-through-problem.epddl" \
  -l "${LIB}" -o "${OUT}"
"${PLANNER}" --task "${OUT}/pass-through-problem.json" --plan "${OUT}/plan.json" \
  --timeout 120 --explain 2>&1 | tee "${OUT}/search.log"
python3 "${SHOW}" --task "${OUT}/pass-through-problem.json" --plan "${OUT}/plan.json" \
  --title "pass-through: three bays, two scouts, one carrier"
