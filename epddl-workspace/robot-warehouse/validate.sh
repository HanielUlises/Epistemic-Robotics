#!/usr/bin/env bash
# =============================================================================
# The warehouse domain, end to end: parse, ground, solve, and two plans that
# must be rejected.
#
# Requires plank (https://github.com/a-burigana/plank) and the epistemic
# planner. Both are found on PATH, or through PLANK and EPISTEMIC_PLANNER.
# =============================================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}"

PLANK="${PLANK:-$(command -v plank || echo "${HOME}/plank/build/plank")}"
PLANNER="${EPISTEMIC_PLANNER:-$(command -v epistemic_planner || echo "${HOME}/aletheia/team-3/build/epistemic_planner")}"
LIB="${PLANK_LIB:-${HOME}/plank/benchmarks/libraries/intermediate.epddl}"

DOMAIN=warehouse-domain.epddl
PROBLEM=warehouse-problem.epddl
OUT=out

mkdir -p "${OUT}"

echo "============================================================"
echo " PARSE and GROUND"
echo "============================================================"
"${PLANK}" export -d "${DOMAIN}" -p "${PROBLEM}" -l "${LIB}" -o "${OUT}"

echo
echo "============================================================"
echo " SOLVE"
echo "============================================================"
# The solution branches, so the search has to be AO*: after r1 looks into
# bay2 there are two situations, and one plan cannot serve both.
"${PLANNER}" \
  --task "${OUT}/warehouse-problem.json" \
  --plan "${OUT}/warehouse-plan.json" \
  --conditional --timeout 120

echo
echo "============================================================"
echo " REJECT - a sequence, where a policy is needed"
echo "============================================================"
# Everything here is right except its shape. After the inspection the model
# still designates two worlds, and picking up in bay2 is applicable in only
# one of them: a plan that does it anyway is a robot reaching for a pallet
# that is somewhere else half the time.
"${PLANK}" validate -d "${DOMAIN}" -p "${PROBLEM}" -l "${LIB}" -a \
  "go_r1_depot_bay2" \
  "inspect_r1_bay2" \
  "pickup_r1_bay2" \
  "go_r1_bay2_bay3" \
  "go_r1_bay3_dock1" \
  "unload_r1_dock1"

echo
echo "============================================================"
echo " REJECT - picking up without looking"
echo "============================================================"
# The modal precondition of pickup: a robot may lift the pallet only where it
# knows the pallet is. Drop that conjunct from the domain and this plan
# becomes valid, which is exactly the classical domain this one is not.
"${PLANK}" validate -d "${DOMAIN}" -p "${PROBLEM}" -l "${LIB}" -a \
  "go_r1_depot_bay2" \
  "pickup_r1_bay2"

echo
echo "Both rejections are the expected result. The plan that is accepted is"
echo "the conditional one, in ${OUT}/warehouse-plan.json."
