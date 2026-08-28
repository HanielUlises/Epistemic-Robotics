#!/usr/bin/env bash
# =============================================================================
# The warehouse domain, end to end, on the floor of the AWS RoboMaker small
# warehouse -- the world the RoboticsAcademy multi-robot Amazon warehouse
# exercise runs on.
#
#   1. ONE AGENT    r1 has to deliver the pallet and know which bay it was in
#   2. TWO AGENTS   r2 has to know, and never leaves receiving to find out
#   3. REJECTED     two plans that must not validate, and why
#
# plank parses, type checks, grounds and validates; the planner searches. The
# reader beside them prints what those two wrote, and follows the model through
# the plan -- which needs the product update, so that much it computes. It
# computes it against the planner: every action must be applicable and the goal
# must hold at every leaf, or the trace reports the disagreement and fails.
#
# PACE=0.4 slows the trace to something a screen recording can follow.
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
SHOW="${HERE}/../../scenarios/warehouse/tools/show_plan.py"

# Seconds between steps of the trace. Zero prints the whole showcase in about
# half a second, which is what you want at a terminal and useless in front of a
# camera; PACE=0.4 turns it into something a viewer can follow.
PACE="${PACE:-0}"

DOMAIN=warehouse-domain.epddl
OUT=out
SCALE="${OUT}/scaling.tsv"

mkdir -p "${OUT}"
: > "${SCALE}"

# Ground one instance, solve it, and read the result out.
#
#   $1  problem path      $2  stem for the generated files      $3  title
solve() {
  local problem="$1" stem="$2" title="$3"

  echo
  echo "############################################################"
  echo "# ${title}"
  echo "############################################################"

  # plank type checks the EPDDL against the domain and the action-type library
  # and writes the ground task: atoms, the initial pointed model, every ground
  # action with its events, and the goal.
  "${PLANK}" export -d "${DOMAIN}" -p "${problem}" -l "${LIB}" -o "${OUT}"

  # The solution branches, so the search has to be AO*: after an inspection
  # there are two situations, and one sequence cannot serve both.
  local started elapsed
  started=$(date +%s.%N)
  # The planner reports its search on stderr, so that is the stream the
  # scaling row is read from; 2>&1 keeps it on screen as well.
  "${PLANNER}" \
    --task "${OUT}/${stem}.json" \
    --plan "${OUT}/${stem}-plan.json" \
    --conditional --timeout 300 2>&1 | tee "${OUT}/${stem}-search.log"
  elapsed=$(python3 -c "print(f'{$(date +%s.%N) - ${started}:.2f}')")

  # One row of the scaling table, taken from what the two tools just said
  # rather than from anything this script decides.
  python3 - "${OUT}/${stem}.json" "${OUT}/${stem}-search.log" \
    "${elapsed}" >> "${SCALE}" <<'PYEOF'
import json, re, sys
info = json.load(open(sys.argv[1]))['planning-task-info']
log = open(sys.argv[2]).read()
found = re.search(r'depth (\d+)\s+Expanded=(\d+)\s+Generated=(\d+)', log)
depth, expanded, generated = found.groups() if found else ('?', '?', '?')
print('\t'.join([str(info.get('agents-number')), str(info.get('atoms-number')),
                 str(info.get('actions-number')),
                 str(info.get('initial-worlds-number')),
                 depth, expanded, generated, sys.argv[3]]))
PYEOF

  python3 "${SHOW}" \
    --task "${OUT}/${stem}.json" \
    --plan "${OUT}/${stem}-plan.json" \
    --pace "${PACE}" \
    --title "${title}"
}

# -- 1 -----------------------------------------------------------------------
#
# One robot, and the knowing is its own. r1 cannot deliver the pallet without
# finding it, and cannot satisfy the modal conjunct without having looked.

solve instances/problem_1.epddl problem_1 \
  "ONE AGENT: r1 delivers the pallet and has to know which bay it came from"

# -- 2 -----------------------------------------------------------------------
#
# Two robots, and the knowing is somebody else's. r2 waits at receiving the
# whole time; the goal still asks that r2 know which bay the pallet was in.
# Watch what the plan does *not* contain.

solve warehouse-problem.epddl warehouse-problem \
  "TWO AGENTS: r2 must know, and never leaves receiving to find out"

# -- 3 -----------------------------------------------------------------------
#
# Three, and the knowing is two other robots'. Nothing about the mission
# changed; the model the planner searches did.

solve instances/problem_3.epddl problem_3 \
  "THREE AGENTS: r2 and r3 must both know, and neither goes to look"

echo
echo "  Note what the two-agent plan does not contain: an announcement."
echo "  pickup is public, so the fleet seeing r1 lift the pallet out of a bay"
echo "  is already enough for r2 to know which bay it was. The domain offers"
echo "  report-pallet-at and the planner declines to spend an action on it."

# -- 3 -----------------------------------------------------------------------

echo
echo "############################################################"
echo "# REJECTED: a sequence, where a policy is needed"
echo "############################################################"
# Everything here is right except its shape. After the inspection the model
# still designates two worlds, and picking up in bay2 is applicable in only
# one of them: a plan that does it anyway is a robot reaching for a pallet
# that is somewhere else half the time.
"${PLANK}" validate -d "${DOMAIN}" -p warehouse-problem.epddl -l "${LIB}" -a \
  "go_r1_dock_south_lane" \
  "go_r1_lane_bay2" \
  "inspect_r1_bay2" \
  "pickup_r1_bay2" \
  "go_r1_bay2_lane" \
  "go_r1_lane_dock_south" \
  "go_r1_dock_south_corridor" \
  "go_r1_corridor_dock_north" \
  "unload_r1_dock_north"

echo
echo "############################################################"
echo "# REJECTED: picking up without looking"
echo "############################################################"
# The modal precondition of pickup: a robot may lift the pallet only where it
# knows the pallet is. Drop that conjunct from the domain and this plan
# becomes valid, which is exactly the classical domain this one is not.
"${PLANK}" validate -d "${DOMAIN}" -p warehouse-problem.epddl -l "${LIB}" -a \
  "go_r1_dock_south_lane" \
  "go_r1_lane_bay2" \
  "pickup_r1_bay2"

echo
echo "############################################################"
echo "# WHAT IT COSTS TO ADD A ROBOT"
echo "############################################################"
echo
python3 - "${SCALE}" <<'PYEOF'
import sys
rows = [l.split('\t') for l in open(sys.argv[1]).read().splitlines() if l.strip()]
head = ('agents', 'atoms', 'ground actions', 'worlds', 'depth',
        'expanded', 'generated', 'seconds')
width = [max(len(head[i]), *(len(r[i]) for r in rows)) for i in range(len(head))]
line = '  ' + '  '.join(h.rjust(width[i]) for i, h in enumerate(head))
print(line)
print('  ' + '  '.join('-' * w for w in width))
for r in rows:
    print('  ' + '  '.join(c.rjust(width[i]) for i, c in enumerate(r)))
if len(rows) > 1:
    def grow(i):
        try:
            return ' -> '.join(f'x{int(rows[k+1][i])/int(rows[k][i]):.0f}'
                               for k in range(len(rows) - 1))
        except (ValueError, ZeroDivisionError):
            return '?'
    print()
    print('  Nodes expanded grows ' + grow(5) + ' as each robot is added.')
PYEOF
echo
echo "  The mission does not get harder when a robot is added -- the plan is"
echo "  the same shape and r1 still does all the walking. The search does."
echo "  Every agent is another relation over the worlds, and a formula about"
echo "  what somebody knows has to be checked against all of them."
echo
echo "Both rejections are the expected result. The plans that are accepted are"
echo "the conditional ones, in ${OUT}/*-plan.json."
