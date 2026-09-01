#!/usr/bin/env bash
# =============================================================================
# The relay domain, end to end, scaling the fleet from one robot to as many as
# this AO* build finishes in reasonable time.
#
#   1. ONE ROBOT     r1 finds the package and delivers it -- nobody else to
#                    tell, so KD45_n has nothing to diverge from yet.
#   2. TWO ROBOTS    r2 waits at home the whole shift; the only way it learns
#                    which bay is if r1 says so.
#   3. THREE ROBOTS  r2 and r3 both need telling; one broadcast reaches both.
#   4. FOUR ROBOTS   same broadcast, one more listener.
#   5. FIVE ROBOTS   as massive as this build finishes in a demo-length run;
#                    six was tried and left running well past worth waiting on.
#   6. REJECTED      two plans that must not validate, and why.
#
# plank parses, type checks, grounds and validates; the planner searches. The
# scaling table at the end is read from what those two tools said, the same
# way robot-warehouse's validate.sh builds its own.
#
# Requires plank (https://github.com/HanielUlises/plank) and the epistemic
# planner. Both are found on PATH, or through PLANK and EPISTEMIC_PLANNER.
# =============================================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}"

PLANK="${PLANK:-$(command -v plank || echo "${HOME}/plank/build/plank")}"
PLANNER="${EPISTEMIC_PLANNER:-$(command -v epistemic_planner || echo "${HOME}/Projects/team-3/build/epistemic_planner")}"
LIB="${PLANK_LIB:-${HOME}/plank/benchmarks/libraries/intermediate.epddl}"

DOMAIN=robot-relay-domain.epddl
OUT=out
SCALE="${OUT}/scaling.tsv"

mkdir -p "${OUT}"
: > "${SCALE}"

# Ground one instance, solve it, and read the result out.
#
#   $1  problem path      $2  stem for the generated files      $3  title
#   $4  timeout in seconds (planner search only)
solve() {
  local problem="$1" stem="$2" title="$3" tmo="${4:-300}"

  echo
  echo "############################################################"
  echo "# ${title}"
  echo "############################################################"

  "${PLANK}" export -d "${DOMAIN}" -p "${problem}" -l "${LIB}" -o "${OUT}"

  local started elapsed
  started=$(date +%s.%N)
  # The planner reports its search on stderr; 2>&1 keeps it on screen and in
  # the log the scaling row below is read from. timeout bounds the wall clock
  # -- the planner itself takes a node/depth limit, not a time one.
  timeout "${tmo}" "${PLANNER}" \
    --task "${OUT}/${stem}.json" \
    --plan "${OUT}/${stem}-plan.json" \
    --conditional 2>&1 | tee "${OUT}/${stem}-search.log" || true
  elapsed=$(python3 -c "print(f'{$(date +%s.%N) - ${started}:.2f}')")

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
}

# -- 1 -------------------------------------------------------------------
solve instances/problem_1.epddl problem_1 \
  "ONE ROBOT: r1 finds the package and delivers it, alone"

# -- 2 -------------------------------------------------------------------
solve instances/problem_2.epddl problem_2 \
  "TWO ROBOTS: r2 waits at home; only r1 telling it settles which bay"

# -- 3 -------------------------------------------------------------------
solve instances/problem_3.epddl problem_3 \
  "THREE ROBOTS: r2 and r3 both need telling; one broadcast reaches both"

# -- 4 -------------------------------------------------------------------
solve instances/problem_4.epddl problem_4 \
  "FOUR ROBOTS: same broadcast, one more listener"

# -- 5 -------------------------------------------------------------------
#
# The empirical ceiling for this script, not a round number picked in
# advance: five agents solved in under five minutes in testing; six was
# still searching well past that. A wider timeout here is the honest way to
# show it rather than quietly shrinking the instance to fit.
solve instances/problem_5.epddl problem_5 \
  "FIVE ROBOTS: as massive as this AO* build finishes in a demo-length run" \
  600

echo
echo "  Every idle robot needs the same one fact -- which bay -- and every"
echo "  instance answers it with the same one broadcast, however many are"
echo "  listening. What grows with the fleet is not the plan; it is the"
echo "  model the planner has to search to be sure the broadcast was needed"
echo "  and sufficient for all of them at once."

# -- REJECTED -------------------------------------------------------------

echo
echo "############################################################"
echo "# REJECTED: a sequence, where a policy is needed"
echo "############################################################"
# inspect leaves the model designating both bays open; picking up in bayA
# regardless is applicable in only one of them.
"${PLANK}" validate -d "${DOMAIN}" -p instances/problem_1.epddl -l "${LIB}" -a \
  "go_r1_home_lane" \
  "go_r1_lane_bayA" \
  "inspect_r1_bayA" \
  "pickup_r1_bayA"

echo
echo "############################################################"
echo "# REJECTED: picking up without ever sensing"
echo "############################################################"
# The modal precondition of pickup: a robot may lift the package only where
# it *believes* the package is there. Drop that conjunct from the domain and
# this plan becomes valid -- the classical domain this one deliberately is
# not.
"${PLANK}" validate -d "${DOMAIN}" -p instances/problem_1.epddl -l "${LIB}" -a \
  "go_r1_home_lane" \
  "go_r1_lane_bayA" \
  "pickup_r1_bayA"

echo
echo "############################################################"
echo "# WHAT IT COSTS TO ADD AN IDLE ROBOT"
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
            return ' -> '.join(f'x{int(rows[k+1][i])/int(rows[k][i]):.1f}'
                               for k in range(len(rows) - 1))
        except (ValueError, ZeroDivisionError):
            return '?'
    print()
    print('  Nodes expanded grows ' + grow(5) + ' as each idle robot is added.')
PYEOF
echo
echo "  In robot-warehouse an extra robot costs the search a proportionally"
echo "  bigger model but the plan never spends an action on it -- the fleet"
echo "  learns for free from watching a public pickup. Here nothing is free:"
echo "  every idle robot really does hold a false belief until told, so the"
echo "  broadcast is load-bearing at every fleet size -- and checking a modal"
echo "  goal against one more agent's relation over the same worlds is what"
echo "  drives the search cost past linear."
echo
echo "Both rejections are the expected result. The plans that are accepted are"
echo "the conditional ones, in ${OUT}/*-plan.json."
