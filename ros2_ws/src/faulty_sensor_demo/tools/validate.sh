#!/bin/bash
# Copyright 2026 Haniel Ulises
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
# Everything about a faulty sensor in the fleet that needs no simulator.
#
#   1. the floor: tools/check_views.py, on the porter's six rounds;
#   2. the problems: tools/model.py writes the two domains and the six
#      problems, plank grounds them; tools/make_pddl.py writes the vocabulary;
#   3. the policies: tools/policy_check.py builds each act's policy under each
#      formulation and replays it through the product update, checking every
#      robot's cell and every world's grade at every node against the
#      reference, and on which nights a bed-mover drives into a shut passage;
#   4. the schedule: the groups tools/plan.py plans on are the groups ePlanSys
#      makes of the policy (tools/schedule.py), on every night;
#   5. Aletheia, unmodified, plans act 1 itself;
#   6. the expectations tools/plan.py predicts.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/faulty-sensor")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/watcher_overlay_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
OUT=${OUT:-$(mktemp -d)}

echo "== 1. the floor"
python3 "$HERE/check_views.py" | tail -2

echo "== 2. the problems"
python3 "$HERE/model.py" --out "$EPDDL"
python3 "$HERE/make_pddl.py" --out "$HERE/../pddl"
for act in 1 2 3; do
  "$PLANK" export -d "$EPDDL/faulty-sensor.epddl" -p "$EPDDL/instances/act$act-sensors.epddl" -l "$LIB" -o "$OUT/task$act-sensors" > /dev/null 2>&1
  "$PLANK" export -d "$EPDDL/faulty-readings.epddl" -p "$EPDDL/instances/act$act-readings.epddl" -l "$LIB" -o "$OUT/task$act-readings" > /dev/null 2>&1
done
python3 - "$OUT" <<'PY'
import json, sys
for kind in ('sensors', 'readings'):
    ts = [json.load(open(f'{sys.argv[1]}/task{a}-{kind}/act{a}-{kind}.json')) for a in (1, 2, 3)]
    print(f'  {kind}: ' + ', '.join(f"{len(t['initial-state']['worlds'])} worlds and {len(t['actions'])} ground actions"
                                    for t in ts))
PY

echo "== 3. the policies"
python3 "$HERE/policy_check.py" --out "$OUT" | sed 's/^/  /'

echo "== 4. the schedule"
python3 - "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
sys.setrecursionlimit(10000)
import plan as P, ranks as R, schedule as S
for act in sorted(R.FLEETS):
    worst = 0.0
    for f in R.FORMULATIONS:
        pl, tree = P.plan(act, f)
        root = S.schedule(tree, f, act)
        for n in R.nights(act):
            worst = max(worst, abs(S.timeline(root, n)[0] - P.timeline(P.trace(tree, n), pl.fleet)[0]))
    print(f'  act {act}: the planner\'s groups and ePlanSys\'s differ by at most {worst:.0f} s on a night')
PY

echo "== 5. Aletheia"
timeout ${ALETHEIA_LIMIT:-600} "$PLANNER" --task "$OUT/task1-sensors/act1-sensors.json" --plan "$OUT/aletheia-act1.json" --conditional \
    > "$OUT/aletheia-act1.log" 2>&1 || true
echo "  act 1: $(grep -E 'Solution found|No solution|exhausted' "$OUT/aletheia-act1.log" | head -1)"

echo "== 6. the expectations"
python3 - "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
sys.setrecursionlimit(10000)
import plan as P, ranks as R
for act in sorted(R.FLEETS):
    print(f'  act {act}')
    for f in R.FORMULATIONS:
        pl, tree = P.plan(act, f)
        s = P.summary(P.report(pl, tree))
        print(f'    {f:10s} ' + '  '.join(f'{k}: {v[0]:.1f} s, a shut passage entered {v[1]:.3f}'
                                          for k, v in s.items() if v[0] is not None))
PY
echo "  written to $OUT"
