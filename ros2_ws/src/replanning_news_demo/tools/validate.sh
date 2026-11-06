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
#
# Everything about replanning on news that needs no simulator.
#
#   1. the views: tools/check_views.py, in all eight states of the doors;
#   2. the problems: tools/model.py writes each act's model before and after
#      its news, plank grounds them, Aletheia's AO* plans each;
#   3. the policies: tools/policy_check.py checks each against the domain's
#      semantics, the policy before the news on the model the news leaves
#      among them, and whether the news changes the valuations;
#   4. the classical plan POPF returns from the station.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/replanning-news")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/watcher_overlay_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
OUT=${OUT:-$(mktemp -d)}

echo "== 1. the views"
python3 "$HERE/check_views.py" | tail -1

echo "== 2. the problems"
python3 "$HERE/model.py" --out "$EPDDL/instances"
for p in act1 act1-news act2 act2-news act3 act3-news; do
  "$PLANK" export -d "$EPDDL/replanning-news.epddl" -p "$EPDDL/instances/$p.epddl" -l "$LIB" -o "$OUT/$p" > /dev/null
  "$PLANNER" --task "$OUT/$p/$p.json" --plan "$OUT/$p/plan.json" --conditional --consistent-beliefs \
      > "$OUT/$p/aletheia.log" 2>&1
  echo "  $p: $(grep -E 'Solution found' "$OUT/$p/aletheia.log" | head -1)"
done

echo "== 3. the policies"
python3 "$HERE/policy_check.py" "$OUT" | sed 's/^/  /'

echo "== 4. POPF"
python3 -c "import sys; sys.path.insert(0, '$HERE'); import make_pddl as P; open('$OUT/p.pddl', 'w').write(P.problem_classical('st', ['w']))"
python3 "$HERE/make_pddl.py" --out "$OUT/pddl"
echo "  from the station, the west door assumed open:" $("$POPF" "$OUT/pddl/news-classical.pddl" "$OUT/p.pddl" 2>&1 \
    | sed -n '/Solution Found/,$p' | grep -oE '\((goto|cross|deliver)[a-z_]* c\)' | tr '\n' ' ')
echo "  written to $OUT"
