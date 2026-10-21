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
# limitations under the License.
#
# Everything about the collapsed wing that needs no simulator.
#
#   1. the views: tools/check_views.py, in all seven worlds;
#   2. the policy: plank grounds the task, Aletheia's AO* plans it, and in
#      every world the policy sends both scouts out first, has the medic cross
#      only a way it was told is clear, and in the world where only the middle
#      passage is clear announces it by elimination;
#   3. the classical plans POPF returns, from the start and after rubble.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/collapsed-wing")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
POPF=${POPF:-/opt/ros/humble/lib/popf/popf}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

echo "== 1. the views"
python3 "$HERE/check_views.py" | tail -1

echo "== 2. the policy"
"$PLANK" export -d "$EPDDL/collapsed-wing.epddl" -p "$EPDDL/instances/problem_1.epddl" -l "$LIB" -o "$OUT" > /dev/null
"$PLANNER" --task "$OUT/problem_1.json" --plan "$OUT/plan.json" --conditional --consistent-beliefs 2>&1 \
    | grep -E 'Solution found|validator' | sed 's/^/  /'
python3 - "$OUT/problem_1.json" "$OUT/plan.json" "$HERE" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[3])
import worlds as W
task, plan = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
events = {a: v['designated'] for a, v in task['actions'].items()}

def walk(n, d=0):
    if n is None:
        return
    print('    ' + '  ' * d + n['action'])
    for b in n['branches']:
        walk(b['subtree'], d + (len(n['branches']) > 1))
walk(plan)

def run(world):
    """The branch the policy takes in a world, as (action, event) pairs."""
    out, n = [], plan
    while n is not None:
        evs = events[n['action']]
        pick = None
        for b in n['branches']:
            e = evs[b['event']]
            for d, clear in world.items():
                if f'-{d}-' in e + '-' and (e.endswith('clear') != clear):
                    break
            else:
                pick = b
                break
        pick = pick or n['branches'][0]
        out.append((n['action'], evs[pick['event']]))
        n = pick['subtree']
    return out

ok = True
first = [a for a, _ in run(W.WORLDS[0])[:2]]
if sorted(first) != ['goto-e_s1', 'goto-w_s2']:
    print(f'  FAIL the policy does not send both scouts out first: {first}')
    ok = False
for world in W.WORLDS:
    tag = ''.join(d.upper() if c else d for d, c in world.items())
    steps = run(world)
    told = {a.split('_')[0][5:] for a, e in steps if a.startswith('tell-') and e.endswith('clear')}
    crossed = [a.split('_')[0][6:] for a, _ in steps if a.startswith('cross-')]
    looked = {a.split('_')[0][8:] for a, _ in steps if a.startswith('inspect-')}
    good = len(crossed) == 1 and crossed[0] in told and world[crossed[0]]
    print(f'  {"ok  " if good else "FAIL"} {tag}: looked at {sorted(looked)}, told clear {sorted(told)}, '
          f'medic crosses {crossed}')
    ok &= good
    if tag == 'wMe' and ('m' in looked or crossed != ['m']):
        print('  FAIL the middle passage is not crossed by elimination')
        ok = False
sys.exit(0 if ok else 1)
PY

echo "== 3. POPF"
python3 "$HERE/make_pddl.py" --out "$OUT/pddl" > /dev/null
cd "$HERE"
popf() {
  python3 -c "import make_pddl as P; open('$OUT/p.pddl', 'w').write(P.$2)"
  echo "  $1:" $("$POPF" "$OUT/pddl/wing-$3.pddl" "$OUT/p.pddl" 2>&1 | sed -n '/Solution Found/,$p' \
      | grep -oE '\((goto|check|cross|retreat)_[a-z]+ [a-z0-9]+\)' | tr '\n' ' ' || true)
}
popf "optimistic, from the start          " "problem_start(False)" optimistic
popf "optimistic, the west doorway blocked" "problem(['(in_way_w m)', '(assumed_clear_m)', '(assumed_clear_e)'])" optimistic
popf "gated, from the start               " "problem_start(True)" gated
popf "gated, the east doorway checked     " "problem(['(at_hall m)', '(at_vantage_e s1)', '(at_entry_w s2)', '(unchecked_w)'])" gated
popf "gated, both doorways checked        " "problem(['(at_hall m)', '(at_vantage_e s1)', '(at_vantage_w s2)'])" gated
