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
# The hotel's distributed leak, grounded, solved, traced and searched, without
# a simulator.
#
#     tools/validate.sh [out directory, default /tmp/hotel_distributed_validate]
#
# The hotel's instance, two floors and two columns, tools/hotel_domain.py:
#
#   1. KNOWN       the initial model: four worlds, the cleaner knowing the
#                  column, the concierge the floor, the porter and the guest
#                  neither; and no agent knowing the room.
#   2. EPISTEMIC   a policy of depth 7, four leaves, and at every leaf the
#                  whole goal: contained, the stand-down, the secret. Two lift
#                  rides, no inspection, three messages, none public.
#   3. FILTER      a policy of depth 6 for the broadcast fleet's goal with
#                  location pages ruled out; at every leaf the secret fails,
#                  and at the actual world it fails at the all-clear: the
#                  guest goes from four rooms to one.
#   4. BROADCAST   a policy of depth 5; at every leaf the secret fails.
#   5. SILOED      a policy of depth 8 for the leak contained, three
#                  inspections in its deepest branch; at every leaf the
#                  stand-down fails.
#   6. COST        no policy for the whole goal shorter than 7: the planner's
#                  iterative deepening, with the budget the epistemic fleet
#                  needed, does not stop below it.
#   7. LOBBY       the floor said in the lobby: the guest hears it, and can
#                  rule out the other floor.
#   8. UNREACHABLE the siloed fleet and a fleet with the public address alone
#                  reach no model where the whole goal holds; the epistemic
#                  fleet, searched the same way, does.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-/tmp/hotel_distributed_validate}"
TRACE="${HERE}/trace.py"
REACH="${HERE}/reach.py"

PLANK="${PLANK:-$(command -v plank || echo "${HOME}/plank/build/plank")}"
PLANNER="${EPISTEMIC_PLANNER:-${HOME}/eplansys_ws/install/aletheia/bin/epistemic_planner}"
LIB="${PLANK_LIB:-${HOME}/plank/benchmarks/libraries/intermediate.epddl}"
BUDGET="${BUDGET:-900}"

section() {
  echo
  echo "############################################################"
  echo "# $1"
  echo "############################################################"
}

# Grounds one fleet: ground <fleet>.
ground() {
  "${PLANK}" export -d "${OUT}/hotel-distributed.epddl" -p "${OUT}/$1.epddl" \
    -l "${LIB}" "${OUT}/earshot.epddl" -o "${OUT}/$1" > "${OUT}/$1.plank.log" 2>&1 \
    || { cat "${OUT}/$1.plank.log"; exit 1; }
}

# Solves one fleet with AO*: solve <fleet>. The search log is kept beside it.
solve() {
  "${PLANNER}" --task "${OUT}/$1/$1.json" --plan "${OUT}/$1/plan.json" --timeout "${BUDGET}" \
    --strategy aostar --no-portfolio --threads 1 > "${OUT}/$1/search.log" 2>&1 || true
  grep -E 'Solution found|No solution' "${OUT}/$1/search.log" || true
}

# Fails with the log shown: fail <message> <log>.
fail() {
  echo "FAILED: $1"
  [ -n "$2" ] && cat "$2"
  exit 1
}

# The policy's leaves must all give one verdict: leaves <log> <count> <safe> <stand-down> <secret>.
leaves() {
  local log="$1" n="$2"
  local want="safe $3  stand-down $4  secret $5"
  [ "$(grep -c '^ *leaf' "$log")" -eq "$n" ] || fail "expected $n leaves" "$log"
  [ "$(grep -c "$want" "$log")" -eq "$n" ] || fail "expected every leaf to give: $want" "$log"
  echo "  $n leaves, each: $want"
}

mkdir -p "${OUT}"
python3 "${HERE}/hotel_domain.py" --out "${OUT}" > /dev/null
for fleet in epistemic broadcast filter siloed pa-only; do
  ground "$fleet"
done
WHOLE="${OUT}/epistemic/epistemic.json"

section "1. KNOWN: who knows what at the start"
python3 "${TRACE}" --task "${WHOLE}" --leak L3_room1 --actions \
  > "${OUT}/known.log" 2>&1 || true
python3 - "${WHOLE}" <<'PY' | tee "${OUT}/known.log"
import json, sys
t = json.load(open(sys.argv[1]))
s = t['initial-state']
worlds, rel, lab = s['worlds'], s['relations'], s['labels']
rooms = sorted({a[7:] for w in worlds for a in lab[w] if a.startswith('source_')})
print(f'{len(worlds)} worlds, {len(s["designated"])} designated')
for agent in ('cleaner', 'concierge', 'porter', 'guest'):
    sizes = sorted({len({z for v in rel[agent][w] for z in rooms if 'source_' + z in lab[v]})
                    for w in worlds})
    print(f'{agent}: rooms it cannot rule out at each world: {sizes}')
both = all(len({z for v in set(rel['cleaner'][w]) & set(rel['concierge'][w])
                for z in rooms if 'source_' + z in lab[v]}) == 1 for w in worlds)
print('cleaner and concierge together: one room at every world' if both else 'NOT DISTRIBUTED')
PY
grep -q '^4 worlds, 4 designated' "${OUT}/known.log" || fail 'not four worlds' "${OUT}/known.log"
grep -q '^cleaner: rooms it cannot rule out at each world: \[2\]' "${OUT}/known.log" \
  || fail 'the cleaner should rule out half' "${OUT}/known.log"
grep -q '^concierge: rooms it cannot rule out at each world: \[2\]' "${OUT}/known.log" \
  || fail 'the concierge should rule out half' "${OUT}/known.log"
grep -q '^porter: rooms it cannot rule out at each world: \[4\]' "${OUT}/known.log" \
  || fail 'the porter should know nothing' "${OUT}/known.log"
grep -q '^guest: rooms it cannot rule out at each world: \[4\]' "${OUT}/known.log" \
  || fail 'the guest should know nothing' "${OUT}/known.log"
grep -q 'one room at every world' "${OUT}/known.log" \
  || fail 'the room should be distributed knowledge' "${OUT}/known.log"

section "2. EPISTEMIC: the whole goal at every leaf"
solve epistemic
grep -q 'Solution found at depth 7 ' "${OUT}/epistemic/search.log" \
  || fail 'no policy of depth 7' "${OUT}/epistemic/search.log"
python3 "${TRACE}" --task "${OUT}/epistemic/epistemic.json" --whole "${WHOLE}" \
  --plan "${OUT}/epistemic/plan.json" > "${OUT}/epistemic.log"
leaves "${OUT}/epistemic.log" 4 holds holds holds
[ "$(grep -c '(2 lift rides, 0 inspections, 3 private and 0 public messages)' "${OUT}/epistemic.log")" -eq 4 ] \
  || fail 'expected two rides and three private messages on every branch' "${OUT}/epistemic.log"
echo '  every branch: two lift rides, no inspection, three private messages'

section "3. FILTER: the all-clear tells the guest the room"
solve filter
grep -q 'Solution found at depth 6 ' "${OUT}/filter/search.log" \
  || fail 'no policy of depth 6' "${OUT}/filter/search.log"
python3 "${TRACE}" --task "${OUT}/filter/filter.json" --whole "${WHOLE}" \
  --plan "${OUT}/filter/plan.json" > "${OUT}/filter.log"
leaves "${OUT}/filter.log" 4 holds holds FAILS
python3 "${TRACE}" --task "${OUT}/filter/filter.json" --whole "${WHOLE}" --leak L3_room1 \
  --plan "${OUT}/filter/plan.json" > "${OUT}/filter-L3_room1.log"
guest=$(grep -E '^ *guest' "${OUT}/filter-L3_room1.log" | sed -E 's/ +/ /g; s/^ //' | cut -d' ' -f3- | uniq | tr '\n' '|')
echo "  guest, update by update: ${guest}"
[ "$(grep -A5 'page-safe_porter' "${OUT}/filter-L3_room1.log" | grep -c 'guest     rooms: L3_room1')" -eq 1 ] \
  || fail 'the guest should know the room after the all-clear' "${OUT}/filter-L3_room1.log"
[ "$(grep -B1 'page-safe_porter' "${OUT}/filter-L3_room1.log" | grep -c 'guest     rooms: 4 rooms')" -eq 1 ] \
  || fail 'the guest should not know the room before the all-clear' "${OUT}/filter-L3_room1.log"
echo '  the guest: four rooms until the all-clear, L3_room1 after it'

section "4. BROADCAST: the pages tell the guest the room"
solve broadcast
grep -q 'Solution found at depth 5 ' "${OUT}/broadcast/search.log" \
  || fail 'no policy of depth 5' "${OUT}/broadcast/search.log"
python3 "${TRACE}" --task "${OUT}/broadcast/broadcast.json" --whole "${WHOLE}" \
  --plan "${OUT}/broadcast/plan.json" > "${OUT}/broadcast.log"
leaves "${OUT}/broadcast.log" 4 holds holds FAILS

section "5. SILOED: the leak contained, and nobody told"
solve siloed
grep -q 'Solution found at depth 8 ' "${OUT}/siloed/search.log" \
  || fail 'no policy of depth 8' "${OUT}/siloed/search.log"
python3 "${TRACE}" --task "${OUT}/siloed/siloed.json" --whole "${WHOLE}" \
  --plan "${OUT}/siloed/plan.json" > "${OUT}/siloed.log"
leaves "${OUT}/siloed.log" 4 holds FAILS holds
deepest=$(grep -oE '[0-9]+ inspections' "${OUT}/siloed.log" | sort -n | tail -1)
[ "${deepest}" = '3 inspections' ] || fail "expected three inspections at the deepest leaf, got ${deepest}" "${OUT}/siloed.log"
echo "  the deepest branch opens three rooms; the fourth is known by elimination"

section "6. COST: no shorter policy for the whole goal"
for d in 1 2 3 4 5 6; do
  grep -q "Trying depth $d\$" "${OUT}/epistemic/search.log" \
    || fail "the search never tried depth $d" "${OUT}/epistemic/search.log"
done
echo '  AO* tried every depth from 1 to 6 and found no policy: 7 is the fewest actions'
echo '  for the whole goal; 6 suffice when the secret is left to a rule (step 3)'

section "7. LOBBY: the floor said where the guest is"
python3 "${TRACE}" --task "${WHOLE}" --whole "${WHOLE}" --leak L3_room1 \
  --actions tell-floor_concierge_lobby > "${OUT}/lobby.log"
grep -q 'guest     rooms: L3_room1, L3_room15' "${OUT}/lobby.log" \
  || fail 'the guest should hear the floor in the lobby' "${OUT}/lobby.log"
echo '  told in the lobby, the floor reaches the guest: it rules out L2'

section "8. UNREACHABLE: no channel, or the public address alone"
for fleet in siloed pa-only; do
  python3 "${REACH}" --task "${OUT}/${fleet}/${fleet}.json" --whole "${WHOLE}" > "${OUT}/reach-${fleet}.log"
  grep -q 'the whole goal is unreachable' "${OUT}/reach-${fleet}.log" \
    || fail "the whole goal should be unreachable for ${fleet}" "${OUT}/reach-${fleet}.log"
  echo "  ${fleet}: $(head -1 "${OUT}/reach-${fleet}.log"); the whole goal in none"
done
grep -q 'stand-down holds in 0 of them' "${OUT}/reach-siloed.log" \
  || fail 'the siloed fleet should never reach the stand-down' "${OUT}/reach-siloed.log"
echo '  siloed: the stand-down in none of them'
python3 "${REACH}" --task "${OUT}/epistemic/epistemic.json" --whole "${WHOLE}" --witness \
  > "${OUT}/reach-epistemic.log"
grep -q 'the whole goal is reachable' "${OUT}/reach-epistemic.log" \
  || fail 'the same search should find the whole goal for the epistemic fleet' "${OUT}/reach-epistemic.log"
echo "  epistemic, as a control: $(head -1 "${OUT}/reach-epistemic.log")"

echo
echo "all eight checks passed"
