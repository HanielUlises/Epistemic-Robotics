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
# The stale-maps domain, grounded, solved and traced without a simulator.
#
#     tools/validate.sh [out directory, default /tmp/stale_maps_validate]
#
#   1. FLOOR     the floor's claims, on the plan: who sees each change, and
#                where a current map and a stale one send each hauler.
#   2. STALE     the forklift's two actions: every robot believes the map its
#                station leaves it with, eleven entries of eight maps are
#                false, and so are some beliefs about other robots' maps.
#   3. RADIO     a policy; every map sent goes to a hauler, every hauler
#                crosses t3, and every robot's belief stays consistent.
#   4. GUARD     a stale map is not sent over a fresh one: the precondition
#                that the sender believe the recipient holds the opposite
#                fails.
#   5. COLLAPSE  a received map taken as fact: a private announcement that
#                t3 is open leaves a robot whose map says otherwise with no
#                world.
#   6. SILENT    no policy without a radio.
#   7. DOUBT     robots that know maps go stale hold no false belief after
#                the forklift, and the policy looks before it crosses.
#   8. RESYNC    every map brought up to date with one message per stale
#                entry, which is the least any policy can send.
#
# Each step fails the script if its tool says anything other than the
# expected result.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EPDDL="${HERE}/../epddl"
OUT="${1:-/tmp/stale_maps_validate}"
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
  "${PLANK}" export -d "${EPDDL}/stale-maps.epddl" -p "${EPDDL}/$1.epddl" \
    -l "${LIB}" "${EPDDL}/maps.epddl" -o "${OUT}/$1" \
    > "${OUT}/$1.plank.log" 2>&1 || { cat "${OUT}/$1.plank.log"; exit 1; }
}

solve() {
  local floor="$1"
  "${PLANNER}" --task "${OUT}/${floor}/${floor}.json" --plan "${OUT}/${floor}/plan.json" \
    --timeout 240 --consistent-beliefs 2>&1 | tee "${OUT}/${floor}/plan.log" \
    | grep -E 'Solution found|No solution|exhausted|validator'
}

mkdir -p "${OUT}"
for floor in radio silent doubt resync; do ground "${floor}"; done
FLEET="${EPDDL}/fleet.json"

section "1. FLOOR: who sees each change, and where each map sends a hauler"
python3 "${HERE}/check_floor.py" | tee "${OUT}/floor.log"

section "2. STALE: the forklift's two actions"
python3 "${TRACE}" --task "${OUT}/radio/radio.json" --actions clear_t3 stage_t1 \
  --about t1 t3 --json "${OUT}/stale.json" | tee "${OUT}/stale.log"
python3 - "${OUT}/stale.json" "${FLEET}" <<'EOF'
import json, sys
steps = json.load(open(sys.argv[1]))
fleet = json.load(open(sys.argv[2]))
maps = steps[-1]['maps']
blocked = set(fleet['shift_blocked'])
problems = []
for a in fleet['agents']:
    mine = set(blocked)
    for t, kind in fleet['changes'].items():
        if a in fleet['sees'][t]:
            (mine.add if kind == 'stage' else mine.discard)(t)
    held = ''.join('x' if t in mine else 'o' for t in fleet['bays'])
    believed = ''.join(maps[a]['map'])
    if held != believed:
        problems.append(f'{a} believes {believed}, its station leaves it {held}')
print(f'the model gives every robot the map its station leaves it: '
      f'{"no" if problems else "yes"}; {steps[-1]["stale"]} stale entries')
for p in problems:
    print('FAIL: ' + p)
sys.exit(1 if problems or steps[-1]['stale'] != 11 else 0)
EOF
grep -q 'second-order beliefs about t1 are false' "${OUT}/stale.log"

section "3. RADIO: a policy, maps sent only to haulers"
solve radio | grep -q 'Solution found' || { echo "FAIL: expected a policy"; exit 1; }
python3 "${TRACE}" --task "${OUT}/radio/radio.json" --plan "${OUT}/radio/plan.json" \
  | tee "${OUT}/radio.trace"
grep -q 'goal holds' "${OUT}/radio.trace"
for to in $(grep -oE 'send-(open|blocked)_r[0-9]+_r[0-9]+' "${OUT}/radio.trace" | cut -d_ -f3); do
  python3 -c "import json,sys; sys.exit(0 if '${to}' in json.load(open('${FLEET}'))['haulers'] else 1)" \
    || { echo "FAIL: a map was sent to ${to}, which hauls nothing"; exit 1; }
done
grep -oE 'cross_r[0-9]+_t[0-9]' "${OUT}/radio.trace" | grep -qv '_t3$' \
  && { echo "FAIL: a hauler crossed a bay other than t3"; exit 1; }
echo "every map sent went to a hauler; every hauler crossed t3"

section "4. GUARD: a stale map is not sent over a fresh one"
if python3 "${TRACE}" --task "${OUT}/radio/radio.json" \
    --actions clear_t3 stage_t1 send-open_r4_r2_t1 | tee "${OUT}/guard.log"; then
  echo "FAIL: expected r4's stale t1 not to be sendable to r2"; exit 1
fi
grep -q 'send-open_r4_r2_t1  NOT APPLICABLE' "${OUT}/guard.log"

section "5. COLLAPSE: a received map taken as fact"
python3 "${TRACE}" --task "${OUT}/radio/radio.json" \
  --actions clear_t3 stage_t1 tell:r4:t3:open | tee "${OUT}/collapse.log"
grep -q 'r4 has no world' "${OUT}/collapse.log" \
  || { echo "FAIL: expected r4 to be left with no world"; exit 1; }

section "6. SILENT: no policy without a radio"
if solve silent | grep -q 'Solution found'; then
  echo "FAIL: expected no policy on the silent floor"; exit 1
fi
echo "no policy"

section "7. DOUBT: no false belief, and looking before crossing"
python3 "${TRACE}" --task "${OUT}/doubt/doubt.json" \
  --actions clear-doubted_t3 stage-doubted_t1 --json "${OUT}/doubt.json" | tee "${OUT}/doubt.log"
python3 -c "import json,sys; sys.exit(0 if json.load(open('${OUT}/doubt.json'))[-1]['stale'] == 0 else 1)" \
  || { echo "FAIL: expected no stale entry on the doubting floor"; exit 1; }
solve doubt | grep -q 'Solution found' || { echo "FAIL: expected a policy"; exit 1; }
python3 "${TRACE}" --task "${OUT}/doubt/doubt.json" --plan "${OUT}/doubt/plan.json" \
  | tee "${OUT}/doubt.trace"
grep -q 'look_' "${OUT}/doubt.trace" || { echo "FAIL: expected the policy to look"; exit 1; }

section "8. RESYNC: one message per stale entry"
solve resync | grep -q 'Solution found' || { echo "FAIL: expected a policy"; exit 1; }
python3 "${TRACE}" --task "${OUT}/resync/resync.json" --plan "${OUT}/resync/plan.json" \
  | tee "${OUT}/resync.trace"
sends=$(grep -cE '^ *send-(open|blocked)_' "${OUT}/resync.trace")
[ "${sends}" -eq 11 ] || { echo "FAIL: ${sends} messages, expected 11"; exit 1; }
grep -q '0 stale entries; every robot has a consistent belief' "${OUT}/resync.trace"
echo "11 messages, one per stale entry; no map stale at the end"

echo
echo "all eight checks passed"
