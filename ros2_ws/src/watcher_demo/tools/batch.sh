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
# The batch: every seed under all three formulations, paired.
#
#     batch.sh FIRST LAST OUT [SEEDS_AT_ONCE]
#
# The seed picks the world, cycling through the four (tools/make_world.py).
# Each seed's three runs go side by side on lanes of their own, so the three
# share the machine's load as well as the world. A run that ends without an
# outcome (gzserver can fail to start when several start at once) is run
# again, once, and the first attempt is kept beside it. A run that already has
# its outcome is not repeated, so the batch resumes.
set +u
FIRST=$1; LAST=$2; OUT=$(realpath -m "$3"); AT_ONCE=${4:-2}
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT/runs"

ended() {
  grep -q '"result": "\(SUCCESS\|FAILURE\|LIMIT\)"' "$1/mission.json" 2>/dev/null
}

one() {  # seed formulation lane
  local dir="$OUT/runs/$2-s$1"
  ended "$dir" && return 0
  [ -d "$dir" ] && rm -rf "$dir"
  bash "$HERE/run_one.sh" "$1" "$2" "$dir" "$3" >> "$OUT/batch.log" 2>&1
  if ! ended "$dir"; then
    mv "$dir" "$dir.first-attempt"
    echo "$(date +%T) $2 s$1: no outcome on the first attempt; again" >> "$OUT/batch.log"
    bash "$HERE/run_one.sh" "$1" "$2" "$dir" "$3" >> "$OUT/batch.log" 2>&1
  fi
}

for ((s = FIRST; s <= LAST; s += AT_ONCE)); do
  for ((k = 0; k < AT_ONCE && s + k <= LAST; k++)); do
    # Started a few seconds apart, so that no two stacks configure at once.
    one $((s + k)) epistemic $((3 * k + 1)) &
    sleep 4
    one $((s + k)) direct $((3 * k + 2)) &
    sleep 4
    one $((s + k)) discreet $((3 * k + 3)) &
    sleep 4
  done
  wait
  echo "$(date +%T) seeds $s..$((s + AT_ONCE - 1)) done" >> "$OUT/batch.log"
done
echo "$(date +%T) batch over" >> "$OUT/batch.log"
