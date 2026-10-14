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
# The batch: every seed under the five formulations and both watcher
# strategies, paired.
#
#     batch.sh FIRST LAST OUT
#
# The seed picks the world, cycling through the four (tools/make_world.py).
# A seed's five runs under one strategy go side by side, lanes 1..5, then the
# five under the other. A run that ends without an
# outcome is run again, once, and the first attempt is kept beside it; a run
# that already has its outcome is not repeated, so the batch resumes.
set +u
FIRST=$1; LAST=$2; OUT=$(realpath -m "$3")
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT/runs"

ended() {
  grep -q '"result": "\(SUCCESS\|FAILURE\|LIMIT\)"' "$1/mission.json" 2>/dev/null
}

one() {  # seed formulation strategy lane
  local dir="$OUT/runs/$2-$3-s$1"
  ended "$dir" && return 0
  [ -d "$dir" ] && rm -rf "$dir"
  bash "$HERE/run_one.sh" "$1" "$2" "$3" "$dir" "$4" >> "$OUT/batch.log" 2>&1
  if ! ended "$dir"; then
    mv "$dir" "$dir.first-attempt"
    echo "$(date +%T) $2 $3 s$1: no outcome on the first attempt; again" >> "$OUT/batch.log"
    bash "$HERE/run_one.sh" "$1" "$2" "$3" "$dir" "$4" >> "$OUT/batch.log" 2>&1
  fi
}

for ((s = FIRST; s <= LAST; s++)); do
  for strat in stay chase; do
    lane=1
    for form in v1 shortest opaque direct discreet; do
      one "$s" "$form" "$strat" "$lane" &
      lane=$((lane + 1))
      sleep 4
    done
    wait
  done
  echo "$(date +%T) seed $s done" >> "$OUT/batch.log"
done
echo "$(date +%T) batch over" >> "$OUT/batch.log"
