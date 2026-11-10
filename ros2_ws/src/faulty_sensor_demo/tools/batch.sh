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
# A batch of runs over lanes.
#
#   [ACTS="1 2"] batch.sh OUT [LANES]
#
# Every distinct path of every policy is run once: a policy takes the same path
# on every night on which its lasers read the same, and tools/analyse.py --jobs
# lists the first night of each. Runs already written are skipped.
set -u
OUT=$(realpath -m "$1"); LANES=${2:-4}
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT"
mapfile -t jobs < <(python3 "$HERE/analyse.py" --jobs ${ACTS:-})
lane() {
  local l=$1
  for ((k = l - 1; k < ${#jobs[@]}; k += LANES)); do
    set -- ${jobs[$k]}
    local dir="$OUT/a$1-$3-s$2"
    [ -f "$dir/mission.json" ] && continue
    bash "$HERE/run_one.sh" "$1" "$2" "$3" "$dir" "$l" >> "$OUT/batch.log" 2>&1
  done
}
date '+%F %T start' >> "$OUT/batch.log"
for l in $(seq 1 "$LANES"); do lane "$l" & done
wait
date '+%F %T finished' >> "$OUT/batch.log"
