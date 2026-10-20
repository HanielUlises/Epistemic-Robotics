#!/bin/bash
# Every run of the link grades: seeds FIRST..LAST, each under the three grades
# and the four formulations, LANES runs at a time.
#
#     batch.sh FIRST LAST OUT [LANES]
#
# The runs are dealt round-robin to the lanes, and each lane runs its share one
# after another, so that no two runs ever share a lane. A run already in OUT
# with a mission.json is skipped, so a batch that stopped can be resumed.
# FORMS, if set, names the formulations to run.
set -u
FIRST=$1; LAST=$2; OUT=$(realpath -m "$3"); LANES=${4:-5}
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT"
jobs=()
for s in $(seq "$FIRST" "$LAST"); do
  for g in full partial none; do
    for f in ${FORMS:-epistemic visit report replan}; do
      jobs+=("$s $f $g")
    done
  done
done
lane() {
  local l=$1
  for ((k = l - 1; k < ${#jobs[@]}; k += LANES)); do
    set -- ${jobs[$k]}
    local dir="$OUT/$2-$3-s$1"
    [ -f "$dir/mission.json" ] && continue
    bash "$HERE/run_one.sh" "$1" "$2" "$3" "$dir" "$l" >> "$OUT/batch.log" 2>&1
  done
}
date '+%F %T start' >> "$OUT/batch.log"
for l in $(seq 1 "$LANES"); do lane "$l" & done
wait
date '+%F %T finished' >> "$OUT/batch.log"
