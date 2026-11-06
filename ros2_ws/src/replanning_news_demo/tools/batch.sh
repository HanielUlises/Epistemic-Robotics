#!/bin/bash
# Runs of replanning on news: for each act, its seeds under each formulation,
# LANES runs at a time.
#
#     batch.sh OUT [LANES]
#
# SEEDS_1, SEEDS_2 and SEEDS_3 name each act's seeds ("1 2 3" by default,
# one cycle through the act's worlds), FORMS the formulations. The runs are
# dealt round-robin to the lanes, and each lane runs its share one after
# another, so that no two runs ever share a lane. A run already in OUT with a
# mission.json is skipped, so a batch that stopped can be resumed.
set -u
OUT=$(realpath -m "$1"); LANES=${2:-4}
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT"
jobs=()
for act in 1 2 3; do
  var="SEEDS_$act"
  for s in ${!var:-1 2 3}; do
    for f in ${FORMS:-replan null keep classical}; do
      jobs+=("$act $s $f")
    done
  done
done
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
