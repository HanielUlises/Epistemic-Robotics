#!/bin/bash
# Every run of the belief and empathy demonstration: seeds FIRST..LAST, each
# under every formulation, LANES runs at a time.
#
#     batch.sh FIRST LAST OUT [LANES]
#
# The runs are dealt round-robin to the lanes, and each lane runs its share one
# after another, so that no two runs ever share a lane. A run already in OUT
# with a mission.json is skipped, so a batch that stopped can be resumed.
# FORMS, if set, names the formulations to run; VARIANT=latent runs the
# variant with latent tool faults, by default under its four formulations.
set -u
FIRST=$1; LAST=$2; OUT=$(realpath -m "$3"); LANES=${4:-4}
HERE=$(dirname "$(realpath "$0")")
mkdir -p "$OUT"
jobs=()
for s in $(seq "$FIRST" "$LAST"); do
  if [ "${VARIANT:-replica}" = latent ]; then default="epistemic timed published strong ideal"
  else default="epistemic particles ideal flock"; fi
  for f in ${FORMS:-$default}; do
    jobs+=("$s $f")
  done
done
lane() {
  local l=$1
  for ((k = l - 1; k < ${#jobs[@]}; k += LANES)); do
    set -- ${jobs[$k]}
    local dir="$OUT/$2-s$1"
    [ -f "$dir/mission.json" ] && continue
    bash "$HERE/run_one.sh" "$1" "$2" "$dir" "$l" >> "$OUT/batch.log" 2>&1
  done
}
date '+%F %T start' >> "$OUT/batch.log"
for l in $(seq 1 "$LANES"); do lane "$l" & done
wait
date '+%F %T finished' >> "$OUT/batch.log"
