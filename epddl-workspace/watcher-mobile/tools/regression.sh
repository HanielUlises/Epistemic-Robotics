#!/bin/bash
# Subjective branching against the installed planner, on every task at hand.
#
#     regression.sh OUT.txt
#
# Aletheia's own benchmarks: the changed binary without --subjective must
# return what the installed one does, expansions and validity. The EPDDL
# workspace's domains: with --subjective auto, depth and leaves of every policy
# against the installed binary's. One line per task.
set -u
OUT=$(realpath -m "$1"); WS=$(realpath "$(dirname "$0")/../../..")
NEW=${SUBJECTIVE_PLANNER:-$HOME/aletheia-subjective/build/epistemic_planner}
OLD=${EPISTEMIC_PLANNER:-$HOME/eplansys_ws/install/aletheia/bin/epistemic_planner}
LIB=$HOME/plank/benchmarks/libraries/intermediate.epddl
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
r() { timeout 140 "$@" --conditional --timeout 120 --threads 1 2>&1 | grep -oE 'depth [0-9]+  Expanded=[0-9]+|[0-9]+ leaves|No solution' | tail -2 | tr '\n' ' '; }
: > "$OUT"
for t in $(find "$HOME/aletheia-subjective/benchmarks" -name '*.json' | xargs grep -l '"planning-task-info"' | sort); do
  a=$(r "$OLD" --task "$t" --plan "$TMP/a.json"); b=$(r "$NEW" --task "$t" --plan "$TMP/b.json")
  echo "benchmark $(basename "$(dirname "$t")")/$(basename "$t") | $([ "$a" = "$b" ] && echo same || echo DIFFERENT) | $a | $b" >> "$OUT"
done
for d in "$WS"/epddl-workspace/*/; do
  name=$(basename "$d"); [ "$name" = watcher-mobile ] && continue
  for p in "$d"instances/*.epddl "$d"*problem*.epddl; do
    [ -f "$p" ] || continue
    dom=$(ls "$d"*.epddl | grep -v -- '-look\|problem\|sightlines\|gate.epddl' | head -1)
    libs=$(ls "$d"*.epddl | grep -E '/sightlines.epddl|/gate.epddl' | tr '\n' ' ')
    o="$TMP/$name"; rm -rf "$o"
    "$HOME/plank/build/plank" export -d "$dom" -p "$p" -l "$LIB" $libs -o "$o" > /dev/null 2>&1 || continue
    t=$(ls "$o"/*.json | head -1); [ -f "$t" ] || continue
    a=$(r "$OLD" --task "$t" --plan "$TMP/a.json"); b=$(r "$NEW" --task "$t" --plan "$TMP/b.json" --subjective auto)
    da=$(echo "$a" | grep -oE 'depth [0-9]+|[0-9]+ leaves' | tr '\n' ' '); db=$(echo "$b" | grep -oE 'depth [0-9]+|[0-9]+ leaves' | tr '\n' ' ')
    echo "workspace $name/$(basename "$p") | $([ "$da" = "$db" ] && echo same || echo DIFFERENT) | $a | $b" >> "$OUT"
  done
done
