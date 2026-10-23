#!/bin/bash
# One headless Gazebo run of the gossip demonstration in its own process
# session.
#
#     run_one.sh SEED CASE FORMULATION OUT [LANE]
#
# A lane is a ROS domain (90 + LANE) and a Gazebo port (11490 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched.
set +u
SEED=$1; CASE=$2; FORM=$3; OUT=$(realpath -m "$4"); LANE=${5:-1}
WALL_LIMIT=${WALL_LIMIT:-2400}
WS=${GC_WS:-$(realpath "$(dirname "$(realpath "$0")")/../../..")}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
# Aletheia with subjective branching, and the planner plugin built on it,
# over the installed ones (see watcher_moves_demo's README).
source "$HOME/watcher_overlay_ws/install/setup.bash"
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((90 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11490 + LANE))
SHARE=$(ros2 pkg prefix gossip_coverage_demo)/share/gossip_coverage_demo
python3 "$SHARE/tools/make_world.py" --seed "$SEED" --case "$CASE" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch gossip_coverage_demo gc_launch.py dir:='$OUT' formulation:=$FORM" \
  > "$OUT/launch.log" 2>&1
pkill -KILL -s "$(cat "$OUT/sid")" 2>/dev/null
python3 - "$OUT" <<'PY'
import json, os, sys
out = sys.argv[1]
p = os.path.join(out, 'mission.json')
l = json.load(open(os.path.join(out, 'launch.json')))
if not os.path.exists(p):
    json.dump({'formulation': l['formulation'], 'result': 'NO_OUTCOME'}, open(p, 'w'))
m = json.load(open(p))
tasks = {t: (x.get('k'), x.get('found_t'), x.get('done_t')) for t, x in (m.get('tasks') or {}).items()}
print(f"{l['formulation']} s{l['seed']} case {l['case']}: {m.get('result')} at {m.get('mission')} s; tasks {tasks}")
PY
