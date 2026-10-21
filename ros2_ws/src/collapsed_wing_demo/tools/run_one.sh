#!/bin/bash
# One headless run of the collapsed wing in its own process session.
#
#     run_one.sh SEED FORMULATION OUT [LANE]
#
# A lane is a ROS domain (80 + LANE) and a Gazebo port (11480 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched.
set +u
SEED=$1; FORM=$2; OUT=$(realpath -m "$3"); LANE=${4:-1}
WALL_LIMIT=${WALL_LIMIT:-1800}
WS=${LINK_WS:-$(realpath "$(dirname "$(realpath "$0")")/../../..")}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
# The planning core with subjective branching, and the planner plugin built on
# it, over the installed ones (see watcher_moves_demo's README).
source "$HOME/watcher_overlay_ws/install/setup.bash"
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((80 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11480 + LANE))
SHARE=$(ros2 pkg prefix collapsed_wing_demo)/share/collapsed_wing_demo
python3 "$SHARE/tools/make_world.py" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch collapsed_wing_demo wing_launch.py dir:='$OUT' formulation:=$FORM" \
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
w = json.load(open(os.path.join(out, 'world.json')))
clear = ''.join(d.upper() if c else d for d, c in w['clear'].items())
print(f"{l['formulation']} s{w['seed']} ({clear}): {m.get('result')} {m.get('t_mission_sim', 0)} s; "
      f"medic with the victim {m.get('medic_with_victim')}; found blocked {[f['way'] for f in m.get('found_blocked', [])]}")
PY
