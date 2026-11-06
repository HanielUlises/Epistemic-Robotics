#!/bin/bash
# One headless run of replanning on news in its own process session.
#
#     run_one.sh ACT SEED FORMULATION OUT [LANE]
#
# A lane is a ROS domain (50 + LANE) and a Gazebo port (11450 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched.
set +u
ACT=$1; SEED=$2; FORM=$3; OUT=$(realpath -m "$4"); LANE=${5:-1}
WALL_LIMIT=${WALL_LIMIT:-1500}
WS=${NEWS_WS:-$(realpath "$(dirname "$(realpath "$0")")/../../..")}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
# The planning core with subjective branching, and the planner plugin built on
# it, over the installed ones (see watcher_moves_demo's README).
source "$HOME/watcher_overlay_ws/install/setup.bash"
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((50 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11450 + LANE))
SHARE=$(ros2 pkg prefix replanning_news_demo)/share/replanning_news_demo
python3 "$SHARE/tools/make_world.py" --act "$ACT" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch replanning_news_demo news_launch.py dir:='$OUT' act:=$ACT formulation:=$FORM" \
  > "$OUT/launch.log" 2>&1
pkill -KILL -s "$(cat "$OUT/sid")" 2>/dev/null
python3 - "$OUT" <<'PY'
import json, os, sys
out = sys.argv[1]
p = os.path.join(out, 'mission.json')
l = json.load(open(os.path.join(out, 'launch.json')))
if not os.path.exists(p):
    json.dump({'act': l['act'], 'formulation': l['formulation'], 'result': 'NO_OUTCOME'}, open(p, 'w'))
m = json.load(open(p))
print(f"act {l['act']} {l['formulation']} s{l['seed']} ({l['tag']}): {m.get('result')} "
      f"{m.get('t_mission_sim', 0)} s; delivered {m.get('delivered')}; plans {len(m.get('plans', []))}; "
      f"found shut {[f['door'] for f in m.get('found_shut', [])]}")
PY
