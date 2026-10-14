#!/bin/bash
# One headless run of the watcher that moves in its own process session.
#
#     run_one.sh SEED FORMULATION STRATEGY OUT [LANE]
#
# A lane is a ROS domain (60 + LANE) and a Gazebo port (11360 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched.
set +u
SEED=$1; FORM=$2; STRAT=$3; OUT=$(realpath -m "$4"); LANE=${5:-1}
WALL_LIMIT=${WALL_LIMIT:-900}
WS=${WATCHER_WS:-$(realpath "$(dirname "$(realpath "$0")")/../../..")}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
# The planning core with subjective branching, and the planner plugin built on
# it, over the installed ones (see README).
source "$HOME/watcher_overlay_ws/install/setup.bash"
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((60 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11360 + LANE))
SHARE=$(ros2 pkg prefix watcher_moves_demo)/share/watcher_moves_demo
python3 "$SHARE/tools/make_world.py" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch watcher_moves_demo watcher_launch.py dir:='$OUT' formulation:=$FORM strategy:=$STRAT" \
  > "$OUT/launch.log" 2>&1
pkill -KILL -s "$(cat "$OUT/sid")" 2>/dev/null
python3 - "$OUT" "$FORM-$STRAT" "$SEED" <<'PY'
import json, os, sys
out, form, seed = sys.argv[1:]
p = os.path.join(out, 'mission.json')
if not os.path.exists(p):
    json.dump({'formulation': form, 'result': 'NO_OUTCOME'}, open(p, 'w'))
m = json.load(open(p))
obs = {}
for r in ('r2', 'r3'):
    q = os.path.join(out, f'observer_{r}.json')
    obs[r] = json.load(open(q)).get('conclusion', '?') if os.path.exists(q) else '?'
w = json.load(open(os.path.join(out, 'world.json')))
print(f"{form} s{seed} ({w['order']}, watcher {w['watcher']}): {m.get('result')} {m.get('t_mission_sim', 0)} s;"
      f" ally saw '{obs['r2']}', watcher saw '{obs['r3']}'; model {m.get('model')}")
PY
