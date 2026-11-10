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
# One run in its own lane and process session.
#
#   run_one.sh ACT SEED FORMULATION OUT [LANE]
#
# Lane L uses ROS domain 40 + L and Gazebo port 11440 + L. Everything the
# launch starts runs in one process session, killed whole when the launch
# ends, or after WALL_LIMIT seconds.
set +u
ACT=$1; SEED=$2; FORM=$3; OUT=$(realpath -m "$4"); LANE=${5:-1}
WALL_LIMIT=${WALL_LIMIT:-2400}
WS=${FLEET_WS:-$(realpath "$(dirname "$(realpath "$0")")/../../..")}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/watcher_overlay_ws/install/setup.bash"
# PlanSys2's executor with patches/plansys2_executor-stale-requests.patch applied,
# built in a workspace of its own (see the README)
source "${EXECUTOR_PATCH_WS:-$HOME/eplansys_patch_ws}/install/setup.bash"
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((40 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11440 + LANE))
# headless: the lasers are ray sensors and need no GL; a server that opens the
# desktop's display can fail there when several start at once
unset DISPLAY WAYLAND_DISPLAY
SHARE=$(ros2 pkg prefix faulty_sensor_demo)/share/faulty_sensor_demo
python3 "$SHARE/tools/make_world.py" --act "$ACT" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch faulty_sensor_demo fleet_launch.py dir:='$OUT' act:=$ACT formulation:=$FORM" \
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
      f"{m.get('t_mission_sim', 0)} s; arrived {m.get('arrived')}")
PY
