#!/bin/bash
# Copyright 2026 Haniel Vásquez Morales
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
# Records one fleet's shift: Gazebo on the left half of a virtual display,
# RViz on the right, and the run's own log beside the capture.
#
#     record_demo.sh epistemic /tmp/raw_epistemic.mkv /tmp/run_epistemic.log
#
# The display is an Xvfb screen carrying the two windows and nothing else, as
# in pass_through_demo's recorder, whose notes on window placement apply.
# The Gazebo camera cuts to whatever the crew has set to work: a hauler, which
# it chases, or the forklift, which has a fixed shot down the bay it changes.
#
# The capture's start time is written beside it as <raw>.t0, so that the
# captions can be timed from the log.
set +u

FLEET=${1:-epistemic}
RAW=${2:-/tmp/stale_maps_raw.mkv}
LOG=${3:-/tmp/stale_maps_run.log}
DISP=${DISP:-:99}
WIDTH=3840
HEIGHT=1080
FPS=${FPS:-12}
FOLLOW=/tmp/stale_maps_follow

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/rmf_ws/install/setup.bash" 2>/dev/null
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
# An overlay of this package alone, built while the shared install space is
# in use by other runs and must not be rebuilt under them.
[ -n "$STALE_MAPS_OVERLAY" ] && source "$STALE_MAPS_OVERLAY/setup.bash"

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1

# A ROS domain and a Gazebo master of this demo's own. Another session may be
# running its own simulation on the same machine; on the default domain and
# port the two would share /clock, /spawn_entity and the Gazebo master, and
# each would corrupt the other's run without an error.
export ROS_DOMAIN_ID=${STALE_MAPS_DOMAIN:-42}
export GAZEBO_MASTER_URI=http://localhost:${STALE_MAPS_GAZEBO_PORT:-11399}

# Only processes of this demo: those whose environment carries its ROS
# domain. Killing gzserver, rviz2 or plansys2_node by name would kill another
# session's simulation as well.
kill_stack() {
  local pid
  for pid in $(pgrep -u "$USER" -f "gzserver|gzclient|rviz2|plansys2_node|epistemic_state_node|robot_state_publisher|static_transform_publisher|living_map|stale_maps|relay_action|chase_camera|knowledge_view|camera_director|ros2 launch"); do
    [ "$pid" = "$$" ] && continue
    if { tr '\0' '\n' < "/proc/$pid/environ"; } 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then
      kill -9 "$pid" 2>/dev/null
    fi
  done
  sleep 3
}
kill_stack

# A gzserver this script did not start, registered on this script's Gazebo
# master. Its scene reaches this gzclient, and the recording then shows
# another world over the robots' own: this happened, on three takes.
foreign_server() {
  local pid
  for pid in $(pgrep -u "$USER" -x gzserver); do
    local env
    env=$({ tr '\0' '\n' < "/proc/$pid/environ"; } 2>/dev/null)
    if grep -qx "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI" <<< "$env" &&
       ! grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" <<< "$env"; then
      echo "$pid"
      return 0
    fi
  done
  return 1
}
if pid=$(foreign_server); then
  echo "another gzserver (pid $pid) is on $GAZEBO_MASTER_URI; set STALE_MAPS_GAZEBO_PORT" >&2
  exit 1
fi

pgrep -x Xvfb > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_stale_maps.log 2>&1 &
  sleep 3
}

GUI_INI=/tmp/stale_maps_gui.ini
cat > "$GUI_INI" <<INI
[geometry]
x = 0
y = 0
width = 1920
height = 1080
INI
export GAZEBO_GUI_INI_FILE=$GUI_INI

python3 - "$FOLLOW" <<'PY'
import sys
sys.path.insert(0, __import__('os').path.join(
    __import__('ament_index_python.packages', fromlist=['x']).get_package_share_directory('stale_map_demo'), 'tools'))
import layout as L
open(sys.argv[1], 'w').write(L.shot_text(L.OPENING_SHOT) + '\n')
PY

# The Gazebo client's first view is the world's opening shot, a fixed pose, and
# is compared with a reference taken from a clean take. Three takes once showed
# another session's world in the client while this world ran in the server;
# such a take is torn down and launched again.
REFERENCE="$(ros2 pkg prefix stale_map_demo)/share/stale_map_demo/config/opening_reference.png"
opening_matches() {
  local grab=/tmp/stale_maps_opening.png
  ffmpeg -y -loglevel error -f x11grab -video_size 1631x943 -i "$DISP+269,69" \
         -vf "scale=160:92,format=gray" -frames:v 1 "$grab" < /dev/null || return 1
  python3 - "$grab" "$REFERENCE" <<'PY'
import sys
import numpy as np
from PIL import Image
a, b = (np.asarray(Image.open(p).convert('L'), float) for p in sys.argv[1:3])
d = float(np.abs(a - b).mean())
print(f'opening shot differs from the reference by {d:.1f}')
sys.exit(0 if d < 20.0 else 1)
PY
}

for attempt in 1 2 3 4; do
  setsid ros2 launch stale_map_demo stale_maps_launch.py \
      fleet:="$FLEET" gui:=true rviz:=true camera:=true follow_file:="$FOLLOW" \
      hold:="${HOLD:-10}" start_after:=50 shutdown:=false \
      policy_out:="${RAW%.*}_policy.json" > "$LOG" 2>&1 &
  LAUNCH=$!
  trap 'kill $CHASE $WATCH 2>/dev/null; kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

  for _ in $(seq 1 150); do
    [ "$(xdotool search --name '^(Gazebo|.*RViz.*)$' 2>/dev/null | wc -l)" -ge 2 ] && break
    sleep 2
  done
  for _ in $(seq 1 90); do
    grep -qi "spawned entity \[r8\]" "$LOG" && break
    sleep 2
  done
  sleep 5
  if opening_matches; then
    break
  fi
  echo "attempt $attempt: the Gazebo client does not show this world; launching again"
  kill -KILL -$LAUNCH 2>/dev/null
  kill_stack
done

CHASE_LOG="${LOG%.*}_camera.log"
: > "$CHASE_LOG"
chase_forever() {
  local bin
  bin="$(ros2 pkg prefix warehouse_xl_rmf_demo)/lib/warehouse_xl_rmf_demo/chase_camera"
  while :; do
    "$bin" --model r2 --follow-file "$FOLLOW" --distance "${CAM_DISTANCE:-3.2}" \
           --height "${CAM_HEIGHT:-1.8}" --look 0.25 --tau 0.5 --rate 25 \
           --publish-rate "${CAM_PUBLISH_RATE:-5}" >> "$CHASE_LOG" 2>&1
    echo "chase_camera exited with $?; restarting" >> "$CHASE_LOG"
    sleep 2
  done
}
chase_forever &
CHASE=$!
sleep 6

date +%s.%N > "${RAW%.*}.t0"
ffmpeg -y -hide_banner -nostdin -loglevel error -f x11grab -framerate "$FPS" \
       -video_size ${WIDTH}x${HEIGHT} -i "$DISP+0,0" \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$RAW" < /dev/null &
FF=$!

# Sampled through the take, since a foreign server may come and go.
rm -f "${RAW%.*}.contaminated"
( while :; do foreign_server > /dev/null && touch "${RAW%.*}.contaminated"; sleep 10; done ) &
WATCH=$!

for _ in $(seq 1 600); do
  grep -q '\[mission\] the run did\|\[mission\] the run did not\|process has died.*stale_maps_mission\|stale_maps_mission.*process has finished' "$LOG" && break
  sleep 3
done
sleep "${TAIL:-12}"
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

kill $WATCH 2>/dev/null
foreign_server > /dev/null && touch "${RAW%.*}.contaminated"
[ -f "${RAW%.*}.contaminated" ] &&
  echo "CONTAMINATED: another gzserver joined $GAZEBO_MASTER_URI during the take"

echo "raw:  $RAW"
echo "t0:   $(cat "${RAW%.*}.t0")"
grep -oE '\[(mission|crew|forklift|haul|map)\] .*' "$LOG" | cut -c1-180
