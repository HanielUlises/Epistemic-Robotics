#!/usr/bin/env bash
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
# Records one floor of the coordinated attack: RViz on the left half of a
# virtual display, Gazebo on the right, and the mission's own log beside the
# capture.
#
#     record_demo.sh radio  s1 /tmp/ca_radio.mkv  /tmp/ca_radio.log
#     record_demo.sh beacon s1 /tmp/ca_beacon.mkv /tmp/ca_beacon.log
#
# The display is an Xvfb screen carrying the two windows and nothing else.
# There is no window manager on it, so both windows are placed before they
# open: Gazebo through a gui.ini of its own, named by GAZEBO_GUI_INI_FILE so
# the user's own Gazebo window is not moved, and RViz through the package's
# configuration.
#
# The Gazebo camera is warehouse_xl_rmf_demo's chase_camera, re-reading the
# file camera_director writes the current shot to. The capture's start time is
# written beside it as <raw>.t0, so the captions can be timed from the log.
set +u

FLOOR=${1:-beacon}
ORDER=${2:-s1}
RAW=${3:-/tmp/coordinated_attack_${FLOOR}.mkv}
LOG=${4:-/tmp/coordinated_attack_${FLOOR}.log}
DISP=${DISP:-:98}
WIDTH=3840
HEIGHT=1080
FPS=${FPS:-12}
FOLLOW=/tmp/coordinated_attack_follow

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/rmf_ws/install/setup.bash" 2>/dev/null
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1

# By exact name, and the launch by a pattern this script's own command line
# does not contain, so a pkill -f cannot match its caller.
kill_stack() {
  for p in gzserver gzclient rviz2 plansys2_node robot_state_publisher \
           static_transform_publisher read_order_action go_view_action radio_action \
           signal_action lift_action coordinated_attack_mission chase_camera; do
    pkill -9 -x "$p" 2>/dev/null
  done
  pkill -9 -f "ros2 launch coordinated_attack""_demo" 2>/dev/null
  pkill -9 -f "knowledge_view""\.py" 2>/dev/null
  pkill -9 -f "camera_director""\.py" 2>/dev/null
  sleep 3
}
kill_stack

pgrep -f "Xvfb $DISP " > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_coordinated_attack.log 2>&1 &
  sleep 3
}

GUI_INI=/tmp/coordinated_attack_gui.ini
cat > "$GUI_INI" <<EOF
[geometry]
x = 1920
y = 0
width = 1920
height = 1080
EOF
export GAZEBO_GUI_INI_FILE=$GUI_INI

# The opening shot, until the first performer names one.
python3 - "$FOLLOW" <<'PY'
import sys
sys.path.insert(0, __import__('os').path.join(
    __import__('ament_index_python.packages', fromlist=['x']).get_package_share_directory(
        'coordinated_attack_demo'), 'tools'))
import layout as L
open(sys.argv[1], 'w').write(L.shot_text(L.OPENING_SHOT) + '\n')
PY

setsid ros2 launch coordinated_attack_demo coordinated_attack_launch.py \
    floor:="$FLOOR" order:="$ORDER" gui:=true rviz:=true camera:=true \
    follow_file:="$FOLLOW" hold:="${HOLD:-10}" start_after:=45 shutdown:=false \
    policy_out:="${RAW%.*}_policy.json" > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill $CHASE 2>/dev/null; pkill -x chase_camera 2>/dev/null; kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

for _ in $(seq 1 150); do
  [ "$(xdotool search --name '^(Gazebo|.*RViz.*)$' 2>/dev/null | wc -l)" -ge 2 ] && break
  sleep 2
done
for _ in $(seq 1 90); do
  grep -q "uccessfully spawned entity \[r2\]" "$LOG" && break
  sleep 2
done
sleep 5

CHASE_LOG="${LOG%.*}_camera.log"
: > "$CHASE_LOG"
chase_forever() {
  local bin
  bin="$(ros2 pkg prefix warehouse_xl_rmf_demo)/lib/warehouse_xl_rmf_demo/chase_camera"
  while :; do
    "$bin" --model r1 --follow-file "$FOLLOW" --distance "${CAM_DISTANCE:-1.4}" \
           --height "${CAM_HEIGHT:-0.85}" --look 0.25 --tau 0.5 --rate 25 \
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

for _ in $(seq 1 500); do
  grep -q '\[mission\] mission complete\|\[mission\] mission failed' "$LOG" && break
  sleep 3
done
sleep "${TAIL:-12}"
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw:  $RAW"
echo "t0:   $(cat "${RAW%.*}.t0")"
grep -oE '\[(mission|order|view|radio|signal|lift|knows)\] .*' "$LOG" | cut -c1-170
