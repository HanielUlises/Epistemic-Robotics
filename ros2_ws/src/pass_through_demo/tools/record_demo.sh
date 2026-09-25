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
# Records the pass-through mission: Gazebo on the left half of a virtual
# display, RViz on the right, and the mission's own log beside the capture.
#
#     record_demo.sh t2 /tmp/pt_raw.mkv /tmp/pt_run.log
#
# The display is an Xvfb screen carrying the two windows and nothing else, so
# a full-screen grab cannot pick up the real desktop. There is no window
# manager on it, so neither window can be moved or resized after it maps: both
# are placed before they open. Gazebo reads its geometry from a gui.ini, which
# is written to a file of its own and named through GAZEBO_GUI_INI_FILE rather
# than into ~/.gazebo, where it would move the user's own Gazebo window too.
# RViz takes its geometry from the package's configuration.
#
# The Gazebo camera follows whichever robot the policy has set to work.
# camera_director writes that robot's name to a file; chase_camera, from
# warehouse_xl_rmf_demo, re-reads it and cuts to the robot.
#
# The capture's start time is written beside it as <raw>.t0, so the captions
# can be timed from the log.
set +u

OPEN=${1:-t2}
RAW=${2:-/tmp/pass_through_raw.mkv}
LOG=${3:-/tmp/pass_through_run.log}
DISP=${DISP:-:99}
WIDTH=3840
HEIGHT=1080
FPS=${FPS:-12}
FOLLOW=/tmp/pass_through_follow

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/rmf_ws/install/setup.bash" 2>/dev/null
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1

# By exact name, and the launch by a pattern this script's own command line
# does not contain: a pkill -f matching its caller kills the recording before
# it starts and leaves an empty log that looks like a launch still coming up.
kill_stack() {
  for p in gzserver gzclient rviz2 plansys2_node epistemic_state_node \
           async_slam_toolbox_node robot_state_publisher knowledge_map \
           survey_action share_action cross_action pass_through_mission chase_camera; do
    pkill -9 -x "$p" 2>/dev/null
  done
  pkill -9 -f "ros2 launch pass_through""_demo" 2>/dev/null
  pkill -9 -f "knowledge_view""\.py" 2>/dev/null
  pkill -9 -f "camera_director""\.py" 2>/dev/null
  sleep 3
}
kill_stack

pgrep -x Xvfb > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_pass_through.log 2>&1 &
  sleep 3
}

GUI_INI=/tmp/pass_through_gui.ini
cat > "$GUI_INI" <<EOF
[geometry]
x = 0
y = 0
width = 1920
height = 1080
EOF
export GAZEBO_GUI_INI_FILE=$GUI_INI

echo r2 > "$FOLLOW"

# hold: the mission waits after the planning system is up, so the capture opens
# on three robots standing still with nothing known.
setsid ros2 launch pass_through_demo pass_through_launch.py \
    open:="$OPEN" gui:=true rviz:=true camera:=true follow_file:="$FOLLOW" \
    hold:="${HOLD:-12}" start_after:=45 shutdown:=false \
    policy_out:="${RAW%.*}_policy.json" > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill $CHASE 2>/dev/null; pkill -x chase_camera 2>/dev/null; kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

# Both windows. Gazebo's does not exist for the first minute or so.
for _ in $(seq 1 150); do
  [ "$(xdotool search --name '^(Gazebo|.*RViz.*)$' 2>/dev/null | wc -l)" -ge 2 ] && break
  sleep 2
done
for _ in $(seq 1 90); do
  grep -q "successfully spawned entity \[r3\]\|Successfully spawned entity \[r3\]" "$LOG" && break
  sleep 2
done
sleep 5

# Supervised, and logged to a file of its own: the launch has the run log open
# at a fixed offset and would overwrite whatever the camera appended to it.
CHASE_LOG="${LOG%.*}_camera.log"
: > "$CHASE_LOG"
chase_forever() {
  local bin
  bin="$(ros2 pkg prefix warehouse_xl_rmf_demo)/lib/warehouse_xl_rmf_demo/chase_camera"
  while :; do
    "$bin" --model r2 --follow-file "$FOLLOW" --distance "${CAM_DISTANCE:-2.7}" \
           --height "${CAM_HEIGHT:-1.5}" --look 0.25 --tau 0.5 --rate 25 \
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

for _ in $(seq 1 400); do
  grep -q '\[mission\] mission complete\|\[mission\] mission failed' "$LOG" && break
  sleep 3
done
# The verdict and the collapsed model, on screen for a few seconds.
sleep "${TAIL:-12}"
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw:  $RAW"
echo "t0:   $(cat "${RAW%.*}.t0")"
grep -oE '\[(mission|survey|share|cross|reach)\] .*' "$LOG" | grep -v '\[policy\]' | cut -c1-160
