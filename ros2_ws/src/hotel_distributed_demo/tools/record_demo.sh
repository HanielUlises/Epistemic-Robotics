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
# Records one fleet in the hotel: Gazebo looking straight down at the floor
# the porter is on, and the run's own log beside the capture.
#
#     record_demo.sh epistemic L3_room1 /tmp/hd_epistemic.mkv /tmp/hd_epistemic.log
#     record_demo.sh filter    L3_room1 /tmp/hd_filter.mkv    /tmp/hd_filter.log
#
# As the hotel's scenarios/hotel/record_demo.sh, whose reasons hold here: a
# private Xvfb display on which only Gazebo is drawn, the window placed and
# measured, the floor toggled by the world's own buttons, and the capture's
# start time written beside it as <raw>.t0, so that the board and captions
# can be timed from the log. Nothing else is filmed live: the knowledge board
# is drawn afterwards from the [knows] lines of the log.
set +u

FLEET=${1:-epistemic}
LEAK=${2:-L3_room1}
RAW=${3:-/tmp/hotel_distributed_${FLEET}.mkv}
LOG=${4:-/tmp/hotel_distributed_${FLEET}.log}
DISP=${DISP:-:96}
WIDTH=1920
HEIGHT=1080
FPS=${FPS:-12}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/rmf_ws/install/setup.bash"
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe PYTHONNOUSERSITE=1

# By exact name, and the launch by a pattern this script's own command line
# does not contain, so a kill cannot match its caller. Process names are cut
# at fifteen characters, so the RMF nodes are found by their paths.
kill_stack() {
  for p in gzserver gzclient plansys2_node epistemic_state_node rmf_action_node \
           door_supervisor lift_supervisor mock_docker hotel_distributed_mission; do
    pkill -9 -x "$p" 2>/dev/null
  done
  for pid in $(ps -eo pid,args | awk '/rmf_traffic_ros2|rmf_fleet_adapter\/|rmf_task_ros2|rmf_demos_fleet_adapter|rmf_visualization|hotel_crew\.py|knowledge_view\.py|follow_floors\.py|hotel_distributed_launch/ && !/awk/ {print $1}'); do
    kill -9 "$pid" 2>/dev/null
  done
  sleep 3
}
kill_stack

pgrep -f "Xvfb $DISP " > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_hotel_distributed.log 2>&1 &
  sleep 3
}

GUI_INI=/tmp/hotel_distributed_gui.ini
cat > "$GUI_INI" <<INI
[geometry]
x = 0
y = 0
width = ${WIDTH}
height = ${HEIGHT}
INI
export GAZEBO_GUI_INI_FILE=$GUI_INI

setsid ros2 launch hotel_distributed_demo hotel_distributed_launch.py \
    fleet:="$FLEET" leak:="$LEAK" gui:=true hold:="${HOLD:-6}" \
    policy_out:="${RAW%.*}_policy.json" > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill $FOLLOW 2>/dev/null; kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

for _ in $(seq 1 150); do
  xdotool search --name '^Gazebo$' > /dev/null 2>&1 && break
  sleep 2
done
sleep 8
ID=$(xdotool search --name '^Gazebo$' | tail -1)
xdotool windowmove "$ID" 0 0 windowsize "$ID" $WIDTH $HEIGHT 2>/dev/null
sleep 3

python3 "$HERE/follow_floors.py" --display "$DISP" > "${LOG%.*}_floors.log" 2>&1 &
FOLLOW=$!

date +%s.%N > "${RAW%.*}.t0"
ffmpeg -y -hide_banner -nostdin -loglevel error -f x11grab -draw_mouse 0 -framerate "$FPS" \
       -video_size ${WIDTH}x${HEIGHT} -i "$DISP+0,0" \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$RAW" < /dev/null &
FF=$!

for _ in $(seq 1 1200); do
  grep -aq '\[mission\] mission complete\|\[mission\] mission failed' "$LOG" && break
  sleep 3
done
sleep "${TAIL:-10}"
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw:  $RAW"
echo "t0:   $(cat "${RAW%.*}.t0")"
grep -aoE '\[(mission|crew|policy)\] .*' "$LOG" | cut -c1-170
