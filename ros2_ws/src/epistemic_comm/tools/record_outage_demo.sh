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
# Records the link outage on a virtual display.
#
# RViz on the left and Gazebo on the right. RViz is the one that matters here:
# the outage has no appearance in the simulator, because nothing about the
# world changes when a radio stops working. What can be seen is the belief --
# an arrow where r2 thinks r1 is, and a disc the size of its uncertainty --
# separating from the robot while the link is down, growing, turning red when
# it passes the threshold past which RF-05 says it should not be believed, and
# snapping back the instant the link returns.
#
#     record_outage_demo.sh /tmp/raw.mkv /tmp/run.log /tmp/out
#
set +u

RAW=${1:-/tmp/link_outage_raw.mkv}
LOG=${2:-/tmp/link_outage_run.log}
OUT=${3:-/tmp/link_outage}
T_DISC=${T_DISC:-70}
T_RECON=${T_RECON:-130}
DISP=${DISP:-:99}
WIDTH=3840
HEIGHT=1080

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash" 2>/dev/null
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1
mkdir -p "$OUT"

# Anything left from a previous run keeps Gazebo's port, and the next launch
# then attaches to the world already loaded.
# Its own pid and its parent are excluded, and so is any shell. This script's
# own command line contains the string `epistemic_comm`, so a kill_stack that
# matched on that alone killed the script that called it, leaving an Xvfb, no
# launch and an empty log.
kill_stack() {
  local self=$$ parent=$PPID
  for pid in $(pgrep -f 'gzserver|gzclient|launch_params_|epistemic_comm|warehouse_demo|slam_toolbox|rviz2' 2>/dev/null); do
    [ "$pid" = "$self" ] && continue
    [ "$pid" = "$parent" ] && continue
    case "$(ps -p "$pid" -o comm= 2>/dev/null)" in
      bash|sh|zsh|ps) continue ;;
    esac
    kill -9 "$pid" 2>/dev/null
  done
  sleep 6
}
kill_stack

pgrep -x Xvfb > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_outage.log 2>&1 &
  sleep 3
}

# Gazebo takes the right half. Its window size and position are preferences it
# reads at startup and there is no command line for either.
mkdir -p "$HOME/.gazebo"
python3 - "$HOME/.gazebo/gui.ini" <<'PY'
import configparser, sys
cfg = configparser.ConfigParser()
cfg.read(sys.argv[1])
if not cfg.has_section('geometry'):
    cfg.add_section('geometry')
cfg['geometry'].update({'x': '1920', 'y': '0', 'width': '1920', 'height': '1080'})
with open(sys.argv[1], 'w') as fh:
    cfg.write(fh)
PY

setsid ros2 launch epistemic_comm link_outage_launch.py \
    gazebo_gui:=true rviz:=true t_disc:="$T_DISC" t_recon:="$T_RECON" \
    out_dir:="$OUT" scenario:=demo > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

# Both windows have to exist before the capture is worth starting, and
# Gazebo's does not exist for the first minute or two: there is a splash
# screen where it will be.
for _ in $(seq 1 150); do
  [ "$(xdotool search --name '^(Gazebo|.*RViz)$' 2>/dev/null | wc -l)" -ge 2 ] && break
  sleep 2
done
sleep 8
date +%s > "${RAW%.*}.t0"

ffmpeg -y -hide_banner -nostdin -loglevel error -f x11grab -framerate 12 \
       -video_size ${WIDTH}x${HEIGHT} -i "$DISP+0,0" \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$RAW" \
       < /dev/null &
FF=$!

for _ in $(seq 1 400); do
  grep -q 'RF-05 acceptance criterion' "$LOG" && break
  sleep 3
done
sleep 12
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw: $RAW"
echo "t0:  $(cat "${RAW%.*}.t0")"
grep -oE 'is DOWN at.*|is UP at.*|[0-9]+ samples over.*|acceptance criterion.*|a learned.*' "$LOG"
