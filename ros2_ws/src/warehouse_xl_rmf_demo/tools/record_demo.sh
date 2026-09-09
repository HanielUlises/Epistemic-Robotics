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
# Records a mission on a virtual display.
#
# The capture is of an Xvfb screen that carries Gazebo and RViz and nothing
# else, so a full-screen grab cannot pick up whatever happens to be open on the
# real desktop. An earlier recording was made by grabbing the physical screen
# and it filmed the terminal along with the simulation.
#
# There is no window manager on that display, so neither window can be resized
# after it maps: X honours the request and Qt never relays out. Both are sized
# and placed before they open instead, Gazebo from ~/.gazebo/gui.ini and RViz
# from a copy of the demo's own RViz configuration.
#
#     record_demo.sh dirty /tmp/raw.mkv /tmp/run.log
#
set +u

VARIANT=${1:-dirty}
RAW=${2:-/tmp/warehouse_xl_raw.mkv}
LOG=${3:-/tmp/warehouse_xl_run.log}
DISP=${DISP:-:99}
WIDTH=3840
HEIGHT=1080

source /opt/ros/humble/setup.bash
source "$HOME/rmf_ws/install/setup.bash"
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
SHARE=$(ros2 pkg prefix warehouse_xl_rmf_demo)/share/warehouse_xl_rmf_demo

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1

# Anything surviving from a previous run keeps the Gazebo port, and the next
# launch then attaches to the world already loaded and reports the other
# variant's answer. This has to be exhaustive.
kill_stack() {
  for p in gzserver gzclient rviz2 fleet_adapter fleet_manager plansys2_node \
           epistemic_state scan_perception schedule_visualizer \
           building_map_server rmf_traffic_schedule rmf_action_node; do
    pkill -9 -x "$p" 2>/dev/null
  done
  pkill -9 -f 'survey_xl_launch' 2>/dev/null
  sleep 6
}
kill_stack

pgrep -x Xvfb > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb.log 2>&1 &
  sleep 3
}

# Gazebo's window size is a preference it reads at startup and there is no
# command line for it.
mkdir -p "$HOME/.gazebo"
python3 - "$HOME/.gazebo/gui.ini" <<'PY'
import configparser, sys
path = sys.argv[1]
cfg = configparser.ConfigParser()
cfg.read(path)
if not cfg.has_section('geometry'):
    cfg.add_section('geometry')
cfg['geometry'].update({'x': '0', 'y': '0', 'width': '1920', 'height': '1080'})
with open(path, 'w') as fh:
    cfg.write(fh)
PY

# RViz goes on the right half. Its saved position is left alone in the package,
# where it would put the window off the edge of an ordinary single screen.
RVIZ=/tmp/warehouse_xl_record.rviz
python3 - "$SHARE/config/warehouse_xl.rviz" "$RVIZ" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src).read().replace('  X: 0\n  Y: 0\n', '  X: 1920\n  Y: 0\n')
open(dst, 'w').write(text)
PY

# The two variants are built here so the recording cannot be of a world that
# has drifted from the one the demo ships. They differ by the pallet and by
# nothing else. The camera looks across aisle_07 from the north: the scout
# approaches from the south, so it drives towards the camera and grows in
# frame, and the pallet it is sent to look at stands beside it throughout.
WORLD=/tmp/warehouse_xl_${VARIANT}_cam.world
PALLET=()
[ "$VARIANT" = dirty ] && PALLET=(--pallet -4.15,2.0,0)
python3 "$SHARE/tools/with_camera.py" \
    --world "$SHARE/worlds/warehouse_xl.world" \
    --pose "-3.45 6.6 2.6 0 0.454 -1.6239" \
    "${PALLET[@]}" --out "$WORLD" || exit 1

setsid ros2 launch warehouse_xl_rmf_demo survey_xl_launch.py \
    world:="$WORLD" rviz_config:="$RVIZ" headless:=false shutdown:=false \
    > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

for _ in $(seq 1 90); do
  [ "$(xdotool search --name '^(Gazebo|.*RViz)$' 2>/dev/null | wc -l)" -ge 2 ] && break
  sleep 2
done
sleep 10
date +%s > "${RAW%.*}.t0"

ffmpeg -y -hide_banner -loglevel error -f x11grab -framerate 12 \
       -video_size ${WIDTH}x${HEIGHT} -i "$DISP+0,0" \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$RAW" &
FF=$!

for _ in $(seq 1 200); do
  grep -q 'came out as specified' "$LOG" && break
  sleep 3
done
sleep 8
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw:  $RAW"
echo "t0:   $(cat "${RAW%.*}.t0")"
grep -oE 'at the site.*|applied relay-[a-z]*_relay_scout.*|came out as specified.*' "$LOG"
