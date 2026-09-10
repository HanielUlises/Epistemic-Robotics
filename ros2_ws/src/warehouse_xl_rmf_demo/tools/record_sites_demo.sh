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
# Records a multi-site mission, and the epistemic models it passes through.
#
# RViz occupies the left half of the capture and Gazebo the right. The
# single-site recording is the other way round; this one puts the roadmap first
# because it is what the mission is about -- two robots sent to two of three
# named places -- and the simulator second, as the evidence that they went.
#
# The capture is of an Xvfb screen carrying Gazebo and RViz and nothing else,
# so a full-screen grab cannot pick up whatever is open on the real desktop.
# There is no window manager on that display, so neither window can be resized
# after it maps: X honours the request and Qt never relays out. Both are sized
# and placed before they open, Gazebo from ~/.gazebo/gui.ini and RViz from a
# copy of this package's own configuration.
#
# The same run writes the epistemic model after each product update.
# `scripts/record_models.py` subscribes to the state topic, which carries the
# whole structure -- worlds, valuations, designated set, one relation per agent
# -- and one recording therefore yields both the film and the figures.
#
#     record_sites_demo.sh a31 /tmp/sites_raw.mkv /tmp/sites_run.log /tmp/models
#
set +u

VARIANT=${1:-a31}
RAW=${2:-/tmp/warehouse_xl_sites_raw.mkv}
LOG=${3:-/tmp/warehouse_xl_sites_run.log}
MODELS=${4:-/tmp/warehouse_xl_sites_models}
# The robot the camera follows. r2 is the scout: it performs the first sensing
# action, at a17, and it is the agent whose announcement the relay acts on.
# `chase_camera` drives the Gazebo GUI camera over gazebo transport; the world
# file supplies only the pose the view opens on.
FOLLOW=${FOLLOW:-r2}
DISP=${DISP:-:99}
WIDTH=3840
HEIGHT=1080

source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/rmf_ws/install/setup.bash"
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
SHARE=$(ros2 pkg prefix warehouse_xl_rmf_demo)/share/warehouse_xl_rmf_demo

export DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 PYTHONNOUSERSITE=1

# Anything surviving from a previous run keeps the Gazebo port, and the next
# launch then attaches to the world already loaded and reports the other
# variant's answer. This has to be exhaustive.
kill_stack() {
  for p in gzserver gzclient rviz2 fleet_adapter fleet_manager plansys2_node \
           epistemic_state site_perception scan_perception record_models \
           chase_camera \
           schedule_visualizer building_map_server rmf_traffic_schedule \
           rmf_action_node radio mission_check; do
    pkill -9 -x "$p" 2>/dev/null
  done
  pkill -9 -f 'survey_sites_launch' 2>/dev/null
  sleep 6
}
kill_stack

pgrep -x Xvfb > /dev/null || {
  Xvfb "$DISP" -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset \
      > /tmp/xvfb_sites.log 2>&1 &
  sleep 3
}

# Gazebo's window size and position are preferences it reads at startup and
# there is no command line for either. It goes on the right half.
mkdir -p "$HOME/.gazebo"
python3 - "$HOME/.gazebo/gui.ini" <<'PY'
import configparser, sys
path = sys.argv[1]
cfg = configparser.ConfigParser()
cfg.read(path)
if not cfg.has_section('geometry'):
    cfg.add_section('geometry')
cfg['geometry'].update({'x': '1920', 'y': '0', 'width': '1920', 'height': '1080'})
with open(path, 'w') as fh:
    cfg.write(fh)
PY

# RViz goes on the left half, which is where the package's own configuration
# already puts it, so the file is used unaltered.
RVIZ="$SHARE/config/warehouse_xl.rviz"

# The worlds are built here so the recording cannot be of a world that has
# drifted from the one the demo ships. Each world carries the follow settings
# the camera adopts once it is told which robot to follow, and an initial pose
# looking down the aisle at a17, which is what is on screen until then.
bash "$SHARE/tools/make_site_worlds.sh" /tmp > /dev/null || exit 1
WORLD=/tmp/warehouse_xl_${VARIANT}.world
[ -f "$WORLD" ] || { echo "no world for variant $VARIANT"; exit 1; }

mkdir -p "$MODELS"

setsid ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py \
    contaminated:="$VARIANT" world:="$WORLD" rviz_config:="$RVIZ" \
    headless:=false shutdown:=false record_models:=true \
    models_dir:="$MODELS" policy_out:="${MODELS}/policy.json" \
    > "$LOG" 2>&1 &
LAUNCH=$!
trap 'kill $CHASE 2>/dev/null; pkill -x chase_camera 2>/dev/null; kill -KILL -$LAUNCH 2>/dev/null; kill_stack' EXIT INT TERM

# Both windows have to exist before the capture is worth starting. Gazebo's own
# window does not exist for the first minute or two -- there is a splash screen
# where it will be -- so a loop that gives up early gives up before the window
# it is waiting for has been created.
for _ in $(seq 1 150); do
  [ "$(xdotool search --name '^(Gazebo|.*RViz)$' 2>/dev/null | wc -l)" -ge 2 ] && break
  sleep 2
done

# The camera follows the scout, and cannot be told to until the scout exists:
# the robots are spawned on a timer well after the world loads. `chase_camera`
# tolerates being started early -- it publishes nothing until a pose for its
# model arrives -- but starting it after the spawn saves it from holding the
# camera at the world origin while it waits.
for _ in $(seq 1 120); do
  grep -qi "successfully spawned entity \[$FOLLOW\]" "$LOG" && break
  sleep 2
done
sleep 6

# Supervised, and logged to its own file. The camera is driven by a process
# that has to outlive an eight-minute run, and the first attempt did not: it
# stopped somewhere in the sixth minute, the view stayed pointing at the aisle
# the robot was in when it stopped, and the robot drove out of frame and never
# came back. Nothing said so, because the launch had the run log open with a
# fixed offset and its writes overwrote whatever the camera appended.
#
# So its output goes somewhere it cannot be clobbered, and a supervisor
# restarts it. A restart costs one jump of the view, which is visible and
# recoverable; the alternative is a recording of an empty aisle.
CHASE_LOG="${LOG%.*}_camera.log"
: > "$CHASE_LOG"
chase_forever() {
  local bin
  bin="$(ros2 pkg prefix warehouse_xl_rmf_demo)/lib/warehouse_xl_rmf_demo/chase_camera"
  while :; do
    "$bin" --model "$FOLLOW" --distance 2.60 --height 1.25 --look 0.30 \
           --tau 0.50 --rate 25 >> "$CHASE_LOG" 2>&1
    echo "chase_camera exited with $?; restarting" >> "$CHASE_LOG"
    sleep 2
  done
}
chase_forever &
CHASE=$!

# Long enough for the camera to fly from the pose the world gave it to the
# robot, so the recording does not open on the tail of that move.
sleep 10
date +%s > "${RAW%.*}.t0"

ffmpeg -y -hide_banner -nostdin -loglevel error -f x11grab -framerate 12 \
       -video_size ${WIDTH}x${HEIGHT} -i "$DISP+0,0" \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$RAW" \
       < /dev/null &
FF=$!

for _ in $(seq 1 400); do
  grep -q 'came out as specified\|checks did not pass\|mission failed' "$LOG" && break
  sleep 3
done
sleep 10
kill -INT $FF 2>/dev/null
wait $FF 2>/dev/null

echo "raw:    $RAW"
echo "t0:     $(cat "${RAW%.*}.t0")"
echo "models: $(ls "$MODELS"/model_*.json 2>/dev/null | wc -l) recorded"
echo "camera: $(grep -c restarting "$CHASE_LOG" 2>/dev/null || echo 0) restart(s), \
$(grep -oE '[0-9]+ poses' "$CHASE_LOG" 2>/dev/null | tail -1)"
grep -oE 'r[0-9] at a[0-9]+ .*|applied [^ ]+.*designated|came out as specified.*' "$LOG"
