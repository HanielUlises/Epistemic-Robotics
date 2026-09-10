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
# Composes the outage recording into the finished demonstration.
#
# Three segments, because the run has three tempos and one speed serves none of
# them. The cut itself is an instant and has to be watched at real time. The
# minute of drift that follows is a slow separation and is worth sampling. The
# return, the verdict and the reconciliation land within three seconds of each
# other and have to be given room to be read.
#
# RViz takes the wide pane, which is the reverse of the other demonstrations
# here, and for a reason particular to this one: the outage has no appearance
# in the simulator. Nothing about the warehouse changes when a radio stops
# working, and the robot carries on driving exactly as before. What can be seen
# is the belief, so the belief gets the room.
#
#     make_outage_video.sh /tmp/raw.mkv /tmp/run.log /tmp/outage.mp4
#
set -euo pipefail

RAW=${1:?usage: make_outage_video.sh RAW LOG OUT}
LOG=${2:?}
OUT=${3:?}

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# make_video.py belongs to warehouse_xl_rmf_demo and is used unchanged: the
# panes, the crops and the caption burning are the same problem here as there,
# and a second copy would drift from the first. It is found through the ROS
# index rather than by a relative path, because the two packages sit side by
# side in the source tree and one above the other once installed.
SITES="$(ros2 pkg prefix warehouse_xl_rmf_demo 2>/dev/null)/share/warehouse_xl_rmf_demo/tools"
[ -f "$SITES/make_video.py" ] || SITES="$HERE/../../warehouse_xl_rmf_demo/tools"
[ -f "$SITES/make_video.py" ] || {
  echo "make_video.py not found; is warehouse_xl_rmf_demo built?" >&2; exit 1; }
T0=$(cat "${RAW%.*}.t0")
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

FPS=30
CAPS="${OUT%.*}_captions.tsv"
python3 "$HERE/make_outage_captions.py" --log "$LOG" --t0 "$T0" --out "$CAPS"

# Both canvases whole. An earlier cut framed the corner of the floor where the
# belief separates, on the grounds that an arrow a metre long is a dozen pixels
# on a forty-metre floor; it did make the arrow bigger and it threw away the
# map being built around it, which is the other half of what there is to watch.
# The panes are the two canvases as they stand.
# Each canvas whole, less its own chrome: RViz without the Displays panel and
# the status bar, Gazebo without the toolbar and the world tree.
# Centred on the belief marker, whose pixels were measured off a frame of this
# capture rather than guessed: the canvas is mostly empty background at this
# zoom, and framing the canvas rather than its contents spends most of the pane
# on nothing.
RV_CROP=${RV_CROP:-900:506:650:367}
GZ_CROP=${GZ_CROP:-1620:911:2188:78}
RV_PANE=1120:630
GZ_PANE=1120:630

common=(--raw "$RAW" --captions "$CAPS" --fps "$FPS"
        --order rviz,gazebo
        --rviz-crop "$RV_CROP" --gazebo-crop "$GZ_CROP"
        --rviz-pane "$RV_PANE" --pane "$GZ_PANE"
        --rviz-label 'RVIZ: what r2 believes about r1'
        --gazebo-label 'GAZEBO: r1, unaware')

# Boundaries in capture seconds, properties of this recording. They are passed
# rather than detected: the instant the link falls is in the log, and the
# seconds either side of it that make a watchable shot are not.
S1_START=${S1_START:-82};  S1_TRIM=${S1_TRIM:-16};  S1_SPEED=${S1_SPEED:-1.0}
S2_START=${S2_START:-98};  S2_TRIM=${S2_TRIM:-40};  S2_SPEED=${S2_SPEED:-4.0}
S3_START=${S3_START:-138}; S3_TRIM=${S3_TRIM:-24};  S3_SPEED=${S3_SPEED:-0.9}

python3 "$SITES/make_video.py" "${common[@]}" --out "$WORK/s1.mp4" \
    --start "$S1_START" --trim "$S1_TRIM" --speed "$S1_SPEED"
python3 "$SITES/make_video.py" "${common[@]}" --out "$WORK/s2.mp4" \
    --start "$S2_START" --trim "$S2_TRIM" --speed "$S2_SPEED"
python3 "$SITES/make_video.py" "${common[@]}" --out "$WORK/s3.mp4" \
    --start "$S3_START" --trim "$S3_TRIM" --speed "$S3_SPEED"

for f in s1 s2 s3; do printf "file '%s'\n" "$WORK/$f.mp4"; done > "$WORK/list"
# Re-encoded and not stream-copied: the segments carry the same parameters and
# different presentation timestamps, and a copy leaves the later ones playing
# from the first one's clock.
ffmpeg -y -hide_banner -loglevel error -f concat -safe 0 -i "$WORK/list" \
       -c:v libx264 -preset slow -crf 23 -pix_fmt yuv420p -r "$FPS" \
       -movflags +faststart "$OUT"

printf '%s: %.1fs\n' "$OUT" \
  "$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT")"
