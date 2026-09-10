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
# Composes a recorded multi-site mission into the finished video.
#
# Four parts: an opening card, the transit, the epistemic sequence, and a
# closing card. The opening card names the film and shows a frame of it; the
# closing card states the outcome.
#
# The two middle parts are at different speeds because the run has two
# timescales. The robots spend some eight minutes crossing the floor and the
# whole epistemic sequence -- two scans, two announcements and the verdict --
# occupies eighteen seconds at the end of it. One playback speed serves either
# the transits or the sequence: fast enough to sit through the first leaves the
# second a blur of five captions in as many seconds, and slow enough to read
# the second makes the first most of the film. So the transit is sampled and
# the sequence is played out. The caption track is offset within each segment,
# so a caption still appears at the frame its event occurred on.
#
# The closing card carries what the footage cannot: the contaminated site is
# the one no robot visited, so the frame in which the mission succeeds looks
# exactly like the frame before it. It is written from the run's own log, as is
# the still on the opening card, so neither can assert anything the recording
# does not contain.
#
#     make_sites_video.sh /tmp/raw.mkv /tmp/run.log /tmp/sites.mp4
#
set -euo pipefail

RAW=${1:?usage: make_sites_video.sh RAW LOG OUT}
LOG=${2:?}
OUT=${3:?}

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T0=$(cat "${RAW%.*}.t0")
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

FPS=30
# 1280 for the Gazebo pane and 576 for RViz. The cards span both.
CARD_W=1856
CARD_H=720

CAPS="${OUT%.*}_captions.tsv"
python3 "$HERE/make_sites_captions.py" --log "$LOG" --t0 "$T0" --out "$CAPS"

# The site the run found contaminated, and the two that were scanned, read off
# the log. A card asserting a31 over a run that found a06 would be the one
# defect in this repository that a viewer could not detect.
SITE=$(grep -oE 'contaminated at [a-z0-9]+|contaminated:=[a-z0-9]+' "$LOG" \
       | head -1 | grep -oE '[a-z][0-9]+' || true)
SITE=${SITE:-a31}
SCANNED=$(grep -oE 'applied scan_[a-z]+_[a-z0-9]+' "$LOG" \
          | sed 's/.*_//' | awk '!seen[$0]++' | paste -sd, -)
SCANNED=${SCANNED:-a17,a06}

# RViz is on the left of the capture and Gazebo on the right, the reverse of
# the single-site recording. Both crops were measured from a frame of this
# capture. The RViz one cannot be the single-site crop shifted, because this
# configuration opens with the Displays panel closed and the canvas is the
# width of the half screen.
#
# The Gazebo crop is the whole 3D canvas at 16:9, less the toolbar and the
# window border. It is not a window onto part of the canvas, because it does
# not need to be: the camera follows the scout, so the robot holds its position
# and its size in the canvas whatever the aisle does, and there is nothing to
# hunt for. Both numbers that decide the framing are set at the camera and not
# recovered here -- the distance, which at 1.3 m fills the frame with robot and
# at 4.6 m reduces it to a speck, and the tilt, which at 4.5 degrees leaves the
# robot at 87 per cent of the frame height and drops it out of the crop at the
# first turn.
#
# The roadmap is 38.8 m across and 60.0 m deep, so the region holding all of it
# is taller than it is wide; giving it the same width as the Gazebo pane would
# be half a pane of empty floor.
GZ_CROP=${GZ_CROP:-1620:911:2188:78}
RV_CROP=${RV_CROP:-800:1030:560:30}
GZ_PANE=1280:720
RV_PANE=576:720

common=(--raw "$RAW" --captions "$CAPS" --fps "$FPS"
        --order rviz,gazebo
        --gazebo-crop "$GZ_CROP"
        --rviz-crop "$RV_CROP"
        --pane "$GZ_PANE" --rviz-pane "$RV_PANE"
        # Short, and without commas. Each label is drawn over its own pane and
        # the RViz pane is 576 px wide; a longer line runs under the Gazebo
        # label and the two read as one sentence. Commas are stripped by the
        # filter-graph escaping, which turns a clause into a gap.
        --gazebo-label 'GAZEBO: chase camera on the scout'
        --rviz-label 'RVIZ: the roadmap')

# Segment boundaries are given in capture seconds and are properties of this
# recording. They are passed rather than detected: an arrival is legible in the
# log, and the seconds either side of it that make a watchable shot are not.
SEG1_START=${SEG1_START:-290}; SEG1_TRIM=${SEG1_TRIM:-96}; SEG1_SPEED=${SEG1_SPEED:-6.0}
# Below one. The five events of the sequence fall in seventeen seconds, and at
# any speed at or above real time two of the captions are on screen for less
# than two seconds each.
SEG2_START=${SEG2_START:-396}; SEG2_TRIM=${SEG2_TRIM:-20}; SEG2_SPEED=${SEG2_SPEED:-0.6}

python3 "$HERE/make_video.py" "${common[@]}" \
    --out "$WORK/seg1.mp4" --start "$SEG1_START" --trim "$SEG1_TRIM" \
    --speed "$SEG1_SPEED"
python3 "$HERE/make_video.py" "${common[@]}" \
    --out "$WORK/seg2.mp4" --start "$SEG2_START" --trim "$SEG2_TRIM" \
    --speed "$SEG2_SPEED"

# The still on the opening card is cut from this recording, with the geometry
# the film is composed at, so the card previews the frame that follows it and
# cannot show some other run. It is taken without captions or pane labels: the
# card carries a title already, and two sets of lettering over one frame is one
# too many.
SHOT_AT=${SHOT_AT:-400}
ffmpeg -y -hide_banner -loglevel error -ss "$SHOT_AT" -i "$RAW" -frames:v 1 \
    -filter_complex \
    "[0:v]crop=$RV_CROP,scale=$RV_PANE[rv];[0:v]crop=$GZ_CROP,scale=$GZ_PANE[gz];[rv][gz]hstack=inputs=2" \
    "$WORK/shot.png"

python3 "$HERE/make_sites_cards.py" --outdir "$WORK" --site "$SITE" \
        --scanned "$SCANNED" --shot "$WORK/shot.png"

# A card is a still. It is given a fade at each end so it does not cut hard
# into moving footage, and the same frame rate and pixel format as the
# segments, because the concat demuxer joins streams by copying them and will
# not reconcile two that disagree.
card() {
  local png="$1" out="$2" secs="$3"
  ffmpeg -y -hide_banner -loglevel error -loop 1 -t "$secs" -i "$png" \
         -vf "fps=$FPS,format=yuv420p,fade=t=in:st=0:d=0.5,fade=t=out:st=$(echo "$secs - 0.6" | bc):d=0.6" \
         -c:v libx264 -preset slow -crf 20 -r "$FPS" "$out"
}
card "$WORK/card_open.png"  "$WORK/open.mp4"  4
card "$WORK/card_close.png" "$WORK/close.mp4" 7

for f in open seg1 seg2 close; do printf "file '%s'\n" "$WORK/$f.mp4"; done > "$WORK/list"
# Re-encoded rather than stream-copied. The parts are encoded with the same
# parameters but different presentation timestamps, and a copy of all four into
# one container leaves each playing from the first one's clock.
ffmpeg -y -hide_banner -loglevel error -f concat -safe 0 -i "$WORK/list" \
       -c:v libx264 -preset slow -crf 23 -pix_fmt yuv420p -r "$FPS" \
       -movflags +faststart "$OUT"

printf '%s: %.1fs  (%s contaminated, scanned %s)\n' "$OUT" \
  "$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT")" \
  "$SITE" "$SCANNED"
