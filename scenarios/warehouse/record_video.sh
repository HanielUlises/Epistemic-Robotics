#!/usr/bin/env bash
# Renders the warehouse scenario as a film: one segment per case, the robot
# walking the route the fixed point returned and stopping where it has to
# look.
#
# The frames never touch the disk -- render_video writes raw RGB24 to stdout
# and ffmpeg encodes as it reads. The captions come from the segment table,
# because text is the one thing a hand-rolled renderer does badly.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/../../ros2_ws" && pwd)"

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

OUT="${HERE}/out"
mkdir -p "${OUT}"

FONT="${FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf}"
FONT_BOLD="${FONT_BOLD:-/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf}"
SEGMENTS="${OUT}/segments.txt"
VIDEO="${OUT}/warehouse.mp4"

read -r SIZE FPS < <(ros2 run warehouse_scenario render_video --size)
ros2 run warehouse_scenario render_video --segments "${SEGMENTS}"

# One drawtext per line per segment, switched on for that segment's seconds.
escape() { sed -e "s/'/\\\\\\\\'/g" -e 's/:/\\:/g' <<< "$1"; }

filter=""
while IFS='|' read -r window name headline caption; do
  [[ "${window}" == \#* || -z "${window}" ]] && continue
  read -r start end <<< "${window}"
  on="between(t,${start},${end})"

  filter+="drawtext=fontfile='${FONT_BOLD}':text='$(escape "${name}")'"
  filter+=":x=24:y=14:fontsize=27:fontcolor=white:enable='${on}',"

  filter+="drawtext=fontfile='${FONT}':text='$(escape "${headline}")'"
  filter+=":x=24:y=50:fontsize=17:fontcolor=0xE8B84B:enable='${on}',"

  filter+="drawtext=fontfile='${FONT}':text='$(escape "${caption}")'"
  filter+=":x=w-tw-24:y=52:fontsize=16:fontcolor=0xBFC7D5:enable='${on}',"
done < "${SEGMENTS}"

filter+="drawtext=fontfile='${FONT}':text='mu-calculus planning over what the robot knows':"
filter+="x=w-tw-24:y=18:fontsize=15:fontcolor=0x8A94A6"

ros2 run warehouse_scenario render_video \
  | ffmpeg -y -hide_banner -loglevel error \
      -f rawvideo -pix_fmt rgb24 -s "${SIZE}" -r "${FPS}" -i - \
      -vf "${filter}" \
      -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart \
      "${VIDEO}"

echo "wrote ${VIDEO}"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "${VIDEO}"
