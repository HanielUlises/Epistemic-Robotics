#!/usr/bin/env bash
# =============================================================================
# Burns the goal and the captions over a recorded run.
#
#   tools/burn_captions.sh out/hotel-l3_suite-raw.mp4 out/hotel-l3_suite.mp4 \
#                          out/hotel-l3_suite.log <recording-start-epoch>
#
# Separate from record_demo.sh so the film can be re-cut without running the
# simulation again: the run takes six minutes and the burn takes seconds.
#
# The banner carries the goal as it is actually written, because the whole
# mission is a consequence of it and a prose paraphrase invites the reader to
# assume the interesting part was configured somewhere.
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAW="$1"; VIDEO="$2"; LOG="$3"; REC_START="$4"
SPEED="${SPEED:-4}"
SEGMENTS="${VIDEO%.mp4}-segments.txt"

FONT="${FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf}"
FONT_BOLD="${FONT_BOLD:-/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf}"
FONT_MONO="${FONT_MONO:-/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf}"

# The goal of instances/problem_2.epddl, verbatim.
GOAL="${GOAL:-(and (safe)  ([porter] (safe))  (<Kw. guest> (leak-at l2_suite)))}"
GLOSS="${GLOSS:-the incident is over  -  the porter comes to know it  -  the guest never learns which suite}"

python3 "${HERE}/captions.py" \
  --log "${LOG}" --start "${REC_START}" --speed "${SPEED}" --out "${SEGMENTS}"

escape() { sed -e "s/'/\\\\\\\\'/g" -e 's/:/\\:/g' -e 's/%/\\%/g' <<< "$1"; }

# The last few actions happen seconds apart, and captions that overlap cannot
# be read, so they are queued one after another. That queue can outlast the
# footage. Rather than cut the ending short, the final frame is held until the
# last caption has had its time.
RAW_SECONDS=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${RAW}")
PAD=$(awk -v raw="${RAW_SECONDS}" -v speed="${SPEED}" '
  !/^#/ && NF { end = $2 }
  END { need = end - raw / speed; print (need > 0 ? need + 0.5 : 0) }
' "${SEGMENTS}")

filter="setpts=PTS/${SPEED}"
if awk -v p="${PAD}" 'BEGIN{exit !(p > 0)}'; then
  echo "holding the last frame for ${PAD}s so the closing captions fit"
  filter+=",tpad=stop_mode=clone:stop_duration=${PAD}"
fi
filter+=",drawbox=x=0:y=0:w=iw:h=104:color=black@0.66:t=fill"
filter+=",drawtext=fontfile='${FONT_MONO}':text='$(escape ":goal ${GOAL}")'"
filter+=":x=32:y=18:fontsize=27:fontcolor=0xF2F2F2"
filter+=",drawtext=fontfile='${FONT}':text='$(escape "${GLOSS}")'"
filter+=":x=32:y=60:fontsize=24:fontcolor=0xB4B4B4"

while IFS='|' read -r window headline detail; do
  [[ "${window}" == \#* || -z "${window// }" ]] && continue
  read -r start end <<< "${window}"
  on="between(t,${start},${end})"

  filter+=",drawbox=x=0:y=ih-118:w=iw:h=118:color=black@0.66:t=fill:enable='${on}'"
  filter+=",drawtext=fontfile='${FONT_BOLD}':text='$(escape "$(xargs <<< "${headline}")")'"
  filter+=":x=32:y=h-100:fontsize=34:fontcolor=white:enable='${on}'"
  filter+=",drawtext=fontfile='${FONT}':text='$(escape "$(xargs <<< "${detail}")")'"
  filter+=":x=32:y=h-54:fontsize=25:fontcolor=0xC8C8C8:enable='${on}'"
done < "${SEGMENTS}"

ffmpeg -hide_banner -loglevel error -y -i "${RAW}" \
  -vf "${filter}" -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
  "${VIDEO}"

echo "video:    ${VIDEO}"
echo "captions: ${SEGMENTS}"
