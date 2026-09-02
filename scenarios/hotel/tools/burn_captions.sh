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

# Where each pane's drawn content sits in the grab, as ffmpeg crop rects.
#
# Without a window manager Qt never gets a proper resize event, so Gazebo keeps
# its own idea of how big it is however large its window is made, and the rest
# of that window stays black. Rather than publish the black, each pane is cut
# out at the size it actually drew and the two are scaled to a common height
# and set side by side.
PANE_LEFT="${PANE_LEFT:-}"
PANE_RIGHT="${PANE_RIGHT:-}"
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

# The grab runs a few seconds past the end of the mission, and in those seconds
# Gazebo and RViz are shutting down and going black. Holding the *last* frame
# would freeze on that. The footage is cut at the moment the mission reported
# done, so what the closing captions sit on is the finished building.
END_STAMP=$(grep -aoE '\[[0-9]+\.[0-9]+\].*(mission complete|mission failed)' "${LOG}" \
            | tail -1 | sed -E 's/^\[([0-9.]+)\].*/\1/')
TRIM=""
if [[ -n "${END_STAMP}" ]]; then
  TRIM=$(awk -v e="${END_STAMP}" -v s="${REC_START}" -v raw="${RAW_SECONDS}" \
    'BEGIN { t = e - s; if (t > 0 && t < raw) print t }')
fi
if [[ -n "${TRIM}" ]]; then
  echo "cutting the grab at ${TRIM}s, where the mission reported done"
  RAW_SECONDS="${TRIM}"
fi
PAD=$(awk -v raw="${RAW_SECONDS}" -v speed="${SPEED}" '
  !/^#/ && NF { end = $2 }
  END { need = end - raw / speed; print (need > 0 ? need + 0.5 : 0) }
' "${SEGMENTS}")

if [[ -n "${PANE_LEFT}" && -n "${PANE_RIGHT}" ]]; then
  echo "composing ${PANE_LEFT} and ${PANE_RIGHT} side by side"
  compose="[0:v]crop=${PANE_LEFT},scale=-2:540[l];"
  compose+="[0:v]crop=${PANE_RIGHT},scale=-2:540[r];"
  compose+="[l][r]hstack=inputs=2,scale=1920:-2,"
else
  compose=""
fi

filter="${compose}setpts=PTS/${SPEED}"
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

# Cutting two panes out of one grab reads the input twice, which makes the
# graph a complex one; -vf only takes graphs with a single input.
if [[ -n "${compose}" ]]; then
  ffmpeg -hide_banner -loglevel error -y ${TRIM:+-t "${TRIM}"} -i "${RAW}" \
    -filter_complex "${filter}[out]" -map "[out]" \
    -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
    "${VIDEO}"
else
  ffmpeg -hide_banner -loglevel error -y ${TRIM:+-t "${TRIM}"} -i "${RAW}" \
    -vf "${filter}" -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
    "${VIDEO}"
fi

echo "video:    ${VIDEO}"
echo "captions: ${SEGMENTS}"
