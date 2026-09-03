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

# Each pane is scaled to this height before the two are stacked, so a pair of
# 16:9 panes gives a 3840x1080 film, the shape the rest of the demos on the
# site are in. The caption geometry below was authored against a 1920-wide
# frame and U carries it up to whatever height this is set to.
PANE_HEIGHT="${PANE_HEIGHT:-1080}"
U=$(( PANE_HEIGHT / 540 ))
(( U < 1 )) && U=1
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
GOAL="${GOAL:-(and (delivered)  ([Kw. r2] (pallet-at bay2)))}"
GLOSS="${GLOSS:-the pallet reaches receiving  -  and r2 comes to know which aisle held it, without going}"

# How long a caption stays up when nothing follows it soon. A caption holds
# until the next one is due, up to this.
HOLD="${HOLD:-6}"

python3 "${HERE}/captions.py" \
  --log "${LOG}" --start "${REC_START}" --speed "${SPEED}" --hold "${HOLD}" \
  --out "${SEGMENTS}"

# Squeezes the whitespace around and inside a caption field. The tool that
# did this before read its input as shell words, which eats the quote out
# of "the planner's r1".
trim() { sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e 's/[[:space:]]\{2,\}/ /g' <<< "$1"; }

# drawtext takes its text inside single quotes, and no amount of backslashes
# puts a literal one back in, so apostrophes are turned into the typographic
# character, which needs no escaping and reads better anyway.
escape() { sed -e "s/'/’/g" -e 's/:/\\:/g' -e 's/%/\\%/g' <<< "$1"; }

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
  compose="[0:v]crop=${PANE_LEFT},scale=-2:${PANE_HEIGHT}[l];"
  compose+="[0:v]crop=${PANE_RIGHT},scale=-2:${PANE_HEIGHT}[r];"
  # The two caption bands get a band of their own above and below the panes
  # instead of being laid over them. Drawn over, the top band hides whatever
  # the schedule view has in its first two hundred rows, which here is the
  # dock the mission ends at.
  compose+="[l][r]hstack=inputs=2,"
  compose+="pad=iw:ih+$(( 222 * U )):0:$(( 104 * U )):black,"
else
  compose=""
fi

filter="${compose}setpts=PTS/${SPEED}"
if awk -v p="${PAD}" 'BEGIN{exit !(p > 0)}'; then
  echo "holding the last frame for ${PAD}s so the closing captions fit"
  filter+=",tpad=stop_mode=clone:stop_duration=${PAD}"
fi
filter+=",drawbox=x=0:y=0:w=iw:h=$(( 104 * U )):color=black@0.66:t=fill"
filter+=",drawtext=fontfile='${FONT_MONO}':text='$(escape ":goal ${GOAL}")'"
filter+=":x=$(( 32 * U )):y=$(( 18 * U )):fontsize=$(( 27 * U )):fontcolor=0xF2F2F2"
filter+=",drawtext=fontfile='${FONT}':text='$(escape "${GLOSS}")'"
filter+=":x=$(( 32 * U )):y=$(( 60 * U )):fontsize=$(( 24 * U )):fontcolor=0xB4B4B4"

while IFS='|' read -r window headline detail; do
  [[ "${window}" == \#* || -z "${window// }" ]] && continue
  read -r start end <<< "${window}"
  on="between(t,${start},${end})"

  filter+=",drawbox=x=0:y=ih-$(( 118 * U )):w=iw:h=$(( 118 * U )):color=black@0.66:t=fill:enable='${on}'"
  filter+=",drawtext=fontfile='${FONT_BOLD}':text='$(escape "$(trim "${headline}")")'"
  filter+=":x=$(( 32 * U )):y=h-$(( 100 * U )):fontsize=$(( 34 * U )):fontcolor=white:enable='${on}'"
  filter+=",drawtext=fontfile='${FONT}':text='$(escape "$(trim "${detail}")")'"
  filter+=":x=$(( 32 * U )):y=h-$(( 54 * U )):fontsize=$(( 25 * U )):fontcolor=0xC8C8C8:enable='${on}'"
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
