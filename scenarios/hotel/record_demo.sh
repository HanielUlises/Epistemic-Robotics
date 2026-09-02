#!/usr/bin/env bash
# =============================================================================
# Films the hotel incident: Gazebo on the left, RViz on the right, the way the
# Open-RMF demo recordings are laid out.
#
#   bash scenarios/hotel/record_demo.sh                 # leak in the L2 suite
#   LEAK=l3_suite bash scenarios/hotel/record_demo.sh   # and the other branch
#
# The mission ends itself, and the recorder stops when it does, so the length
# of the film is the length of the run. Most of that is lift rides, which is
# the honest picture: the epistemic work takes milliseconds and the building
# takes minutes.
#
# The two windows are moved onto a 3840x1080 strip and grabbed as one, then
# scaled to 1920x540. On a single-monitor machine set GEOMETRY to whatever
# fits and the crop follows.
# =============================================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/../../ros2_ws" && pwd)"
OUT="${HERE}/out"
mkdir -p "${OUT}"

LEAK="${LEAK:-l2_suite}"
GEOMETRY="${GEOMETRY:-3840x1080+0+0}"
FPS="${FPS:-12}"
# Most of a run is two robots riding lifts. The film is sped up so that the
# parts worth watching are not separated by four minutes of corridor.
SPEED="${SPEED:-4}"
RAW="${OUT}/hotel-${LEAK}-raw.mp4"
VIDEO="${OUT}/hotel-${LEAK}.mp4"
SEGMENTS="${OUT}/hotel-${LEAK}-segments.txt"
LOG="${OUT}/hotel-${LEAK}.log"
FONT="${FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf}"
FONT_BOLD="${FONT_BOLD:-/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf}"

source /opt/ros/humble/setup.bash
[ -f "${HOME}/eplansys_ws/install/setup.bash" ] && source "${HOME}/eplansys_ws/install/setup.bash"
[ -f "${HOME}/rmf_ws/install/setup.bash" ]      && source "${HOME}/rmf_ws/install/setup.bash"
source "${WS}/install/setup.bash"
set -u

DEMO_PID=""
FFMPEG_PID=""
cleanup() {
  [[ -n "${FFMPEG_PID}" ]] && kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  [[ -n "${DEMO_PID}"  ]] && kill -TERM -"${DEMO_PID}" 2>/dev/null || true
  sleep 3
  [[ -n "${DEMO_PID}"  ]] && kill -KILL -"${DEMO_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "launching the hotel, leak in ${LEAK}"
setsid env PYTHONNOUSERSITE=1 ros2 launch hotel_rmf_demo hotel_rmf_launch.py \
  "leak:=${LEAK}" > "${LOG}" 2>&1 &
DEMO_PID=$!

# Gazebo and RViz take their time. Wait for both windows, then put them side by
# side: a recording of one of them is half the point, because the floor plan
# and the robot are different pictures of the same fact.
echo "waiting for the windows"
place_window() {           # name-pattern x y w h
  for _ in $(seq 1 120); do
    local id
    id=$(xdotool search --name "$1" 2>/dev/null | tail -1 || true)
    if [[ -n "${id}" ]]; then
      xdotool windowmove "${id}" "$2" "$3" windowsize "${id}" "$4" "$5" 2>/dev/null || true
      echo "  placed $1"
      return 0
    fi
    sleep 2
  done
  echo "  never saw a window named $1" >&2
  return 1
}
place_window "Gazebo" 0 0 1920 1080 || true
place_window "RViz"   1920 0 1920 1080 || true
sleep 5

echo "recording to ${RAW}"
# The instant the grab starts, so the log's wall-clock stamps can be turned
# into positions in the film.
REC_START=$(date +%s.%N)
ffmpeg -hide_banner -loglevel error -y \
  -f x11grab -framerate "${FPS}" -video_size "${GEOMETRY%%+*}" \
  -i ":0.0+${GEOMETRY#*+}" \
  -vf "scale=1920:-2" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
  "${RAW}" &
FFMPEG_PID=$!

# The mission node emits one of these two lines and then the launch shuts
# everything down, so this is also how long the film runs.
echo "waiting for the mission"
for _ in $(seq 1 900); do
  if grep -qE "mission complete|mission failed" "${LOG}" 2>/dev/null; then
    break
  fi
  sleep 1
done

sleep 4
kill -INT "${FFMPEG_PID}" 2>/dev/null || true
wait "${FFMPEG_PID}" 2>/dev/null || true
FFMPEG_PID=""

# ffmpeg writes the moov atom last. Burning before it lands reads a file with
# no index and fails, so wait until the recording can actually be probed.
for _ in $(seq 1 30); do
  ffprobe -v error -show_entries format=duration -of csv=p=0 "${RAW}" >/dev/null 2>&1 && break
  sleep 1
done

grep -aE "mission complete|mission failed" "${LOG}" || echo "the mission never reported"

SPEED="${SPEED}" "${HERE}/tools/burn_captions.sh" \
  "${RAW}" "${VIDEO}" "${LOG}" "${REC_START}"

echo "raw:       ${RAW}"
echo "log:       ${LOG}"
