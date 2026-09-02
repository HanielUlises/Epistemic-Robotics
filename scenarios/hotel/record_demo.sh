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
# The recording happens on a private X display, not on yours. Gazebo and RViz
# are the only things ever drawn on it, so a stray window, a notification or a
# taskbar cannot end up in the film -- which is not a hypothetical: the first
# version of this script grabbed the desktop and published it.
#
# DISPLAY_ID picks that display, GEOMETRY its size. Set USE_XVFB=0 to record
# the real screen instead, and know what is on it.
# =============================================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/../../ros2_ws" && pwd)"
OUT="${HERE}/out"
mkdir -p "${OUT}"

LEAK="${LEAK:-l2_suite}"
GEOMETRY="${GEOMETRY:-3840x1080+0+0}"
USE_XVFB="${USE_XVFB:-1}"
DISPLAY_ID="${DISPLAY_ID:-:99}"
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
XVFB_PID=""
cleanup() {
  [[ -n "${FFMPEG_PID}" ]] && kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  [[ -n "${DEMO_PID}"  ]] && kill -TERM -"${DEMO_PID}" 2>/dev/null || true
  sleep 3
  [[ -n "${DEMO_PID}"  ]] && kill -KILL -"${DEMO_PID}" 2>/dev/null || true
  [[ -n "${XVFB_PID}"  ]] && kill -KILL "${XVFB_PID}" 2>/dev/null || true
}
trap cleanup EXIT

if [[ "${USE_XVFB}" == "1" ]]; then
  SIZE="${GEOMETRY%%+*}"
  echo "starting a private display on ${DISPLAY_ID} at ${SIZE}"
  Xvfb "${DISPLAY_ID}" -screen 0 "${SIZE}x24" -nolisten tcp &
  XVFB_PID=$!
  for _ in $(seq 1 40); do
    xdpyinfo -display "${DISPLAY_ID}" >/dev/null 2>&1 && break
    sleep 0.5
  done
  xdpyinfo -display "${DISPLAY_ID}" >/dev/null 2>&1 || {
    echo "the private display never came up" >&2; exit 1; }
  export DISPLAY="${DISPLAY_ID}"
  # Xvfb has no GPU, so the two GUIs need a software renderer.
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  # Nothing else is drawn there, so the strip starts at the origin.
  GEOMETRY="${SIZE}+0+0"
fi

echo "launching the hotel, leak in ${LEAK}"
setsid env PYTHONNOUSERSITE=1 ros2 launch hotel_rmf_demo hotel_rmf_launch.py \
  "leak:=${LEAK}" > "${LOG}" 2>&1 &
DEMO_PID=$!

# Gazebo and RViz take their time. Wait for both windows, then put them side by
# side: a recording of one of them is half the point, because the floor plan
# and the robot are different pictures of the same fact.
echo "waiting for the windows"
# Gazebo and RViz both size themselves after their window first appears, so a
# single placement is undone a second later. They are placed, given time to
# settle, and placed again until the geometry sticks.
# A Qt application owns several X windows -- a 1x1 here, a 3x3 there -- and
# only one of them is the one you can see. Picking the last match gets the
# right answer sometimes, which is worse than never.
largest_window() {         # name-pattern
  local best="" best_area=0 id w h area geometry
  for id in $(xdotool search --name "$1" 2>/dev/null); do
    geometry=$(xdotool getwindowgeometry --shell "${id}" 2>/dev/null) || continue
    w=$(sed -n 's/^WIDTH=//p' <<< "${geometry}")
    h=$(sed -n 's/^HEIGHT=//p' <<< "${geometry}")
    [[ -z "${w}" || -z "${h}" ]] && continue
    area=$(( w * h ))
    if (( area > best_area )); then best_area=${area}; best=${id}; fi
  done
  # Anything this small is a helper window, not the application.
  (( best_area < 10000 )) && return 1
  echo "${best}"
}

move_window() {            # name-pattern x y
  local name="$1" x="$2" y="$3" id=""
  for _ in $(seq 1 120); do
    id=$(largest_window "${name}" || true)
    [[ -n "${id}" ]] && break
    sleep 2
  done
  [[ -z "${id}" ]] && { echo "  never saw a window named ${name}" >&2; return 1; }
  for _ in $(seq 1 10); do
    xdotool windowmove "${id}" "${x}" "${y}" 2>/dev/null || true
    sleep 1
    [[ "$(xdotool getwindowgeometry --shell "${id}" 2>/dev/null)" == *"X=${x}"* ]] && {
      echo "  moved ${name} to ${x},${y}"; return 0; }
  done
  echo "  ${name} would not move" >&2
  return 1
}

place_window() {           # name-pattern x y w h
  local name="$1" x="$2" y="$3" w="$4" h="$5" id=""
  for _ in $(seq 1 120); do
    id=$(largest_window "${name}" || true)
    [[ -n "${id}" ]] && break
    sleep 2
  done
  if [[ -z "${id}" ]]; then
    echo "  never saw a window named ${name}" >&2
    return 1
  fi

  for _ in $(seq 1 10); do
    xdotool windowmove "${id}" "${x}" "${y}" 2>/dev/null || true
    xdotool windowsize "${id}" "${w}" "${h}" 2>/dev/null || true
    sleep 1
    local geometry
    geometry=$(xdotool getwindowgeometry --shell "${id}" 2>/dev/null || true)
    if [[ "${geometry}" == *"X=${x}"* && "${geometry}" == *"WIDTH=${w}"* ]]; then
      echo "  placed ${name} at ${x},${y} ${w}x${h}"
      return 0
    fi
  done
  echo "  ${name} would not stay where it was put" >&2
  return 1
}
# RViz relays out when it is resized and Gazebo does not, so only RViz is
# given a size. Gazebo is moved to the origin and left at whatever it drew,
# which is then exactly what its window geometry reports and exactly what gets
# cut out below.
move_window "Gazebo" 0 0 || true
place_window "RViz" 1920 0 1920 1080 || true
sleep 3

# What each of them actually drew, which is not the same as the window it was
# given: see the note in tools/burn_captions.sh.
pane_of() {                # name-pattern offset-x
  local id geometry w h
  id=$(largest_window "$1") || return 1
  geometry=$(xdotool getwindowgeometry --shell "${id}")
  w=$(sed -n 's/^WIDTH=//p' <<< "${geometry}")
  h=$(sed -n 's/^HEIGHT=//p' <<< "${geometry}")
  # Even sides, and never past the bottom of the grab.
  (( w = w / 2 * 2, h = h / 2 * 2 ))
  (( h > 1080 )) && h=1080
  echo "${w}:${h}:$2:0"
}
PANE_LEFT=$(pane_of "Gazebo" 0 || echo "")
PANE_RIGHT=$(pane_of "RViz" 1920 || echo "")
echo "  panes: left ${PANE_LEFT:-none}, right ${PANE_RIGHT:-none}"

echo "recording to ${RAW}"
# The instant the grab starts, so the log's wall-clock stamps can be turned
# into positions in the film.
REC_START=$(date +%s.%N)
ffmpeg -hide_banner -loglevel error -y \
  -f x11grab -framerate "${FPS}" -video_size "${GEOMETRY%%+*}" \
  -i "${DISPLAY}+${GEOMETRY#*+}" \
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

SPEED="${SPEED}" PANE_LEFT="${PANE_LEFT}" PANE_RIGHT="${PANE_RIGHT}" \
  "${HERE}/tools/burn_captions.sh" \
  "${RAW}" "${VIDEO}" "${LOG}" "${REC_START}"

echo "raw:       ${RAW}"
echo "log:       ${LOG}"
