#!/usr/bin/env bash
# Records the warehouse mission, executed by ePlanSys.
#
# Gazebo on the left screen, rviz on the right, one recording across both. The
# left half is the AWS warehouse and a robot that has to drive around the
# racks; the right half is the map that robot is building, the route the
# mu-calculus planner returned over it, and the pallet being found in one bay
# rather than the other.
#
# Left to right is therefore what-is-there to what-is-known: the warehouse as
# it stands, and beside it the far smaller thing the robot has measured of it.
#
# What is worth watching is the order: the policy branches at `look_into`, and
# which branch runs is decided by what plansys2_epistemic_perception read off
# the map -- not by this script, and not by the action node.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# One recorder at a time.
#
# Two of these running together is not merely wasteful: each ends by running
# stop.sh, so whichever finishes first kills the other's ffmpeg in the middle
# of writing. The half-written mp4 then has no index and the speed-up pass
# fails with "Error splitting the input into NAL units", which reads like a
# codec problem rather than what it is -- two recorders sharing one file.
LOCK="/tmp/warehouse-record.lock"
exec 9>"${LOCK}"
if ! flock -n 9; then
  echo "another recording is already running (${LOCK}); refusing to start" >&2
  exit 1
fi
WS="$(cd "${HERE}/../../ros2_ws" && pwd)"
EPLANSYS_WS="${EPLANSYS_WS:-${HOME}/eplansys_ws}"

source /opt/ros/humble/setup.bash
source "${EPLANSYS_WS}/install/setup.bash"
source "${WS}/install/setup.bash"
set -u

export TURTLEBOT3_MODEL=burger

OUT="${HERE}/out"
mkdir -p "${OUT}"
LOG="${OUT}/demo.log"
VIDEO="${OUT}/warehouse_demo.mp4"

SECONDS_TO_RECORD="${SECONDS_TO_RECORD:-200}"

# The mission is a robot crossing a twenty-metre warehouse at a fifth of a
# metre a second, and most of that is driving in a straight line. SPEED is the
# factor the finished recording is sped up by; 1 leaves it alone.
SPEED="${SPEED:-6}"
GRAB="${GRAB:-3840x1040}"
GRAB_AT="${GRAB_AT:-+0,40}"

# Anything left from a previous run answers the same topics and publishes the
# same frames, and the demonstration then measures two worlds at once.
bash "${HERE}/stop.sh"

# A display of its own, if one can be had.
#
# x11grab films the actual screen, so anything put in front of Gazebo during
# the run -- a browser, a file manager -- is what ends up in the recording.
# Xvfb gives the demo a virtual screen nobody else is using: the desktop is
# untouched for the seven minutes this takes, and the capture cannot catch
# stray windows. Without it the recording falls back to the real display and
# the screens have to be left alone.
XVFB_PID=""
if command -v Xvfb > /dev/null && [ "${USE_XVFB:-1}" = "1" ]; then
  VDISPLAY="${VDISPLAY:-:77}"
  Xvfb "${VDISPLAY}" -screen 0 3840x1080x24 -nolisten tcp > /dev/null 2>&1 &
  XVFB_PID=$!
  for _ in $(seq 1 50); do
    DISPLAY="${VDISPLAY}" xdpyinfo > /dev/null 2>&1 && break
    sleep 0.2
  done
  if DISPLAY="${VDISPLAY}" xdpyinfo > /dev/null 2>&1; then
    export DISPLAY="${VDISPLAY}"
    echo "recording on a virtual display ${VDISPLAY}; your desktop is untouched"
  else
    echo "Xvfb did not come up; falling back to the real display" >&2
    kill "${XVFB_PID}" 2>/dev/null || true
    XVFB_PID=""
  fi
else
  echo "Xvfb not installed: filming the real screen, so leave it alone" >&2
fi


setsid ros2 launch warehouse_demo warehouse_demo_launch.py > "${LOG}" 2>&1 &
LAUNCH=$!
trap 'kill -TERM -"${LAUNCH}" 2>/dev/null || true; bash "${HERE}/stop.sh" > /dev/null; [ -n "${XVFB_PID}" ] && kill "${XVFB_PID}" 2>/dev/null' EXIT

# Wait for both windows rather than a fixed number of seconds: Gazebo's first
# load is slow, and getting this wrong films the desktop instead of the demo.
# --onlyvisible matters: a window exists before it is mapped, and moving one
# that is not yet on screen does nothing at all.
gz_window=""
rviz_window=""
for _ in $(seq 1 150); do
  gz_window=$(xdotool search --onlyvisible --name '^Gazebo$' 2>/dev/null | head -1 || true)
  rviz_window=$(xdotool search --onlyvisible --name 'RViz' 2>/dev/null | head -1 || true)
  [ -n "${gz_window}" ] && [ -n "${rviz_window}" ] && break
  sleep 1
done

if [ -z "${gz_window}" ] || [ -z "${rviz_window}" ]; then
  echo "gazebo or rviz never appeared; refusing to record the desktop" >&2
  exit 1
fi

xdotool windowmove "${gz_window}" 0 40 windowsize "${gz_window}" 1900 1000
xdotool windowmove "${rviz_window}" 1920 40 windowsize "${rviz_window}" 1900 1000
sleep 2

gz_x=$(xdotool getwindowgeometry --shell "${gz_window}" | sed -n 's/^X=//p')
rviz_x=$(xdotool getwindowgeometry --shell "${rviz_window}" | sed -n 's/^X=//p')
if [ "${gz_x}" -gt 960 ] || [ "${rviz_x}" -lt 960 ]; then
  echo "windows did not land where they were put (gazebo x=${gz_x}, rviz x=${rviz_x});" >&2
  echo "refusing to record what is actually on those screens" >&2
  exit 1
fi

# The mission node waits for the map to settle before it plans; start filming
# just before it does, so the policy arrives on camera.
sleep 8

# Fifteen frames a second, not thirty. The grab is 3840 px wide and the
# machine is also running Gazebo, SLAM and Nav2; at thirty the encoder falls
# behind the wall clock and a 380-second capture takes a quarter of an hour to
# finish. Nothing here moves fast enough to need more.
# The simulator's clock at the moment the capture begins, not the wall's.
#
# Every stamp in the log is ROS time with use_sim_time, which starts when
# Gazebo does and advances at whatever rate the simulation manages -- not when
# ffmpeg starts and not at one second per second. Anchoring the captions to
# `date` puts them minutes out: the epistemic line then reads "= TRUE" from
# the opening frame, claiming the robot knows which bay before it has looked,
# which is precisely the thing this caption exists to report honestly.
REC_START=$(grep -aoE "\[178[0-9]{7}\.[0-9]+\]" "${LOG}" | tail -1 | tr -d '[]')
if [ -z "${REC_START}" ]; then
  echo "no simulator clock in the log yet; captions would be mistimed" >&2
  REC_START=0
fi
echo "recording ${SECONDS_TO_RECORD}s of ${GRAB}${GRAB_AT}"
ffmpeg -y -hide_banner -loglevel error \
  -f x11grab -framerate 15 -video_size "${GRAB}" -i "${DISPLAY}${GRAB_AT}" \
  -t "${SECONDS_TO_RECORD}" \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -movflags +faststart \
  "${VIDEO}"

if [ "${SPEED}" != "1" ]; then
  # setpts alone: there is no audio track to keep in step with.
  ffmpeg -y -hide_banner -loglevel error -i "${VIDEO}" \
    -filter:v "setpts=PTS/${SPEED}" -r 30 \
    -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
    -movflags +faststart "${VIDEO%.mp4}.x${SPEED}.mp4"
  mv "${VIDEO%.mp4}.x${SPEED}.mp4" "${VIDEO}"
  echo "sped up ${SPEED}x"
fi

# The narration, burned in afterwards rather than drawn in rviz. See the
# comment at the top of caption_video.py: an rviz text marker is scene
# geometry and comes out unreadable once the capture is scaled down.
# Kept uncaptioned. Burning the text in is destructive -- caption twice and
# the second pass draws over the first -- and getting the caption timing wrong
# should not cost another seven-minute run.
cp "${VIDEO}" "${VIDEO%.mp4}.raw.mp4"

python3 "${HERE}/tools/caption_video.py" \
  --video "${VIDEO}" --log "${LOG}" \
  --start "${REC_START}" --speed "${SPEED}" \
  --width "${GRAB%%x*}" || echo "captioning failed; the raw recording stands"

echo "wrote ${VIDEO}"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "${VIDEO}"

echo
grep -aE "warehouse_mission\]: (executing|mission|policy)|is blocked|is clear|holds after|lifting|unloading" \
  "${LOG}" | grep -av rcl.logging | tail -20
