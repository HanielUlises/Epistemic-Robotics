#!/usr/bin/env bash
# =============================================================================
# One nested run: the same warehouse, the same fleet of three, the pallet in
# bay3, and one of the three goals.
#
#   ./run.sh G2      r2 has to know that r1 knows whether
#   ./run.sh G3      r2 itself has to know whether
#
# Everything else is held fixed on purpose. The world, the robots, their start
# poses, the perception regions, the action nodes and the pallet are the same
# in both; the only thing that differs between a G2 run and a G3 run is which
# EPDDL problem file the planner and the epistemic state were given, and those
# two files differ from each other in one line.
#
# While it runs, `formula_probe` asks the epistemic state all three formulas
# twice a second and writes what it said to csv/. The instant that log shows
#
#   (K r2 (Kw r1 pallet-at_bay2))  TRUE     while     (Kw r2 pallet-at_bay2)  FALSE
#
# is the result: two formulas over one model, one of them settled by the look
# alone and the other still open.
# =============================================================================
set -eo pipefail

GOAL="${1:?usage: run.sh G1|G2|G3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HERE}/../../ros2_ws"
STAMP="$(date +%Y%m%d-%H%M%S)"

source /opt/ros/humble/setup.bash
source "${EPLANSYS:-$HOME/eplansys_ws}/install/setup.bash"
source "${WS}/install/setup.bash"
export ALETHEIA_PLANNER="${ALETHEIA_PLANNER:-$HOME/aletheia/team-3/build/epistemic_planner}"

LOG="${HERE}/logs/${GOAL}.log"
CSV="${HERE}/csv/${GOAL}.csv"
VIDEO="${HERE}/media/${GOAL}-raw.mp4"

# Where the run is drawn.
#
# By default, the real display: Gazebo's client wants a GPU, and on this
# machine the difference between hardware GL and llvmpipe is the difference
# between a simulator that keeps up with wall clock and one that does not --
# which changes the run, not just the recording.
#
# XVFB=1 draws it into an off-screen X server instead, which frees the desktop
# and fixes the capture geometry exactly. It is opt-in rather than the default
# because on this machine it does not merely slow the picture down: Xvfb has no
# hardware GL, llvmpipe costs enough CPU that the simulator runs at about two
# thirds of wall clock, and plansys2's bringup then loses a race it otherwise
# wins -- the lifecycle manager declares the epistemic state failed to
# configure two tenths of a second before the state logs that it configured
# successfully. Nothing is wrong with any of the nodes. The run simply does not
# start, and it takes a while to see why.
#
# `gui:=false` remains the way to run with nothing drawn at all.
if [ "${XVFB:-0}" = "1" ]; then
  Xvfb :99 -screen 0 3840x1080x24 -nolisten tcp > /dev/null 2>&1 &
  XSERVER=$!
  sleep 2
  export DISPLAY=:99
  export LIBGL_ALWAYS_SOFTWARE=1
  GRAB=:99.0
else
  export DISPLAY="${DISPLAY:-:0}"
  GRAB="${DISPLAY}.0"
fi

# The two screens Gazebo and rviz are on. The capture is what the run looked
# like; the caption that goes over it later comes from the log beside it, so
# the two cannot disagree about what happened when.
if [ "${RECORD:-1}" = "1" ]; then
  # `-nostdin`, and it is not cosmetic. ffmpeg backgrounded from a script
  # reads the script's own standard input, and what it swallows is whatever
  # the shell was going to read next: the script exits partway through with
  # status 1 while the launch it started carries on running, which looks like
  # the run failing and is the recorder eating the terminal.
  ffmpeg -y -nostdin -loglevel error -f x11grab -framerate 10 \
    -video_size 3840x1080 -i "${GRAB}"+0,0 \
    -c:v libx264 -preset ultrafast -crf 26 -pix_fmt yuv420p \
    "${VIDEO}" < /dev/null &
  FFMPEG=$!
  trap 'kill ${FFMPEG} 2>/dev/null || true' EXIT
fi

# Nothing of a previous run may still be up, and this is not housekeeping.
#
# `epistemic_state` is a node name, and a node name is an address: two of them
# on the graph means the bringup's `change_state` call and every `check_formula`
# call land on whichever answers first. The symptom is not an error. It is the
# lifecycle manager failing to configure the state for no stated reason, and a
# probe whose three formulas flicker between TRUE and FALSE twice a second
# while the robots are still being spawned -- which reads exactly like the
# model being wrong, and is two models being asked one question.
for stray in epistemic_state_node plansys2_node formula_probe_node \
             gzserver gzclient slam_toolbox mu_path_planner; do
  pkill -9 -f "${stray}" 2>/dev/null || true
done
sleep 3

echo "run ${GOAL} at ${STAMP}; log ${LOG}"
date +%s.%N > "${HERE}/logs/${GOAL}.start"

ros2 launch warehouse_demo warehouse_demo_launch.py \
  robots:=3 goal:="${GOAL}" pallet:=bay3 gui:="${GUI:-true}" \
  probe_csv:="${CSV}" > "${LOG}" 2>&1 &
LAUNCH=$!

# Gazebo on the left half of the frame, rviz on the right, once both have
# actually mapped a window. Left to themselves the two come up overlapping and
# the capture is one application on top of the other; there is no window
# manager inside Xvfb to arrange them, and outside it there is no reason to
# leave the geometry to whatever the desktop last remembered.
#
# The two names are anchored, and that is the whole of the bug this had.
# `--name Gazebo` also matches `Qt Selection Owner for gazebo`, an unmapped
# utility window Qt creates before either application has a real one -- so the
# unanchored version fired within seconds of launch, moved two invisible
# windows, and stopped, leaving the actual panes stacked on top of each other
# for the length of the run.
#
# And it re-asserts rather than placing once. Both applications restore their
# own remembered geometry a moment after mapping, and whichever of the two
# does it last is the one that ends up covering the other.
if [ "${GUI:-true}" = "true" ] && command -v xdotool > /dev/null; then
  (
    # `set +e +o pipefail`, and this is the whole reason nothing was ever
    # placed. The script runs under `set -eo pipefail`, a subshell inherits
    # both, and `xdotool search ... | tail -1` exits non-zero for as long as
    # the window does not exist -- which is the first few minutes of every
    # run. So the loop was killed on its first iteration, silently, every
    # time, and the two panes stayed wherever the desktop had put them.
    set +e +o pipefail

    # For the length of the run, not for the length of the bringup. Gazebo's
    # own window does not exist for the first few minutes -- there is a splash
    # screen where it will be -- so a loop that gives up early gives up before
    # the window it is waiting for has been created.
    for _ in $(seq 1 "${TIMEOUT_TICKS:-900}"); do
      gz=$(xdotool search --name '^Gazebo$' 2>/dev/null | tail -1)
      rv=$(xdotool search --name 'RViz$' 2>/dev/null | tail -1)
      if [ -n "${gz}" ] && [ -n "${rv}" ]; then
        # `--sync`, and without it nothing moves at all. A window manager
        # reparents each window into a frame of its own, and an asynchronous
        # move races the reparenting: the request is answered, the geometry
        # reads back unchanged, and both panes stay wherever the desktop put
        # them. Under Xvfb, where there is no window manager, the same
        # commands work without it -- which is a good way to be misled.
        xdotool windowmove --sync "${gz}" 0 0    2>/dev/null
        xdotool windowsize --sync "${gz}" 1920 1080 2>/dev/null
        xdotool windowmove --sync "${rv}" 1920 0   2>/dev/null
        xdotool windowsize --sync "${rv}" 1920 1080 2>/dev/null
      fi
      sleep 3
    done
  ) &
fi

# Ends when the mission says it is done, and not on a stopwatch: a fleet of
# three on a loaded machine takes as long as it takes.
for _ in $(seq 1 "${TIMEOUT_TICKS:-900}"); do
  if grep -q "mission complete\|did not reach its goal" "${LOG}"; then
    sleep 8   # a few seconds of the finished state, for the recording
    break
  fi
  if ! kill -0 ${LAUNCH} 2>/dev/null; then break; fi
  sleep 1
done

kill -INT ${LAUNCH} 2>/dev/null || true
sleep 6
pkill -f gzserver || true; pkill -f gzclient || true
[ -n "${XSERVER:-}" ] && kill "${XSERVER}" 2>/dev/null
grep -E "policy:|executing |mission complete|did not reach" "${LOG}" | tail -20
