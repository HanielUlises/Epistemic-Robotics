#!/usr/bin/env bash
# The warehouse scenario, end to end.
#
#   1. writes the two stages of the floor and the three snapshots
#   2. answers every case offline, with no ROS in the way
#   3. puts the same cases to a running mu_path_planner node over topics
#   4. draws both sets of answers
#
# Steps 2 and 3 are both here on purpose. The offline run says what the
# fixed point answers; the driver says that the answer survives the wire.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/../../ros2_ws" && pwd)"

# The ROS setup scripts read variables they have not set, so `set -u` only
# goes on once they are done.
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

mkdir -p "${HERE}/out"
LOG="${HERE}/out/planner.log"

echo "== floor and snapshots =="
ros2 run warehouse_scenario make_maps "${HERE}"

echo
echo "== offline =="
ros2 run warehouse_scenario run_offline "${HERE}"

echo
echo "== over ROS =="
# A planner left running from an earlier session answers these queries too,
# and the driver would read its answers as the answers to its own questions.
# One planner, started here and stopped here.
if pgrep -f "mu_path_planner_node" > /dev/null; then
  echo "stopping a mu_path_planner_node left running from an earlier run"
  pkill -f "mu_path_planner_node" || true
  sleep 0.5
fi

# `ros2 run` is a Python wrapper around the executable: killing the wrapper
# leaves the node alive. setsid puts both in their own process group, and the
# trap takes the group down.
setsid ros2 run mu_path_planner mu_path_planner_node --ros-args \
  -p free_below:=25 -p occupied_above:=65 -p sensor_range_cells:=6 \
  > "${LOG}" 2>&1 &
PLANNER=$!
trap 'kill -TERM -"${PLANNER}" 2>/dev/null || pkill -f mu_path_planner_node || true' EXIT

for _ in $(seq 1 100); do
  grep -q "MuPathPlannerNode ready" "${LOG}" && break
  sleep 0.1
done

status=0
ros2 run warehouse_scenario scenario_driver "${HERE}" || status=1

kill -TERM -"${PLANNER}" 2>/dev/null || true
wait "${PLANNER}" 2>/dev/null || true

echo
echo "== drawings =="
ros2 run warehouse_scenario render_scenario "${HERE}"

# PPM is what the renderer writes, since it needs no library to write. PNG is
# what a README wants, and netpbm is usually around.
if command -v pnmtopng > /dev/null; then
  for image in "${HERE}"/out/*.ppm; do
    pnmtopng "${image}" > "${image%.ppm}.png" && rm -f "${image}"
  done
  echo "converted to PNG"
fi

echo
echo "planner log: ${LOG}"
exit "${status}"
