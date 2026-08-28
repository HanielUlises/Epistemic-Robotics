#!/usr/bin/env bash
# Kills a warehouse demo and everything it started.
#
# pkill -x cannot be used for these: Linux truncates a process name to 15
# characters, so `async_slam_toolbox_node` is `async_slam_tool` in the table
# and an exact match never fires. Matching the full command line is the only
# thing that works, and it has to happen from a script so that the pattern is
# not in the calling shell's own arguments -- otherwise the cleanup kills the
# shell that runs it.
for pattern in \
  gzserver gzclient rviz2 \
  async_slam_toolbox_node \
  plansys2_node epistemic_state_node epistemic_perception_node \
  mu_path_planner_node \
  controller_server planner_server behavior_server bt_navigator \
  velocity_smoother lifecycle_manager map_server \
  drive_action_node look_action_node handle_action_node mission_node \
  warehouse_pilot scenario_driver robot_state_publisher
do
  pkill -9 -f "$pattern" 2>/dev/null
done
# Recorders too. An ffmpeg left grabbing the screen from an abandoned run
# keeps writing to the same output file as the next one, and the pair produce
# an mp4 with no index that no player will open -- which looks like a bug in
# the recording and is really two writers.
pkill -9 -f "x11grab" 2>/dev/null
pkill -9 -f "Xvfb :77" 2>/dev/null

sleep 2
left=$(pgrep -f "slam_toolbox|plansys2_node|mu_path_planner|gzserver" | wc -l)
echo "processes left: ${left}"
