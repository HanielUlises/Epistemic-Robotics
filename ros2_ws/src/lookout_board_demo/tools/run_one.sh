#!/bin/bash
# One headless run of the lookout board in its own process session.
#
#     run_one.sh SEED FORMULATION OUT [LANE]
#
# A lane is a ROS domain (50 + LANE) and a Gazebo port (11350 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched.
set +u
SEED=$1; FORM=$2; OUT=$(realpath -m "$3"); LANE=${4:-1}
WALL_LIMIT=${WALL_LIMIT:-900}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((50 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11350 + LANE))
SHARE=$(ros2 pkg prefix lookout_board_demo)/share/lookout_board_demo
python3 "$SHARE/tools/make_world.py" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch lookout_board_demo lookout_board_launch.py dir:='$OUT' formulation:=$FORM" \
  > "$OUT/launch.log" 2>&1
pkill -KILL -s "$(cat "$OUT/sid")" 2>/dev/null
[ -f "$OUT/mission.json" ] || echo '{"formulation": "'"$FORM"'", "result": "NO_OUTCOME", "success": false}' > "$OUT/mission.json"
python3 -c "import json;m=json.load(open('$OUT/mission.json'));print('$FORM s$SEED', m.get('result'), round(m.get('t_mission_sim',0),1), m.get('zone_on_map'), m.get('visited'))"
