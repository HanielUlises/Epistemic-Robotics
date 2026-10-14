#!/bin/bash
# One headless run of the gate signal in its own process session.
#
#     run_one.sh SEED FORMULATION OUT [LANE]
#
# A lane is a ROS domain (50 + LANE) and a Gazebo port (11350 + LANE) of the
# run's own, so that runs side by side, and another session's simulator, never
# share /clock or a Gazebo master. When the run ends its whole session is
# killed, and nothing of another run's is touched. DEADLINE, in simulated
# seconds, is the cautious r2's.
set +u
SEED=$1; FORM=$2; OUT=$(realpath -m "$3"); LANE=${4:-1}
WALL_LIMIT=${WALL_LIMIT:-900}
DEADLINE=${DEADLINE:-50.0}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$HOME/eplansys_ws/install/setup.bash"
source "$HOME/Projects/Epistemic-Robotics/ros2_ws/install/setup.bash"
source /usr/share/gazebo/setup.sh
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$((50 + LANE)) GAZEBO_MASTER_URI=http://localhost:$((11350 + LANE))
SHARE=$(ros2 pkg prefix gate_signal_demo)/share/gate_signal_demo
python3 "$SHARE/tools/make_world.py" --seed "$SEED" --out "$OUT" > "$OUT/world.log" 2>&1 || exit 1
cut -d' ' -f1-3 /proc/loadavg > "$OUT/load"
setsid bash -c "echo \$\$ > '$OUT/sid'; exec timeout -s INT -k 30 $WALL_LIMIT \
  ros2 launch gate_signal_demo gate_signal_launch.py dir:='$OUT' formulation:=$FORM deadline:=$DEADLINE" \
  > "$OUT/launch.log" 2>&1
pkill -KILL -s "$(cat "$OUT/sid")" 2>/dev/null
python3 - "$OUT" "$FORM" "$SEED" <<'PY'
import json, os, sys
out, form, seed = sys.argv[1:]
names = ['mission.json'] if form == 'epistemic' else ['mission_r1.json', 'mission_r2.json']
parts = []
for n in names:
    p = os.path.join(out, n)
    if not os.path.exists(p):
        json.dump({'formulation': form, 'result': 'NO_OUTCOME'}, open(p, 'w'))
    m = json.load(open(p))
    parts.append(f"{m.get('robot') or 'team'} {m.get('result')} {m.get('t_mission_sim', 0)} {m.get('final_bay')}")
print(f'{form} s{seed}:', '; '.join(parts))
PY
