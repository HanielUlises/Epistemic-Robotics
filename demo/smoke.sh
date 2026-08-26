#!/bin/bash
set +u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DEMO/.." && pwd)"
source /opt/ros/humble/setup.bash
source $REPO/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78 TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Leftovers from an earlier run are not harmless: a second planner answers the
# same query with a second path, and a second follower fights the first one for
# /cmd_vel, which looks exactly like a robot that cannot drive. Sweep first,
# and check the sweep worked before starting anything.
sweep() {
  for pat in "follow_path.py" "demo_driver.py" "mu_path_planner_node" \
             "mu_path_planner/lib" "static_transform_publisher"; do
    pkill -9 -f "$pat" 2>/dev/null
  done
  pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x rviz2
  # gzserver holds the master port, and a new one dies with "Address already
  # in use" if the old is still down. That failure is silent from the outside:
  # no simulation means no TF, and the follower then sits waiting for a pose
  # it will never get, which looks exactly like a controller that cannot
  # drive. Wait for the port to actually come free.
  for _ in $(seq 1 15); do
    pgrep -x gzserver > /dev/null || break
    sleep 1
  done
}

sweep

mkdir -p $DEMO/.build
python3 $DEMO/make_world.py $DEMO/.build/building_rooms_r1.sdf || exit 1

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > $DEMO/tf.log 2>&1 & TF=$!
gzserver --verbose -s libgazebo_ros_init.so $DEMO/.build/building_rooms_r1.sdf > $DEMO/gazebo.log 2>&1 & GZ=$!
ros2 run mu_path_planner mu_path_planner_node > $DEMO/planner.log 2>&1 & PL=$!

# Nothing below works without a simulation, and a topic appearing in the
# listing is not proof of one: a dying gzserver leaves its names in discovery
# for a while, so the check has to be a message that actually arrives.
ok=0
for i in $(seq 1 40); do
  if grep -q "Address already in use" $DEMO/gazebo.log 2>/dev/null; then
    echo "FATAL: gazebo could not bind its port"; exit 1
  fi
  if timeout 4 ros2 topic echo /gazebo/model_states --once > /dev/null 2>&1; then
    ok=1; break
  fi
  sleep 1
done
if [ "$ok" != "1" ]; then
  echo "FATAL: no live /gazebo/model_states, the simulation is not running"
  exit 1
fi
if ! pgrep -x gzserver > /dev/null; then
  echo "FATAL: gzserver died during start-up"; exit 1
fi

python3 $DEMO/follow_path.py > $DEMO/follow.log 2>&1 & FO=$!
timeout 420 python3 -u $DEMO/demo_driver.py > $DEMO/driver.log 2>&1
kill $FO $PL $TF $GZ 2>/dev/null
sleep 2
sweep
echo SMOKE_DONE
