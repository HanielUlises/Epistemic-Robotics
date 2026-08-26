#!/bin/bash
# Gazebo and RViz side by side on the first two monitors, recorded, while the
# mu-calculus planner answers a scripted sequence of epistemic queries and a
# TurtleBot3 drives whatever route comes back.
set +u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DEMO/.." && pwd)"
OUT="$1"

source /opt/ros/humble/setup.bash
source $REPO/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
export DISPLAY=:0
export TURTLEBOT3_MODEL=burger
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
if pgrep -f "mu_path_planner_node" > /dev/null; then
  echo "FATAL: a planner survived the sweep"; exit 1
fi
rm -f $DEMO/.done

# Nothing in this workspace publishes TF, and RViz needs its fixed frame to
# exist. The diff drive plugin reports odometry in world coordinates with the
# spawn pose already in it, so map and odom coincide and this is identity.
mkdir -p $DEMO/.build
python3 $DEMO/make_world.py $DEMO/.build/building_rooms_r1.sdf || exit 1

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  > $DEMO/tf.log 2>&1 &
TF=$!

ros2 run mu_path_planner mu_path_planner_node > $DEMO/planner.log 2>&1 &
PLANNER=$!
gazebo --verbose -s libgazebo_ros_init.so $DEMO/.build/building_rooms_r1.sdf > $DEMO/gazebo.log 2>&1 &
GAZEBO=$!
rviz2 -d $DEMO/demo.rviz > $DEMO/rviz.log 2>&1 &
RVIZ=$!

for i in $(seq 1 40); do
  GZ=$(xdotool search --onlyvisible --name "^Gazebo$" 2>/dev/null | head -1)
  RV=$(xdotool search --onlyvisible --name "RViz" 2>/dev/null | head -1)
  [ -n "$GZ" ] && [ -n "$RV" ] && break
  sleep 1
done
echo "gazebo win=$GZ rviz win=$RV"
[ -n "$GZ" ] && xdotool windowmove "$GZ" 0 0 windowsize "$GZ" 1920 1040
[ -n "$RV" ] && xdotool windowmove "$RV" 1920 0 windowsize "$RV" 1920 1040
sleep 6

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

python3 $DEMO/follow_path.py > $DEMO/follow.log 2>&1 &
FOLLOW=$!

if [ "${REHEARSE:-0}" = "1" ]; then
  echo "rehearsal: not recording"
  FF=""
else
  ffmpeg -y -hide_banner -loglevel warning \
    -f x11grab -framerate 15 -video_size 3840x1040 -i :0.0+0,0 \
    -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p "$OUT" &
  FF=$!
fi
sleep 2

timeout 420 python3 -u $DEMO/demo_driver.py > $DEMO/driver.log 2>&1
echo "driver exit: $?"
sleep 2

[ -n "$FF" ] && { kill -INT $FF; wait $FF 2>/dev/null; }
kill $FOLLOW $PLANNER $RVIZ $GAZEBO $TF 2>/dev/null
sleep 2
sweep
echo "recorded to $OUT"
