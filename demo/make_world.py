#!/usr/bin/env python3
"""Write the demo world: 02_building_rooms with a TurtleBot3 in room 1.

Derived rather than committed, so the world stays the single source of truth
for the geometry and this file only records what the demo adds to it: a robot,
a camera that looks at the building, and the plugin that publishes model poses.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, '..', 'ros2_ws', 'src', 'worlds',
                      '02_building_rooms.sdf')

ADDITIONS = """
    <!-- Publishes /gazebo/model_states: the robot's true pose. Wheel odometry
         keeps integrating while a base is held against something, so it
         cannot answer whether anything was hit. Declaring this with -s on the
         command line does not register it; it has to be in the world. -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>10.0</update_rate>
    </plugin>

    <!-- Without a camera pose the GUI opens looking at empty space beside the
         building, which is what a recording then shows. -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -26 24 0 0.72 1.5708</pose>
      </camera>
    </gui>

    <!-- The agent r1, where the epistemic snapshot places it. -->
    <include>
      <uri>model://turtlebot3_burger</uri>
      <name>r1</name>
      <pose>-10.5 6.5 0.01 0 0 -1.5708</pose>
    </include>

  </world>"""


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, '.build', 'building_rooms_r1.sdf')
    text = open(SOURCE).read()
    if text.count('</world>') != 1:
        raise SystemExit('unexpected world file: %s' % SOURCE)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    open(out, 'w').write(text.replace('  </world>', ADDITIONS))
    print('wrote %s' % out)


if __name__ == '__main__':
    main()
