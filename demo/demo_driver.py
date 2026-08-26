#!/usr/bin/env python3
"""Drive mu_path_planner over the building_rooms world.

The repository ships the planner but nothing that feeds it: no launch file, no
map source, no snapshot publisher. This is that harness, and it is external to
the repository on purpose. It rasterises the world SDF into the OccupancyGrid
the planner subscribes to, publishes a Kripke snapshot that grounds each of the
six rooms as a zone, and then walks a script of queries so the run shows the
epistemic behaviour rather than only a shortest path.

  step 1  ontic goal, next room               -> a route
  step 2  ontic goal, across the building     -> a longer route
  step 3  ontic goal in a room nobody looked  -> no route: unknown is not free
  step 4  the room is sensed, map republished -> the same query now answers
  step 5  epistemic goal, target disputed     -> sensing waypoints, not a route
  step 6  the observation collapses the model -> the agent knows, and goes
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from epistemic_msgs.msg import MuPathQuery
import tf2_ros
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_grid import Grid, shapes  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.join(HERE, '..', 'ros2_ws', 'src', 'worlds',
                     '02_building_rooms.sdf')
RES = 0.10

# The six rooms of the world, as the interior walls cut it.
ROOMS = {
    'room1': (-11.8, 0.25, -4.25, 7.8),
    'room2': (-3.75, 0.25, 3.75, 7.8),
    'room3': (4.25, 0.25, 11.8, 7.8),
    'room4': (-11.8, -7.8, -4.25, -0.25),
    'room5': (-3.75, -7.8, 3.75, -0.25),
    'room6': (4.25, -7.8, 11.8, -0.25),
}

# Where the agent starts, and the two places the mission target might be: the
# north-east room or the south-east one. The disc in the world sits at (9,-5).
START = (-10.5, 6.5)
TARGET_NORTH = (9.0, 5.0, 1.2)
TARGET_SOUTH = (9.0, -5.0, 1.2)

MIN_DWELL = 3.0   # a caption has to stay up long enough to read


def map_qos():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)



def sim_time():
    """Run on /clock.

    The GUI does not simulate at wall-clock speed, and a controller ticking on
    wall time then issues several commands per simulated step: the velocity
    window opens faster than the base can actually accelerate, and the robot
    overshoots into whatever it was rounding. On sim time the loop and the
    simulation advance together.
    """
    return [rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)]

class Demo(Node):
    def __init__(self):
        super().__init__('demo_driver', parameter_overrides=sim_time())

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos())
        self.state_pub = self.create_publisher(String, '/epistemic/state', map_qos())
        self.query_pub = self.create_publisher(MuPathQuery, '/mu_planner/query', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/demo/zones', map_qos())

        self.get_logger().info('rasterising %s' % WORLD)
        self.grid = Grid(-12.6, -8.6, 12.6, 8.6, RES)
        self.grid.fill(shapes(WORLD), value=100, inflate=0.25)

        # Rooms 3 and 6 are east of the cross corridor and nobody has looked
        # into them. -1 is a cell no observation covered.
        for name in ('room3', 'room6'):
            self.grid.set_region(*ROOMS[name], value=-1)

        self.seen = set()
        self.step = 0
        self.robot = START
        self.collapsed = False
        self.await_arrival = False
        self.arrived_flag = False
        self.wait_started = 0.0
        self.wait_limit = 0.0

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.create_subscription(Bool, '/demo/arrived', self.on_arrived, 10)

        self.publish_map()
        self.publish_state(collapsed=False)
        self.publish_markers('starting up', None)

        self.timer = self.create_timer(0.5, self.tick)

    # -- publishing ------------------------------------------------------
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RES
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in self.grid.data]
        self.map_pub.publish(msg)

    def disc_cells(self, x, y, r):
        out = []
        for row in range(self.grid.height):
            for col in range(self.grid.width):
                cx, cy = self.grid.centre(col, row)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r * r:
                    i = row * self.grid.width + col
                    if self.grid.data[i] != 100:
                        out.append(i)
        return out

    def publish_state(self, collapsed):
        """The Kripke snapshot.

        Two worlds: the target lies in room 3, or it lies in room 6. r1 cannot
        tell them apart until it senses, so the zone 'target' has a different
        extent in each. Collapsing is the observation: one world survives, and
        the agent then knows where the goal is.
        """
        north = self.disc_cells(*TARGET_NORTH)
        south = self.disc_cells(*TARGET_SOUTH)

        zones = {name: {'bounds': {'min_x': b[0], 'min_y': b[1],
                                   'max_x': b[2], 'max_y': b[3]}}
                 for name, b in ROOMS.items()}
        if collapsed:
            zones['target'] = {'worlds': {'w_north': {'cells': north}}}
        else:
            zones['target'] = {'worlds': {'w_north': {'cells': north},
                                          'w_south': {'cells': south}}}

        if collapsed:
            worlds = ['w_north']
            relations = {'r1': {'w_north': ['w_north']}}
            labels = {'w_north': ['target_in_room3']}
        else:
            worlds = ['w_north', 'w_south']
            relations = {'r1': {'w_north': ['w_north', 'w_south'],
                                'w_south': ['w_north', 'w_south']}}
            labels = {'w_north': ['target_in_room3'], 'w_south': ['target_in_room6']}

        snapshot = {
            'worlds': worlds,
            'designated': ['w_north'],
            'relations': relations,
            'labels': labels,
            'agents': {'1': {'name': 'r1', 'pose': {'x': self.robot[0],
                                                    'y': self.robot[1]}}},
            'zones': zones,
        }
        msg = String()
        msg.data = json.dumps(snapshot)
        self.state_pub.publish(msg)

    def query(self, zone, epistemic):
        msg = MuPathQuery()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.agent_id = 1
        msg.goal_zone = zone
        msg.safety_formula_json = ''
        msg.require_epistemic_goal = epistemic
        self.query_pub.publish(msg)

    # -- annotation ------------------------------------------------------
    def publish_markers(self, caption, goal_zone):
        arr = MarkerArray()
        mid = 0
        for name, (x0, y0, x1, y1) in ROOMS.items():
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'rooms', mid
            mid += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = (x0 + x1) / 2
            m.pose.position.y = (y0 + y1) / 2
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=0.0, y=0.0, z=1.0)
            lit = name in self.seen or name not in ('room3', 'room6')
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9) if lit else \
                ColorRGBA(r=1.0, g=0.75, b=0.2, a=0.9)
            m.text = name if lit else name + ' (unobserved)'
            arr.markers.append(m)

        for label, (x, y, r) in (('north', TARGET_NORTH), ('south', TARGET_SOUTH)):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'target', mid
            mid += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=2 * r, y=2 * r, z=0.05)
            m.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.45)
            arr.markers.append(m)

        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'caption', mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.0, 9.6, 0.0
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.0, y=0.0, z=1.0)
        m.color = ColorRGBA(r=0.4, g=1.0, b=0.6, a=1.0)
        m.text = caption
        arr.markers.append(m)

        self.marker_pub.publish(arr)

    def reveal(self, name):
        x0, y0, x1, y1 = ROOMS[name]
        for row in range(self.grid.height):
            for col in range(self.grid.width):
                x, y = self.grid.centre(col, row)
                if x0 <= x <= x1 and y0 <= y <= y1 and self.grid.data[row * self.grid.width + col] == -1:
                    self.grid.data[row * self.grid.width + col] = 0
        self.seen.add(name)
        self.publish_map()

    # -- pacing ----------------------------------------------------------
    def on_arrived(self, msg):
        if msg.data:
            self.arrived_flag = True

    def read_robot(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_footprint',
                                             rclpy.time.Time())
            self.robot = (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            pass

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def hold(self, seconds, for_arrival=False):
        """Hold the script for a pause, or until the robot finishes driving.

        A step that sets a route ends when the robot gets there; the seconds
        are only a cap, for the steps where no route was produced at all.
        """
        self.await_arrival = for_arrival
        self.arrived_flag = False
        self.wait_started = self.now()
        self.wait_limit = seconds

    def ready(self):
        """Sim time starts at zero and the first /clock may be a while coming;
        a hold set before it arrives would expire against a clock that has not
        started."""
        return self.now() > 0.0

    def tick(self):
        if not self.ready():
            return
        if self.wait_started == 0.0:
            self.hold(8)
            return
        self.read_robot()
        elapsed = self.now() - self.wait_started
        if elapsed < MIN_DWELL:
            return
        if self.await_arrival and self.arrived_flag:
            self.advance()
        elif elapsed >= self.wait_limit:
            self.advance()

    # -- the script ------------------------------------------------------
    def advance(self):
        self.step += 1
        s = self.step
        self.get_logger().info('step %d at t=%.1f  robot=(%.2f, %.2f)'
                               % (s, self.now(), self.robot[0], self.robot[1]))

        if s == 1:
            self.publish_markers('1  ontic goal: room2, through the near door', 'room2')
            self.publish_state(self.collapsed)
            self.query('room2', False)
            self.hold(60, for_arrival=True)

        elif s == 2:
            self.publish_markers('2  ontic goal: room5, across the cross corridor', 'room5')
            self.publish_state(self.collapsed)
            self.query('room5', False)
            self.hold(90, for_arrival=True)

        elif s == 3:
            self.publish_markers(
                '3  goal "target": in a room nobody has observed  ->  no route', 'target')
            self.publish_state(self.collapsed)
            self.query('target', False)
            self.hold(10)

        elif s == 4:
            self.reveal('room3')
            self.publish_markers(
                '4  a sensing action resolves room3: unobserved becomes free', 'room3')
            self.hold(7)

        elif s == 5:
            self.publish_markers('5  the same query answers now', 'target')
            self.publish_state(self.collapsed)
            self.query('target', False)
            self.hold(120, for_arrival=True)

        elif s == 6:
            self.reveal('room6')
            self.publish_markers(
                '6  room6 sensed too. Epistemic goal: r1 must KNOW it arrived', 'target')
            self.publish_state(self.collapsed)
            self.query('target', True)
            self.hold(60, for_arrival=True)

        elif s == 7:
            self.publish_markers(
                '7  red = where sensing pays: the cells the two worlds dispute', 'target')
            self.hold(9)

        elif s == 8:
            self.collapsed = True
            self.publish_markers(
                '8  the observation collapses the model to one world', 'target')
            self.publish_state(collapsed=True)
            self.hold(7)

        elif s == 9:
            self.publish_markers(
                '9  epistemic goal again: nothing disputed, so it knows and goes',
                'target')
            self.query('target', True)
            self.hold(120, for_arrival=True)

        else:
            self.publish_markers('done', None)
            open(os.path.join(HERE, '.done'), 'w').write('1')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
