#!/usr/bin/env python3
"""Drive the TurtleBot3 along whatever route the planner last published.

The route is a global plan over cells; what the base needs is a velocity, and
the two are separated here the way a navigation stack separates them:

  1. Shortcutting. The planner's graph is 4-connected, so its shortest path is
     a staircase grazing every inside corner. Line-of-sight string pulling over
     the same occupancy the planner used turns it into a polyline, and the
     polyline is what the local planner aims at.

  2. A dynamic window. Every tick, the reachable (v, w) pairs one acceleration
     step away are sampled, each is rolled forward as a constant-curvature arc,
     and any arc that brings the footprint within COLLISION of a laser return
     is thrown out. What survives is scored on three terms that pull against
     each other: progress towards the local goal, clearance from everything
     seen, and speed. The scan is the authority here, not the map, so an
     obstacle the map never had is still avoided.

  3. A recovery. When the window is empty -- nose in a corner, every arc
     blocked -- there is no velocity to pick, so the base turns towards the
     freer side, and backs out if it is truly wedged.

Publishes /demo/arrived when the last waypoint is reached.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
    ReliabilityPolicy

import tf2_ros
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker

# The base. A burger is 0.178 wide, so 0.09 is the real half-width; the rest
# is margin. This has to stay well under the 0.25 the map is inflated by, or
# the planner hands over a route whose own clearance the local planner then
# rejects, every arc is inadmissible, and the base sits there recovering from
# a corridor it was always able to drive.
ROBOT_RADIUS = 0.13
COLLISION = ROBOT_RADIUS + 0.02

V_MAX, V_MIN = 0.35, 0.0
W_MAX = 1.6
A_V, A_W = 0.7, 2.0          # m/s^2, rad/s^2
CONTROL_DT = 0.1             # s, one tick
HORIZON = 1.5                # s, how far each arc is rolled forward
SIM_STEPS = 14

N_V, N_W = 7, 25             # samples across the window

# The three terms. Clearance is capped: past CLEAR_CAP metres, more room is
# not worth trading speed for.
W_GOAL, W_CLEAR, W_SPEED = 1.0, 0.32, 0.50
CLEAR_CAP = 0.7

LOOKAHEAD = 1.3              # m, how far along the path the local goal sits
WAYPOINT_REACHED = 0.45      # m
ARRIVED_WITHIN = 0.28        # m
CLEARANCE = 0.10             # m, on top of the inflation already in the map



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

class Follower(Node):
    def __init__(self):
        super().__init__('follow_path', parameter_overrides=sim_time())
        self.waypoints = []
        self.announced = True
        self.grid = None
        self.obstacles = np.zeros((0, 2))
        # The window is built around what was last commanded, not around what
        # odometry reports. A window centred on a measurement that lags, or
        # that never arrives, closes on itself: nothing faster than one
        # acceleration step is ever sampled and the base creeps.
        self.v, self.w = 0.0, 0.0
        self.odom_v, self.odom_w = 0.0, 0.0

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path, '/mu_planner/path', self.on_path, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, best_effort)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arrived = self.create_publisher(Bool, '/demo/arrived', latched)
        self.marker = self.create_publisher(Marker, '/demo/robot', 10)
        self.smoothed = self.create_publisher(Path, '/demo/smoothed', 10)
        self.chosen = self.create_publisher(Path, '/demo/local_plan', 10)

        self.recoveries = 0
        self.debug = False
        self.create_timer(CONTROL_DT, self.tick)
        self.create_timer(5.0, self.report)

    def report(self):
        self.debug = True
        if self.waypoints:
            self.get_logger().info(
                'cmd v=%.2f w=%.2f  odom v=%.2f  waypoints=%d  recoveries=%d'
                % (self.v, self.w, self.odom_v, len(self.waypoints),
                   self.recoveries))

    # -- inputs ----------------------------------------------------------
    def on_map(self, msg):
        self.grid = msg

    def on_odom(self, msg):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z

    def on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        good = np.isfinite(ranges) & (ranges > msg.range_min) & \
            (ranges < min(msg.range_max, 3.0))
        r, a = ranges[good], angles[good]
        self.obstacles = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

    def on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waypoints = self.smooth(self.shortcut(pts))
        self.announced = not self.waypoints
        if self.waypoints:
            self.get_logger().info(
                '%d poses -> %d waypoints' % (len(pts), len(self.waypoints)))
            self.publish_path(self.smoothed, self.waypoints, msg.header.frame_id)

    # -- the map, for line of sight ---------------------------------------
    def blocked(self, x, y):
        g = self.grid
        if g is None:
            return False
        col = int((x - g.info.origin.position.x) / g.info.resolution)
        row = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= col < g.info.width and 0 <= row < g.info.height):
            return True
        v = g.data[row * g.info.width + col]
        return v < 0 or v > 50      # unknown is not free here either

    def clear_segment(self, a, b):
        length = math.dist(a, b)
        if length == 0:
            return True
        steps = max(2, int(length / 0.05))
        nx, ny = (b[1] - a[1]) / length, -(b[0] - a[0]) / length
        for k in range(steps + 1):
            t = k / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            for off in (0.0, CLEARANCE, -CLEARANCE):
                if self.blocked(x + nx * off, y + ny * off):
                    return False
        return True

    def shortcut(self, pts):
        if len(pts) < 3 or self.grid is None:
            return list(pts)
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.clear_segment(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def smooth(self, pts, alpha=0.35, beta=0.35, rounds=60):
        """Pull the polyline towards a curve, refusing any pull into a wall.

        Shortcutting only removes the corners line of sight can see past.
        Where it cannot -- a corridor whose staircase zigzags -- the result is
        still a sawtooth, and a base chasing a lookahead point along a sawtooth
        wags instead of driving. This is the usual smoother: each point is
        drawn back towards where the planner put it and towards the average of
        its neighbours, and any step that would put a point somewhere the map
        does not call free is undone.
        """
        if len(pts) < 3 or self.grid is None:
            return [tuple(p) for p in pts]
        orig = [tuple(p) for p in pts]
        cur = [tuple(p) for p in pts]
        for _ in range(rounds):
            moved = False
            for i in range(1, len(cur) - 1):
                x, y = cur[i]
                nx = x + alpha * (orig[i][0] - x) + \
                    beta * (cur[i - 1][0] + cur[i + 1][0] - 2 * x)
                ny = y + alpha * (orig[i][1] - y) + \
                    beta * (cur[i - 1][1] + cur[i + 1][1] - 2 * y)
                if self.blocked(nx, ny):
                    continue
                if not (self.clear_segment(cur[i - 1], (nx, ny)) and
                        self.clear_segment((nx, ny), cur[i + 1])):
                    continue
                if abs(nx - x) + abs(ny - y) > 1e-4:
                    moved = True
                cur[i] = (nx, ny)
            if not moved:
                break
        return cur

    # -- the dynamic window ------------------------------------------------
    def map_blocked(self, traj, x, y, yaw):
        """Whether each arc crosses something the map already knows about.

        The laser is the authority on what is there now, but it has a minimum
        range and a scan period, and a wall closer than 0.12 m simply is not
        reported. The map does not blink: an arc that leaves the free space
        the planner routed through is rejected on that ground alone.
        """
        c, s = math.cos(yaw), math.sin(yaw)
        pts = traj[:, ::3, :]                    # every third sample is enough
        wx = x + c * pts[:, :, 0] - s * pts[:, :, 1]
        wy = y + s * pts[:, :, 0] + c * pts[:, :, 1]
        bad = np.zeros(traj.shape[0], dtype=bool)
        for i in range(wx.shape[0]):
            for j in range(wx.shape[1]):
                if self.blocked(float(wx[i, j]), float(wy[i, j])):
                    bad[i] = True
                    break
        return bad

    def rollout(self, vs, ws):
        """Arcs for every (v, w) pair, in the base frame. (N, SIM_STEPS, 2)."""
        dt = HORIZON / SIM_STEPS
        k = np.arange(1, SIM_STEPS + 1)[None, :]
        theta = ws[:, None] * dt * k                      # heading at each step
        # Position by summing the per-step displacement along the running
        # heading, which is the same integration the base performs.
        dx = np.cumsum(vs[:, None] * np.cos(theta) * dt, axis=1)
        dy = np.cumsum(vs[:, None] * np.sin(theta) * dt, axis=1)
        return np.stack([dx, dy], axis=2), theta[:, -1]

    def choose(self, goal_base, pose):
        """Best (v, w) for a local goal given in the base frame."""
        v_lo = max(V_MIN, self.v - A_V * CONTROL_DT)
        v_hi = min(V_MAX, self.v + A_V * CONTROL_DT)
        w_lo = max(-W_MAX, self.w - A_W * CONTROL_DT)
        w_hi = min(W_MAX, self.w + A_W * CONTROL_DT)

        vs = np.linspace(v_lo, v_hi, N_V)
        ws = np.linspace(w_lo, w_hi, N_W)
        V, W = np.meshgrid(vs, ws, indexing='ij')
        V, W = V.ravel(), W.ravel()

        traj, _ = self.rollout(V, W)                      # (N, S, 2)

        if self.obstacles.shape[0]:
            d = np.linalg.norm(
                traj[:, :, None, :] - self.obstacles[None, None, :, :], axis=3)
            clearance = d.min(axis=(1, 2))                 # (N,)
        else:
            clearance = np.full(V.shape, np.inf)

        ok = clearance > COLLISION
        ok &= ~self.map_blocked(traj, pose[0], pose[1], pose[2])
        ok |= V < 1e-6           # standing still collides with nothing
        # A stop has to remain possible: never carry more speed into a gap than
        # can be shed before reaching it.
        stoppable = V <= np.sqrt(np.maximum(
            0.0, 2.0 * A_V * np.maximum(0.0, clearance - COLLISION)))
        ok &= stoppable
        if not ok.any():
            return None

        end = traj[:, -1, :]
        to_goal = np.linalg.norm(end - np.asarray(goal_base)[None, :], axis=1)
        here = np.linalg.norm(goal_base)

        progress = (here - to_goal) / max(here, 1e-3)     # 1 = arrives on it
        room = np.minimum(clearance, CLEAR_CAP) / CLEAR_CAP
        speed = V / max(V_MAX, 1e-3)

        score = W_GOAL * progress + W_CLEAR * room + W_SPEED * speed
        score[V < 0.05] -= 0.25          # never prefer sitting still
        score[~ok] = -np.inf
        best = int(np.argmax(score))
        if self.debug:
            self.debug = False
            near = float(np.linalg.norm(self.obstacles, axis=1).min()) \
                if self.obstacles.shape[0] else float('inf')
            self.get_logger().info(
                'window v[%.2f,%.2f] w[%.2f,%.2f] obs=%d nearest=%.2f '
                'ok=%d/%d clear[min=%.2f max=%.2f] picked v=%.2f w=%.2f '
                'goal=(%.2f,%.2f)'
                % (v_lo, v_hi, w_lo, w_hi, self.obstacles.shape[0], near,
                   int(ok.sum()), ok.size, float(clearance.min()),
                   float(clearance.max()), float(V[best]), float(W[best]),
                   goal_base[0], goal_base[1]))
        return float(V[best]), float(W[best]), traj[best]

    def recover(self):
        """No admissible arc. Turn towards the side with more room."""
        if self.obstacles.shape[0] == 0:
            return 0.0, 0.6
        x, y = self.obstacles[:, 0], self.obstacles[:, 1]
        ahead = x > -0.05
        front = np.linalg.norm(self.obstacles[ahead], axis=1).min() \
            if ahead.any() else np.inf
        left = y > 0
        room_left = np.linalg.norm(self.obstacles[left], axis=1).min() \
            if left.any() else np.inf
        room_right = np.linalg.norm(self.obstacles[~left], axis=1).min() \
            if (~left).any() else np.inf
        turn = 1.0 if room_left > room_right else -1.0
        back = -0.09 if front < COLLISION + 0.05 else 0.0
        return back, turn * 1.2

    # -- control ----------------------------------------------------------
    def pose(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_footprint',
                                             rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def local_goal(self, x, y):
        """The point LOOKAHEAD along the path from the nearest point on it.

        Advancing by proximity alone lets the base orbit: a waypoint it keeps
        missing by half a metre is never dropped, and the goal swings around
        it. Projecting onto the polyline instead means progress is measured
        along the path, so passing a corner retires it whether or not the
        corner was driven over.
        """
        pts = self.waypoints
        if len(pts) == 1:
            return pts[0]

        best_seg, best_t, best_d = 0, 0.0, float('inf')
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            t = 0.0 if length2 == 0 else \
                max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
            px, py = ax + t * dx, ay + t * dy
            d = math.dist((x, y), (px, py))
            if d < best_d:
                best_seg, best_t, best_d = i, t, d

        # Everything before the projection is behind us.
        if best_seg > 0:
            self.waypoints = pts[best_seg:]
            pts = self.waypoints
            best_seg = 0

        # Walk LOOKAHEAD forward along what is left.
        ax, ay = pts[0]
        bx, by = pts[1]
        cursor = (ax + best_t * (bx - ax), ay + best_t * (by - ay))
        remaining = LOOKAHEAD
        for i in range(len(pts) - 1):
            seg_start = cursor if i == 0 else pts[i]
            seg_end = pts[i + 1]
            seg = math.dist(seg_start, seg_end)
            if seg >= remaining:
                if seg == 0:
                    return seg_end
                s = remaining / seg
                return (seg_start[0] + s * (seg_end[0] - seg_start[0]),
                        seg_start[1] + s * (seg_end[1] - seg_start[1]))
            remaining -= seg
        return pts[-1]

    def tick(self):
        here = self.pose()
        if here is None:
            return
        x, y, yaw = here
        self.show(x, y)

        if not self.waypoints:
            self.cmd.publish(Twist())
            return

        goal = self.local_goal(x, y)
        last = self.waypoints[-1]
        if math.dist((x, y), last) < ARRIVED_WITHIN:
            self.waypoints = []
            self.cmd.publish(Twist())
            if not self.announced:
                self.arrived.publish(Bool(data=True))
                self.announced = True
                self.get_logger().info('arrived')
            return

        # The goal, in the frame the window is sampled in.
        dx, dy = goal[0] - x, goal[1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        goal_base = (c * dx - s * dy, s * dx + c * dy)

        picked = self.choose(goal_base, (x, y, yaw))
        msg = Twist()
        if picked is None:
            self.recoveries += 1
            msg.linear.x, msg.angular.z = self.recover()
        else:
            v, w, traj = picked
            bearing = math.atan2(goal_base[1], goal_base[0])
            # Facing away from the goal, no forward arc scores well; turn on
            # the spot rather than swinging wide through whatever is beside us.
            if abs(bearing) > 1.6:
                v, w = 0.0, math.copysign(1.4, bearing)
            msg.linear.x, msg.angular.z = v, w
            self.publish_path(self.chosen,
                              [(x + c * px + s * py, y - s * px + c * py)
                               for px, py in traj], 'map')
        self.v, self.w = msg.linear.x, msg.angular.z
        self.cmd.publish(msg)

    # -- output ------------------------------------------------------------
    def publish_path(self, pub, pts, frame):
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y = float(px), float(py)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)

    def show(self, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'robot', 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.3
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.8, y=0.8, z=0.8)
        m.color = ColorRGBA(r=1.0, g=0.35, b=0.35, a=1.0)
        self.marker.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Follower())


if __name__ == '__main__':
    main()
