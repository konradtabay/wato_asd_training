#!/usr/bin/env python3
"""Navigation stress scenarios for the ASD stack.

Each scenario teleports the robot to a hard starting pose, sends a goal on /goal_point,
and checks that the robot reaches it without stalling, touching an obstacle, or tipping.

Runs inside the gazebo container (it needs `ign` for teleporting and rclpy for topics).
Use tests/run_nav_scenarios.sh from the repo root with the stack already up.
"""

import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

# Arena geometry from src/gazebo/launch/robot_env.sdf: (center_x, center_y, half_size).
BOXES = [(7, -6, 1.5), (-8, 6, 1.5), (4, 9, 1.5), (9, 3, 1.0), (1, -10, 1.0)]
WALL_INNER = 14.75

# Robot geometry, relative to the chassis centre (x forward, y left).
LIDAR_AHEAD = 0.8      # /odom/filtered is the lidar, 0.8 m ahead of the chassis centre
AXLE_BEHIND = 0.5      # Gazebo model origin sits on the wheel axle
CHASSIS = (-1.0, 1.0, -0.5, 0.5)
WHEELS = [(-0.9, -0.1, 0.5, 0.7), (-0.9, -0.1, -0.7, -0.5)]

GOAL_RADIUS = 0.6      # chassis centre within this of the goal counts as reached
STALL_SEC = 12.0
MIN_CLEARANCE = 0.10
MAX_TILT_DEG = 15.0

# name: (start chassis centre x, y, yaw_deg), goal (x, y), timeout_sec, why it is hard
SCENARIOS = {
    "corridor_uturn": ((1.0, -12.9, 0), (-6.0, -12.9), 90,
                       "180 deg turn inside the 3.75 m corridor under box (1,-10)"),
    "box_hairpin": ((1.0, -12.9, 180), (1.0, -7.2), 90,
                    "goal directly behind box (1,-10); must wrap around it"),
    "diagonal_gap": ((6.0, -12.8, 90), (2.8, -6.5), 90,
                     "3.8 m corner-to-corner gap between boxes (1,-10) and (7,-6)"),
    "corner_dive": ((-9.0, -9.0, -135), (-13.2, -13.2), 60,
                    "drive into the south-west arena corner"),
    "corner_escape": ((-13.2, -13.2, -135), (-9.0, -9.0), 90,
                      "nose in the arena corner, goal behind; rotate between two walls"),
    "north_gap_hairpin": ((0.0, 12.6, 0), (4.0, 6.0), 90,
                          "4.25 m gap above box (4,9), then U-turn to its south side"),
    "box_squeeze": ((11.5, 7.5, 180), (6.0, 4.5), 90,
                    "4.3 m diagonal gap between boxes (9,3) and (4,9)"),
    "goal_by_box": ((-3.0, 6.0, 180), (-5.3, 6.0), 60,
                    "goal 1.2 m from box (-8,6), just outside the lethal zone"),
    "fast_corner": ((-12.0, -1.0, 90), (-4.5, 9.0), 90,
                    "turn around box (-8,6) corner after a straight at speed"),
    "long_straight": ((-11.0, -1.0, 0), (11.0, -1.0), 90,
                      "22 m straight at full speed, stop 3.75 m before the east wall"),
}


def point_clearance(x, y):
    best = WALL_INNER - max(abs(x), abs(y))
    for cx, cy, h in BOXES:
        dx = max(abs(x - cx) - h, 0.0)
        dy = max(abs(y - cy) - h, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def rect_perimeter(x0, x1, y0, y1, step=0.05):
    pts = []
    nx = int(round((x1 - x0) / step))
    ny = int(round((y1 - y0) / step))
    for i in range(nx + 1):
        x = x0 + i * step
        pts += [(x, y0), (x, y1)]
    for j in range(ny + 1):
        y = y0 + j * step
        pts += [(x0, y), (x1, y)]
    return pts


BODY_POINTS = rect_perimeter(*CHASSIS) + [p for w in WHEELS for p in rect_perimeter(*w)]


def body_clearance(cx, cy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return min(point_clearance(cx + bx * c - by * s, cy + bx * s + by * c)
               for bx, by in BODY_POINTS)


def teleport(cx, cy, yaw):
    mx = cx - AXLE_BEHIND * math.cos(yaw)
    my = cy - AXLE_BEHIND * math.sin(yaw)
    req = ('name: "robot", position: {x: %f, y: %f, z: 0.05}, '
           'orientation: {x: 0, y: 0, z: %f, w: %f}'
           % (mx, my, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
    subprocess.run(
        ["ign", "service", "-s", "/world/sim_world/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--timeout", "3000", "--req", req],
        check=True, capture_output=True)


class Harness(Node):
    def __init__(self):
        super().__init__("nav_scenarios")
        self.odom = None
        self.create_subscription(Odometry, "/odom/filtered", self._on_odom, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

    def _on_odom(self, msg):
        self.odom = msg

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        """Chassis centre (x, y, yaw) and tilt in radians."""
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        cx = p.x - LIDAR_AHEAD * math.cos(yaw)
        cy = p.y - LIDAR_AHEAD * math.sin(yaw)
        return cx, cy, yaw, max(abs(roll), abs(pitch))

    def send_goal(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = "sim_world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = x, y
        self.goal_pub.publish(msg)

    def stop_robot(self):
        # Goal at the current pose puts the planner back to idle, and an empty path makes
        # the controller publish zero velocity, so nothing drives during the teleport.
        cx, cy, _, _ = self.pose()
        self.send_goal(cx, cy)
        self.spin(1.0)
        empty = Path()
        empty.header.frame_id = "sim_world"
        for _ in range(5):
            self.path_pub.publish(empty)
            self.spin(0.2)

    def place(self, cx, cy, yaw):
        self.stop_robot()
        teleport(cx, cy, yaw)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.spin(0.2)
            x, y, _, _ = self.pose()
            if math.hypot(x - cx, y - cy) < 0.3:
                break
        # Let map memory merge a scan from the new pose before planning.
        self.spin(2.0)

    def run(self, name):
        (sx, sy, syaw_deg), (gx, gy), timeout, _ = SCENARIOS[name]
        syaw = math.radians(syaw_deg)
        self.place(sx, sy, syaw)

        t0 = time.time()
        self.send_goal(gx, gy)
        min_clear, max_tilt, top_speed = 99.0, 0.0, 0.0
        cx, cy, yaw, _ = self.pose()
        last_progress_t, last_pose = t0, (cx, cy, yaw)
        prev_xy, prev_t = (cx, cy), t0
        result = "TIMEOUT"

        while time.time() - t0 < timeout:
            self.spin(0.2)
            now = time.time()
            cx, cy, yaw, tilt = self.pose()
            min_clear = min(min_clear, body_clearance(cx, cy, yaw))
            max_tilt = max(max_tilt, tilt)
            top_speed = max(top_speed, math.hypot(cx - prev_xy[0], cy - prev_xy[1]) /
                            max(now - prev_t, 1e-3))
            prev_xy, prev_t = (cx, cy), now

            moved = math.hypot(cx - last_pose[0], cy - last_pose[1])
            turned = abs(math.atan2(math.sin(yaw - last_pose[2]), math.cos(yaw - last_pose[2])))
            if moved > 0.15 or turned > 0.2:
                last_progress_t, last_pose = now, (cx, cy, yaw)

            if math.hypot(cx - gx, cy - gy) < GOAL_RADIUS:
                result = "REACHED"
                break
            if now - last_progress_t > STALL_SEC:
                result = "STALLED"
                break

        elapsed = time.time() - t0
        passed = (result == "REACHED" and min_clear >= MIN_CLEARANCE and
                  math.degrees(max_tilt) < MAX_TILT_DEG)
        return {
            "name": name, "passed": passed, "result": result, "time": elapsed,
            "clearance": min_clear, "tilt": math.degrees(max_tilt), "speed": top_speed,
            "end": (cx, cy), "goal_dist": math.hypot(cx - gx, cy - gy),
        }


def main():
    args = sys.argv[1:]
    if args == ["--list"]:
        for name, (_, _, _, why) in SCENARIOS.items():
            print("%-18s %s" % (name, why))
        return 0

    names = args or list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        print("unknown scenario(s): %s (use --list)" % ", ".join(unknown))
        return 2

    rclpy.init()
    h = Harness()
    h.spin(2.0)
    if h.odom is None:
        print("no /odom/filtered; is the robot stack up?")
        return 2

    results = []
    for name in names:
        r = h.run(name)
        results.append(r)
        print("%-18s %-4s %-8s %5.1fs  clearance %5.2f m  tilt %4.1f deg  top %4.2f m/s  "
              "end (%.1f, %.1f) %.2f m from goal" % (
                  r["name"], "PASS" if r["passed"] else "FAIL", r["result"], r["time"],
                  r["clearance"], r["tilt"], r["speed"], r["end"][0], r["end"][1],
                  r["goal_dist"]), flush=True)

    h.stop_robot()
    failed = [r["name"] for r in results if not r["passed"]]
    print("\n%d/%d passed" % (len(results) - len(failed), len(results)))
    if failed:
        print("failed: " + ", ".join(failed))
    h.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
