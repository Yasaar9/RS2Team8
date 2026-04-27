#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 2: Motion Planning and Control
Author: Jerry Sun

Navigation node that receives a waypoint (POI name or raw coordinates)
and drives the TurtleBot to it using Nav2 + Regulated Pure Pursuit controller.

Topics consumed:
  /navigation/go_to_waypoint  (std_msgs/String)           — POI name e.g. "artifact_1"
  /navigation/go_to_pose      (geometry_msgs/PoseStamped) — raw coordinate goal
  /navigation/cancel          (std_msgs/String)           — cancels current goal

Topics published:
  /navigation/status           (std_msgs/String) — "IDLE"|"NAVIGATING"|"OBSTACLE"|"REACHED"|"FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True when goal is reached
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import threading
import math
import time


# ===========================================================================
# INITIAL POSE CONFIGURATION
# Update once the gallery map is finalised.
# ===========================================================================
INITIAL_POSE_X   = 0.0
INITIAL_POSE_Y   = 0.0
INITIAL_POSE_YAW = 0.0
# ===========================================================================


# ===========================================================================
# WAYPOINT CONFIGURATION  —  "name": (x, y, yaw_degrees)
# Update x/y once the gallery map is finalised (ros2 topic echo /odom).
# ===========================================================================
WAYPOINTS = {
    "home":       (0.0,   0.0,   0.0),
    "artifact_1": (1.5,   0.5,   0.0),
    "artifact_2": (3.0,   1.0,   0.0),
    "artifact_3": (2.5,  -1.5,   0.0),
    "artifact_4": (0.5,  -2.0,   0.0),
    "toilets":    (4.0,   2.0,   0.0),
    "entrance":   (-0.5,  0.0,   0.0),
}
# ===========================================================================


# ===========================================================================
# ARRIVAL CONFIGURATION
#
# XY_ARRIVAL_THRESHOLD — straight-line distance (metres) at which we declare
#   arrival and cancel the Nav2 action ourselves. We do this rather than rely
#   on Nav2's isTaskComplete() because distance_remaining reports arc-length,
#   not Euclidean distance, and never converges on TurtleBot3 + RPP.
#   Rule of thumb: ~half the robot footprint radius (TurtleBot3 ~0.10 m).
#
# AMCL_TIMEOUT_SEC — if /amcl_pose hasn't updated for this many seconds
#   (robot stopped, filter not converging) we fall back to /odom for the
#   distance check to avoid the arrival loop hanging indefinitely.
# ===========================================================================
XY_ARRIVAL_THRESHOLD = 0.15   # metres
AMCL_TIMEOUT_SEC     = 1.0    # seconds before falling back to /odom
# ===========================================================================


# ===========================================================================
# OBSTACLE DETECTION CONFIGURATION
#
# OBSTACLE_STOP_DIST  — minimum range (metres) in the forward arc that
#   triggers an emergency stop. Matches the D-grade requirement of 0.5 m.
#
# OBSTACLE_CLEAR_DIST — range at which the path is considered clear again.
#   Slightly larger than STOP to add hysteresis and prevent rapid on/off.
#
# OBSTACLE_ARC_DEG    — total forward arc (degrees) scanned for obstacles.
#   +/- 30 deg (60 deg total) covers the robot's likely travel corridor.
# ===========================================================================
OBSTACLE_STOP_DIST  = 0.5   # metres
OBSTACLE_CLEAR_DIST = 0.6   # metres
OBSTACLE_ARC_DEG    = 60    # degrees total forward arc
# ===========================================================================


def yaw_to_quaternion(yaw_deg: float) -> tuple:
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class NavigationNode(Node):

    def __init__(self):
        super().__init__("navigation_node")

        self.navigator = BasicNavigator()
        self._set_initial_pose()

        # ── Publishers ───────────────────────────────────────────────────────
        self.status_pub  = self.create_publisher(String, "/navigation/status", 10)
        self.reached_pub = self.create_publisher(Bool,   "/navigation/waypoint_reached", 10)
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel", 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,      "/navigation/go_to_waypoint", self._waypoint_name_callback, 10)
        self.create_subscription(PoseStamped, "/navigation/go_to_pose",     self._pose_callback,          10)
        self.create_subscription(String,      "/navigation/cancel",         self._cancel_callback,        10)

        # ── Pose tracking (AMCL primary, /odom fallback) ─────────────────────
        # /amcl_pose is in the map frame — globally consistent — but AMCL may
        # stop publishing when the robot is stationary. /odom is in the odom
        # frame — drifts over time — but updates continuously. We prefer AMCL
        # and fall back to odom only if AMCL has been silent for AMCL_TIMEOUT_SEC.
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._pose_lock = threading.Lock()
        self._last_amcl_time: float = 0.0

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10,
        )

        # ── Obstacle detection ───────────────────────────────────────────────
        # Subscribes to /scan and checks the forward arc for obstacles.
        # _obstacle_blocked is set True when something is within OBSTACLE_STOP_DIST
        # and cleared when the path opens beyond OBSTACLE_CLEAR_DIST.
        # The navigation thread pauses while blocked and resumes automatically.
        self._obstacle_blocked: bool = False
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            10,
        )

        # ── Internal state ───────────────────────────────────────────────────
        self._navigating = False
        self._nav_thread: threading.Thread | None = None

        self._publish_status("IDLE")
        self.get_logger().info("NavigationNode ready. Waiting for Nav2...")

        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active — ready to navigate.")

    # ── Pose tracking ────────────────────────────────────────────────────────

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._pose_lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
        self._last_amcl_time = time.time()

    def _odom_callback(self, msg: Odometry):
        # Only use odometry if AMCL has not published recently.
        if time.time() - self._last_amcl_time > AMCL_TIMEOUT_SEC:
            with self._pose_lock:
                self._robot_x = msg.pose.pose.position.x
                self._robot_y = msg.pose.pose.position.y

    def _distance_to_goal(self, goal_x: float, goal_y: float) -> float:
        with self._pose_lock:
            dx = self._robot_x - goal_x
            dy = self._robot_y - goal_y
        return math.hypot(dx, dy)

    # ── Obstacle detection ───────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        """
        Check the forward arc of the laser scan for obstacles.

        TurtleBot3 LaserScan: 360 readings, index 0 = directly forward,
        increasing counter-clockwise. We check +/- (OBSTACLE_ARC_DEG / 2)
        degrees around index 0.

        Uses hysteresis: stops at OBSTACLE_STOP_DIST, resumes at
        OBSTACLE_CLEAR_DIST to prevent rapid toggling.
        """
        num = len(msg.ranges)
        if num == 0:
            return

        half_arc = int((OBSTACLE_ARC_DEG / 2.0) / 360.0 * num)

        # Forward arc: last half_arc readings + first half_arc readings
        indices = list(range(num - half_arc, num)) + list(range(0, half_arc + 1))
        valid = [
            msg.ranges[i]
            for i in indices
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]

        if not valid:
            return

        min_dist = min(valid)

        if not self._obstacle_blocked and min_dist < OBSTACLE_STOP_DIST:
            self._obstacle_blocked = True
            self.get_logger().warn(
                f"[OBSTACLE] Obstacle detected at {min_dist:.2f} m — emergency stop."
            )
            self._stop_robot()
            if self._navigating:
                self._publish_status("OBSTACLE")

        elif self._obstacle_blocked and min_dist > OBSTACLE_CLEAR_DIST:
            self._obstacle_blocked = False
            self.get_logger().info(
                f"[OBSTACLE] Path cleared ({min_dist:.2f} m) — resuming navigation."
            )
            if self._navigating:
                self._publish_status("NAVIGATING")

    # ── Initial pose ─────────────────────────────────────────────────────────

    def _set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp    = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(INITIAL_POSE_YAW)
        initial_pose.pose.position.x    = INITIAL_POSE_X
        initial_pose.pose.position.y    = INITIAL_POSE_Y
        initial_pose.pose.position.z    = 0.0
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(
            f"Initial pose set: ({INITIAL_POSE_X}, {INITIAL_POSE_Y}, {INITIAL_POSE_YAW}°)"
        )

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _waypoint_name_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in WAYPOINTS:
            self.get_logger().warn(f"Unknown waypoint '{name}'. Available: {list(WAYPOINTS.keys())}")
            return
        x, y, yaw = WAYPOINTS[name]
        self.get_logger().info(f"Received waypoint name: '{name}' -> ({x}, {y}, {yaw}deg)")
        self._navigate_to(make_pose(x, y, yaw), label=name, goal_x=x, goal_y=y)

    def _pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Received raw pose goal: ({x:.2f}, {y:.2f})")
        msg.header.frame_id = "map"
        self._navigate_to(msg, label="raw_pose", goal_x=x, goal_y=y)

    def _cancel_callback(self, _msg: String):
        self.get_logger().info("Cancel received.")
        self.navigator.cancelTask()
        self._stop_robot()
        self._navigating = False
        self._publish_status("IDLE")

    # ── Navigation logic ─────────────────────────────────────────────────────

    def _navigate_to(self, pose: PoseStamped, label: str, goal_x: float, goal_y: float):
        if self._navigating:
            self.get_logger().warn("Already navigating — cancelling current goal first.")
            self.navigator.cancelTask()
            self._stop_robot()

        pose.header.stamp = self.get_clock().now().to_msg()
        self._navigating  = True
        self._nav_thread  = threading.Thread(
            target=self._navigation_thread,
            args=(pose, label, goal_x, goal_y),
            daemon=True,
        )
        self._nav_thread.start()

    def _navigation_thread(self, pose: PoseStamped, label: str, goal_x: float, goal_y: float):
        """
        Background thread. Sends the Nav2 goal then polls pose until the robot
        is within XY_ARRIVAL_THRESHOLD, then cancels the Nav2 action and
        declares REACHED. Also handles Nav2 reporting task completion early
        (e.g. FAILED due to a blocked path).

        When _obstacle_blocked is True, the loop publishes zero velocity and
        pauses. Nav2 continues to hold the goal but receives no cmd_vel from
        the controller while we are stopping it. When the obstacle clears,
        the loop resumes and Nav2 takes back over driving.
        """
        self._publish_status("NAVIGATING")
        self.get_logger().info(f"[NAV] Navigating to '{label}'...")

        self.navigator.goToPose(pose)

        last_log_time = 0.0

        while True:
            now = time.time()

            # ── Obstacle hold ─────────────────────────────────────────────
            if self._obstacle_blocked:
                self._stop_robot()
                time.sleep(0.1)
                continue

            dist = self._distance_to_goal(goal_x, goal_y)

            # ── Periodic progress log ─────────────────────────────────────
            if now - last_log_time > 2.0:
                feedback = self.navigator.getFeedback()
                arc_dist = feedback.distance_remaining if feedback else float("nan")
                using_odom = (now - self._last_amcl_time) > AMCL_TIMEOUT_SEC
                self.get_logger().info(
                    f"[NAV] '{label}': straight={dist:.3f} m  "
                    f"path={arc_dist:.3f} m  (threshold={XY_ARRIVAL_THRESHOLD:.2f} m)"
                    + ("  [odom fallback]" if using_odom else "")
                )
                last_log_time = now

            # ── Arrival check (primary) ───────────────────────────────────
            if dist <= XY_ARRIVAL_THRESHOLD:
                self.get_logger().info(
                    f"[NAV] '{label}': arrived at {dist:.3f} m — declaring REACHED."
                )
                self.navigator.cancelTask()
                self._stop_robot()
                self._navigating = False
                self._publish_status("REACHED")
                self._publish_reached(True)
                return

            # ── Nav2 failure check (secondary) ───────────────────────────
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f"[NAV] '{label}': Nav2 SUCCEEDED at dist={dist:.3f} m — accepting."
                    )
                    self._stop_robot()
                    self._navigating = False
                    self._publish_status("REACHED")
                    self._publish_reached(True)
                else:
                    self.get_logger().warn(
                        f"[NAV] '{label}': Nav2 ended with result={result} — declaring FAILED."
                    )
                    self._stop_robot()
                    self._navigating = False
                    self._publish_status("FAILED")
                    self._publish_reached(False)
                return

            time.sleep(0.1)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _stop_robot(self):
        """Publish a zero-velocity Twist to halt the robot immediately."""
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_reached(self, reached: bool):
        msg = Bool()
        msg.data = reached
        self.reached_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.cancelTask()
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()