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
  /artifact_goto              (std_msgs/String)           — UI "Go To" button

Topics published:
  /navigation/status           (std_msgs/String) — "IDLE"|"NAVIGATING"|"OBSTACLE"|"REACHED"|"FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True when goal is reached
  /navigation_status           (std_msgs/String) — mirror of above for UI node

Debug output (every 3 s):
  [DEBUG] odom=(...) amcl=(...) source=amcl|odom  amcl_age=...s
  [DEBUG] obstacle: forward_min=...m  blocked=True/False
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
# Positions measured using a 0.5m radius cylinder in Gazebo world frame,
# then converted to map frame by adding the spawn offset:
#   map_x = world_x + 2.0  (spawn is at world -2.0, map 0.0)
#   map_y = world_y + 0.5  (spawn is at world -0.5, map 0.0)
# Yaw set so robot faces point of interest on arrival.
# ROS yaw convention: 0=East(+X), 90=North(+Y), 180=West, -90=South(-Y)
# ===========================================================================
WAYPOINTS = {
    "home":        ( 0.000,  0.000,    0.0),
    "artifact_1":  ( 1.029,  2.240, -135.0),  # faces SW toward artifact
    "artifact_2":  ( 2.963,  2.253,  135.0),  # faces NW toward artifact
    "artifact_3":  ( 2.950, -1.261,   45.0),  # faces NE toward artifact
    "artifact_4":  ( 1.030, -1.237,  -45.0),  # faces SE toward artifact
    "toilets":     ( 3.819,  0.485,   90.0),  # faces North toward toilets
    "fire_exit_1": ( 1.994,  2.502,  180.0),  # faces West toward exit
    "fire_exit_2": ( 1.987, -1.528,    0.0),  # faces East toward exit
}
# ===========================================================================


# ===========================================================================
# FIRE EXIT LIST — used by nearest fire exit logic.
# Add any additional fire exit waypoint names here.
# ===========================================================================
FIRE_EXITS = ["fire_exit_1", "fire_exit_2"]
# ===========================================================================


# ===========================================================================
# UI LABEL -> WAYPOINT NAME MAP
# Maps the display names from the UI "Go To" buttons to WAYPOINTS keys above.
# ===========================================================================
UI_TO_WAYPOINT = {
    "modern art":          "artifact_1",
    "sculpture":           "artifact_2",
    "portrait":            "artifact_3",
    "historical artefact": "artifact_4",
    "toilet":              "toilets",
    "fire exit":           "nearest_fire_exit",   # resolved dynamically at runtime
}
# ===========================================================================


# ===========================================================================
# ARRIVAL CONFIGURATION
#
# XY_ARRIVAL_THRESHOLD — straight-line distance (metres) at which we declare
#   arrival and cancel the Nav2 action ourselves. We do this rather than rely
#   on Nav2's isTaskComplete() because distance_remaining reports arc-length,
#   not Euclidean distance, and never converges on TurtleBot3 + RPP.
# ===========================================================================
XY_ARRIVAL_THRESHOLD = 0.15   # metres
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
#   60 deg total (+/- 30 deg) covers the robot's travel corridor.
# ===========================================================================
OBSTACLE_STOP_DIST  = 0.35   # metres  (D-grade: emergency stop within 0.5m)
OBSTACLE_CLEAR_DIST = 0.5  # metres  (slightly larger for hysteresis)
OBSTACLE_ARC_DEG    = 60    # degrees total forward arc
# ===========================================================================


# ===========================================================================
# DEBUG CONFIGURATION
#
# DEBUG_POSE     — logs coordinate comparison (odom vs amcl) every
#                  DEBUG_INTERVAL_SEC seconds.
# DEBUG_OBSTACLE — logs forward scan min distance every DEBUG_INTERVAL_SEC
#                  seconds so you can verify obstacle detection is reading
#                  the right values before an object is placed.
#
# Set both to False for real-robot deployment.
# ===========================================================================
DEBUG_POSE         = True
DEBUG_OBSTACLE     = True
DEBUG_INTERVAL_SEC = 3.0
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
        self.status_pub    = self.create_publisher(String, "/navigation/status",       10)
        self.ui_status_pub = self.create_publisher(String, "/navigation_status",       10)  # UI mirror
        self.reached_pub   = self.create_publisher(Bool,   "/navigation/waypoint_reached", 10)
        self.cmd_vel_pub   = self.create_publisher(Twist,  "/cmd_vel",                 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,      "/navigation/go_to_waypoint", self._waypoint_name_callback, 10)
        self.create_subscription(PoseStamped, "/navigation/go_to_pose",     self._pose_callback,          10)
        self.create_subscription(String,      "/navigation/cancel",         self._cancel_callback,        10)
        self.create_subscription(String,      "/artifact_goto",             self._ui_goto_callback,       10)

        # ── Pose tracking ────────────────────────────────────────────────────
        # Strategy: AMCL is the primary source (map frame, globally consistent).
        # Odom is only used at startup BEFORE AMCL has ever published.
        # Once AMCL publishes even once, we freeze on the last known AMCL value
        # rather than falling back to drifting odom near the goal — odom error
        # near the goal exceeds XY_ARRIVAL_THRESHOLD and causes false misses.
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._pose_lock = threading.Lock()

        # Track whether AMCL has ever published (not just recency)
        self._amcl_ever_received: bool = False
        self._last_amcl_time: float    = 0.0

        # Separate stores for debug comparison
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_lock = threading.Lock()

        self._amcl_x: float = 0.0
        self._amcl_y: float = 0.0
        self._amcl_lock = threading.Lock()

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
        self._obstacle_blocked: bool    = False
        self._obstacle_min_dist: float  = float("inf")  # for debug
        self._scan_lock = threading.Lock()
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)

        # ── Debug timer ──────────────────────────────────────────────────────
        if DEBUG_POSE or DEBUG_OBSTACLE:
            self.create_timer(DEBUG_INTERVAL_SEC, self._debug_log)

        # ── Internal state ───────────────────────────────────────────────────
        self._navigating = False
        self._nav_thread: threading.Thread | None = None

        self._publish_status("IDLE")
        self.get_logger().info("NavigationNode ready. Waiting for Nav2...")

        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active — ready to navigate.")

    # ── Pose tracking ────────────────────────────────────────────────────────

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._amcl_lock:
            self._amcl_x = x
            self._amcl_y = y
        # Always update active pose from AMCL — it is the authoritative source
        with self._pose_lock:
            self._robot_x = x
            self._robot_y = y
        self._amcl_ever_received = True
        self._last_amcl_time = time.time()

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._odom_lock:
            self._odom_x = x
            self._odom_y = y
        # Only promote odom to active source before AMCL has ever published.
        # After the first AMCL message we freeze on last known AMCL value —
        # never replace it with drifting odom, even if AMCL goes quiet.
        if not self._amcl_ever_received:
            with self._pose_lock:
                self._robot_x = x
                self._robot_y = y

    def _distance_to_goal(self, goal_x: float, goal_y: float) -> float:
        with self._pose_lock:
            dx = self._robot_x - goal_x
            dy = self._robot_y - goal_y
        return math.hypot(dx, dy)

    # ── Debug logging ─────────────────────────────────────────────────────────

    def _debug_log(self):
        if DEBUG_POSE:
            with self._odom_lock:
                ox, oy = self._odom_x, self._odom_y
            with self._amcl_lock:
                ax, ay = self._amcl_x, self._amcl_y
            with self._pose_lock:
                rx, ry = self._robot_x, self._robot_y

            amcl_age = time.time() - self._last_amcl_time if self._amcl_ever_received else float("inf")
            source   = "amcl" if self._amcl_ever_received else "odom(pre-amcl)"

            # Warn if odom and amcl are far apart — indicates localisation drift
            odom_amcl_err = math.hypot(ox - ax, oy - ay) if self._amcl_ever_received else float("nan")
            err_str = f"{odom_amcl_err:.3f} m"
            if self._amcl_ever_received and odom_amcl_err > 0.5:
                err_str += "  <-- large odom/amcl divergence"

            self.get_logger().info(
                f"\n[DEBUG POSE]"
                f"\n  odom   : ({ox:+.3f}, {oy:+.3f})  (odom frame — drifts)"
                f"\n  amcl   : ({ax:+.3f}, {ay:+.3f})  (map frame  — used for nav)"
                f"\n  active : {source} -> ({rx:+.3f}, {ry:+.3f})"
                f"\n  amcl age : {amcl_age:.1f} s"
                f"\n  odom/amcl error : {err_str}"
            )

        if DEBUG_OBSTACLE:
            with self._scan_lock:
                min_d   = self._obstacle_min_dist
                blocked = self._obstacle_blocked
            self.get_logger().info(
                f"[DEBUG OBSTACLE]  forward_min={min_d:.2f} m  "
                f"stop_threshold={OBSTACLE_STOP_DIST} m  "
                f"blocked={blocked}"
            )

    # ── Obstacle detection ───────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        """
        Check the forward arc of the laser scan for obstacles.

        TurtleBot3 waffle_pi LaserScan: 360 readings, index 0 = directly
        forward, increasing counter-clockwise. We check +/- 30 deg (60 deg
        total) around index 0.

        Uses hysteresis: stops at OBSTACLE_STOP_DIST, resumes only when
        clear beyond OBSTACLE_CLEAR_DIST to prevent rapid toggling.
        """
        num = len(msg.ranges)
        if num == 0:
            return

        half_arc = int((OBSTACLE_ARC_DEG / 2.0) / 360.0 * num)
        indices  = list(range(num - half_arc, num)) + list(range(0, half_arc + 1))
        valid    = [
            msg.ranges[i]
            for i in indices
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]

        if not valid:
            return

        min_dist = min(valid)

        with self._scan_lock:
            self._obstacle_min_dist = min_dist

        if not self._obstacle_blocked and min_dist < OBSTACLE_STOP_DIST:
            self._obstacle_blocked = True
            self.get_logger().warn(
                f"[OBSTACLE] Obstacle at {min_dist:.2f} m — emergency stop."
            )
            self._stop_robot()
            if self._navigating:
                self._publish_status("OBSTACLE")

        elif self._obstacle_blocked and min_dist > OBSTACLE_CLEAR_DIST:
            self._obstacle_blocked = False
            self.get_logger().info(
                f"[OBSTACLE] Path cleared ({min_dist:.2f} m) — resuming."
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
            self.get_logger().warn(
                f"Unknown waypoint '{name}'. Available: {list(WAYPOINTS.keys())}"
            )
            return
        x, y, yaw = WAYPOINTS[name]
        self.get_logger().info(f"Received waypoint: '{name}' -> ({x}, {y}, {yaw}deg)")
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

    def _nearest_fire_exit(self) -> str:
        """
        Returns the name of the closest fire exit waypoint to the robot's
        current AMCL position. Compares straight-line distance to all entries
        in FIRE_EXITS and returns the nearest one.
        """
        with self._pose_lock:
            rx, ry = self._robot_x, self._robot_y

        best_name = FIRE_EXITS[0]
        best_dist = float("inf")
        for name in FIRE_EXITS:
            ex, ey, _ = WAYPOINTS[name]
            dist = math.hypot(rx - ex, ry - ey)
            self.get_logger().info(
                f"[FIRE EXIT] {name}: distance={dist:.2f} m from robot ({rx:.2f}, {ry:.2f})"
            )
            if dist < best_dist:
                best_dist = dist
                best_name = name

        self.get_logger().info(f"[FIRE EXIT] Nearest exit: {best_name} ({best_dist:.2f} m)")
        return best_name

    def _ui_goto_callback(self, msg: String):
        """Receives 'Go To' button presses from the UI and maps to waypoints."""
        label    = msg.data.strip().lower()
        waypoint = UI_TO_WAYPOINT.get(label)
        if waypoint is None:
            self.get_logger().warn(
                f"[UI] No waypoint mapped for '{label}'. "
                f"Available UI labels: {list(UI_TO_WAYPOINT.keys())}"
            )
            return

        # Resolve nearest fire exit dynamically at request time
        if waypoint == "nearest_fire_exit":
            waypoint = self._nearest_fire_exit()

        self.get_logger().info(f"[UI] Go To '{label}' -> waypoint '{waypoint}'")
        x, y, yaw = WAYPOINTS[waypoint]
        self._navigate_to(make_pose(x, y, yaw), label=waypoint, goal_x=x, goal_y=y)

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
        Background thread. Sends the Nav2 goal then polls AMCL pose until the
        robot is within XY_ARRIVAL_THRESHOLD, then cancels and declares REACHED.

        Pose source:
          - Before first AMCL message: uses raw odom (startup only)
          - After first AMCL message:  always uses last known AMCL value,
            even if AMCL goes quiet near the goal. Never falls back to odom
            once AMCL has been received — odom drift exceeds arrival threshold.

        Obstacle behaviour:
          - When blocked: holds zero velocity, Nav2 keeps the goal active.
          - When cleared: resumes — Nav2 takes back control immediately.
        """
        self._publish_status("NAVIGATING")
        self.get_logger().info(f"[NAV] Navigating to '{label}' at ({goal_x}, {goal_y})...")

        self.navigator.goToPose(pose)

        retry_count   = 2   # number of re-attempts if Nav2 fails
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
                source   = "amcl" if self._amcl_ever_received else "odom"
                amcl_age = now - self._last_amcl_time if self._amcl_ever_received else float("inf")
                self.get_logger().info(
                    f"[NAV] '{label}': straight={dist:.3f} m  "
                    f"path={arc_dist:.3f} m  "
                    f"source={source}(age={amcl_age:.1f}s)  "
                    f"threshold={XY_ARRIVAL_THRESHOLD:.2f} m"
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
                        f"[NAV] '{label}': Nav2 result={result}."
                    )
                    if retry_count > 0:
                        self.get_logger().warn(
                            f"[NAV] '{label}': Retrying ({retry_count} attempt(s) left)..."
                        )
                        self._stop_robot()
                        time.sleep(0.5)
                        pose.header.stamp = self.get_clock().now().to_msg()
                        self.navigator.goToPose(pose)
                        retry_count -= 1
                        last_log_time = 0.0
                        continue
                    self.get_logger().warn(
                        f"[NAV] '{label}': All retries exhausted — FAILED."
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
        self.ui_status_pub.publish(msg)   # mirror to /navigation_status for UI

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