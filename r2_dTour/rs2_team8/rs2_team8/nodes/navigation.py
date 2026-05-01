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
  /navigation/status           (std_msgs/String) — "IDLE"|"ROTATING"|"NAVIGATING"|"OBSTACLE"|"REACHED"|"FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True when goal is reached
  /navigation_status           (std_msgs/String) — mirror of above for UI node

Debug output (every 3 s):
  [DEBUG] odom=(...) amcl=(...) source=amcl|odom  amcl_age=...s
  [DEBUG] obstacle: forward_min=...m  blocked=True/False

Waypoints are loaded from waypoints.ini at startup (same directory as this
file). Set [active] -> map in that file to switch between simulation and
real-robot maps without touching this file.
"""

import configparser
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


# ===========================================================================
# WAYPOINTS CONFIG PATH
# Resolved relative to this source file so it works regardless of the
# working directory when the node is launched.
# ===========================================================================
_THIS_DIR    = os.path.dirname(os.path.abspath(__file__))
WAYPOINTS_FILE = os.path.join(_THIS_DIR, "waypoints.ini")


# ===========================================================================
# UI LABEL -> WAYPOINT NAME MAP
# Maps the display names from the UI "Go To" buttons to waypoint keys.
# "nearest_fire_exit" is resolved dynamically at runtime using any waypoint
# whose name starts with "fire_exit".
# ===========================================================================
UI_TO_WAYPOINT = {
    "modern art":          "artifact_1",
    "sculpture":           "artifact_2",
    "portrait":            "artifact_3",
    "historical artefact": "artifact_4",
    "toilet":              "toilets",
    "fire exit":           "nearest_fire_exit",
}


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
# ROTATE-FIRST CONFIGURATION
#
# Before handing a goal to Nav2, the node checks whether the robot is already
# facing roughly the right direction. If the angular error to the goal exceeds
# ROTATE_THRESHOLD_DEG, the node spins the robot in-place using /cmd_vel until
# it is within ROTATE_TOLERANCE_DEG, then proceeds with Nav2.
#
# This prevents the robot from driving forward into a wall when it arrives at
# a waypoint facing away from the next goal.
#
# ROTATE_SPEED_RAD  — angular velocity (rad/s) during the pre-rotation.
#                     Keep below the controller's max_angular_accel limit.
# ROTATE_TIMEOUT_S  — safety cap; gives up if rotation takes longer than this
#                     (e.g. if IMU/odom is stale) and proceeds anyway.
# ===========================================================================
ROTATE_THRESHOLD_DEG  = 45.0   # degrees — rotate first if error exceeds this
ROTATE_TOLERANCE_DEG  = 10.0   # degrees — stop rotating when within this
ROTATE_SPEED_RAD      = 0.6    # rad/s
ROTATE_TIMEOUT_S      = 10.0   # seconds — give up and proceed if exceeded


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
OBSTACLE_STOP_DIST  = 0.5    # metres  (D-grade: emergency stop within 0.5m)
OBSTACLE_CLEAR_DIST = 0.6    # metres  (slightly larger for hysteresis)
OBSTACLE_ARC_DEG    = 60     # degrees total forward arc


# ===========================================================================
# DEBUG CONFIGURATION
#
# Set both to False for the final real-robot demonstration.
# ===========================================================================
DEBUG_POSE         = True
DEBUG_OBSTACLE     = True
DEBUG_INTERVAL_SEC = 3.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw_deg: float) -> tuple:
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(qz: float, qw: float) -> float:
    """Extract yaw (radians) from a quaternion (z, w components only for 2-D)."""
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def angle_diff(a: float, b: float) -> float:
    """Signed angular difference a - b, wrapped to [-pi, pi]."""
    diff = a - b
    while diff >  math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


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


# ---------------------------------------------------------------------------
# Waypoint loader
# ---------------------------------------------------------------------------

def load_waypoints(filepath: str) -> tuple[dict, list, tuple, str]:
    """
    Parse waypoints.ini and return:
      waypoints      : dict[name -> (x, y, yaw_deg)]
      fire_exits     : list of waypoint names starting with "fire_exit"
      initial_pose   : (x, y, yaw_deg)
      active_map     : name of the loaded map section (for logging)

    Raises FileNotFoundError if the file is missing, or ValueError if the
    [active] section or referenced map section is absent.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(
            f"Waypoints file not found: {filepath}\n"
            f"Expected alongside navigation.py at: {_THIS_DIR}"
        )

    cfg = configparser.ConfigParser()
    cfg.read(filepath)

    if "active" not in cfg or "map" not in cfg["active"]:
        raise ValueError(
            "waypoints.ini must contain an [active] section with a 'map' key."
        )

    active_map = cfg["active"]["map"].strip()

    if active_map not in cfg:
        raise ValueError(
            f"waypoints.ini: active map '{active_map}' has no matching section."
        )

    waypoints: dict = {}
    for name, value in cfg[active_map].items():
        parts = [p.strip() for p in value.split(",")]
        if len(parts) != 3:
            raise ValueError(
                f"waypoints.ini [{active_map}] '{name}': "
                f"expected 'x, y, yaw_deg', got '{value}'"
            )
        waypoints[name] = (float(parts[0]), float(parts[1]), float(parts[2]))

    fire_exits = [n for n in waypoints if n.startswith("fire_exit")]

    # Initial pose (optional section; defaults to 0,0,0)
    pose_section = f"{active_map}_initial_pose"
    if pose_section in cfg:
        ip = cfg[pose_section]
        initial_pose = (
            float(ip.get("x",   0.0)),
            float(ip.get("y",   0.0)),
            float(ip.get("yaw", 0.0)),
        )
    else:
        initial_pose = (0.0, 0.0, 0.0)

    return waypoints, fire_exits, initial_pose, active_map


# ===========================================================================
# NavigationNode
# ===========================================================================

class NavigationNode(Node):

    def __init__(self):
        super().__init__("navigation_node")

        # ── Load waypoints from config ────────────────────────────────────────
        try:
            self.WAYPOINTS, self.FIRE_EXITS, initial_pose_cfg, active_map = \
                load_waypoints(WAYPOINTS_FILE)
            self.get_logger().info(
                f"Loaded {len(self.WAYPOINTS)} waypoints from map '{active_map}' "
                f"({WAYPOINTS_FILE})"
            )
        except (FileNotFoundError, ValueError) as exc:
            self.get_logger().fatal(f"Failed to load waypoints: {exc}")
            raise SystemExit(1) from exc

        self._initial_pose_x   = initial_pose_cfg[0]
        self._initial_pose_y   = initial_pose_cfg[1]
        self._initial_pose_yaw = initial_pose_cfg[2]

        self.navigator = BasicNavigator()
        self._set_initial_pose()

        # ── Publishers ───────────────────────────────────────────────────────
        self.status_pub    = self.create_publisher(String, "/navigation/status",       10)
        self.ui_status_pub = self.create_publisher(String, "/navigation_status",       10)
        self.reached_pub   = self.create_publisher(Bool,   "/navigation/waypoint_reached", 10)
        self.cmd_vel_pub   = self.create_publisher(Twist,  "/cmd_vel",                 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,      "/navigation/go_to_waypoint", self._waypoint_name_callback, 10)
        self.create_subscription(PoseStamped, "/navigation/go_to_pose",     self._pose_callback,          10)
        self.create_subscription(String,      "/navigation/cancel",         self._cancel_callback,        10)
        self.create_subscription(String,      "/artifact_goto",             self._ui_goto_callback,       10)

        # ── Pose tracking (XY) ───────────────────────────────────────────────
        # AMCL is primary. Odom is used only before AMCL has ever published.
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._pose_lock = threading.Lock()

        self._amcl_ever_received: bool = False
        self._last_amcl_time: float    = 0.0

        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_lock = threading.Lock()

        self._amcl_x: float = 0.0
        self._amcl_y: float = 0.0
        self._amcl_lock = threading.Lock()

        # ── Heading tracking (yaw) ───────────────────────────────────────────
        # Yaw is always sourced from /odom (always available, good enough for
        # in-place rotation which is short-duration and low-drift).
        self._robot_yaw: float = 0.0   # radians, updated from odom
        self._yaw_lock = threading.Lock()

        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 10
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )

        # ── Obstacle detection ───────────────────────────────────────────────
        self._obstacle_blocked: bool   = False
        self._obstacle_min_dist: float = float("inf")
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

    # ── Pose / yaw tracking ──────────────────────────────────────────────────

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._amcl_lock:
            self._amcl_x = x
            self._amcl_y = y
        with self._pose_lock:
            self._robot_x = x
            self._robot_y = y
        self._amcl_ever_received = True
        self._last_amcl_time = time.time()

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        with self._odom_lock:
            self._odom_x = x
            self._odom_y = y

        # Always update yaw from odom — used for rotate-first logic
        with self._yaw_lock:
            self._robot_yaw = quaternion_to_yaw(qz, qw)

        # XY position: odom is only promoted before AMCL has ever published
        if not self._amcl_ever_received:
            with self._pose_lock:
                self._robot_x = x
                self._robot_y = y

    def _get_robot_yaw(self) -> float:
        with self._yaw_lock:
            return self._robot_yaw

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
            with self._yaw_lock:
                yaw_deg = math.degrees(self._robot_yaw)

            amcl_age     = time.time() - self._last_amcl_time if self._amcl_ever_received else float("inf")
            source       = "amcl" if self._amcl_ever_received else "odom(pre-amcl)"
            odom_amcl_err = math.hypot(ox - ax, oy - ay) if self._amcl_ever_received else float("nan")
            err_str = f"{odom_amcl_err:.3f} m"
            if self._amcl_ever_received and odom_amcl_err > 0.5:
                err_str += "  <-- large odom/amcl divergence"

            self.get_logger().info(
                f"\n[DEBUG POSE]"
                f"\n  odom   : ({ox:+.3f}, {oy:+.3f})  (drifts)"
                f"\n  amcl   : ({ax:+.3f}, {ay:+.3f})  (map frame)"
                f"\n  active : {source} -> ({rx:+.3f}, {ry:+.3f})"
                f"\n  yaw    : {yaw_deg:+.1f} deg  (from odom)"
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
        Check the forward arc for obstacles. Skip during rotate-first phase
        (the arc would see the wall the robot is turning away from).
        TurtleBot3 waffle_pi: 360 readings, index 0 = forward, CCW increasing.
        """
        if self._rotating:
            return

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
        qx, qy, qz, qw = yaw_to_quaternion(self._initial_pose_yaw)
        initial_pose.pose.position.x    = self._initial_pose_x
        initial_pose.pose.position.y    = self._initial_pose_y
        initial_pose.pose.position.z    = 0.0
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(
            f"Initial pose set: ({self._initial_pose_x}, {self._initial_pose_y}, "
            f"{self._initial_pose_yaw}°)"
        )

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _waypoint_name_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in self.WAYPOINTS:
            self.get_logger().warn(
                f"Unknown waypoint '{name}'. Available: {list(self.WAYPOINTS.keys())}"
            )
            return
        x, y, yaw = self.WAYPOINTS[name]
        self.get_logger().info(f"Received waypoint: '{name}' -> ({x}, {y}, {yaw}°)")
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
        self._rotating   = False
        self._publish_status("IDLE")

    def _nearest_fire_exit(self) -> str:
        with self._pose_lock:
            rx, ry = self._robot_x, self._robot_y

        best_name = self.FIRE_EXITS[0]
        best_dist = float("inf")
        for name in self.FIRE_EXITS:
            ex, ey, _ = self.WAYPOINTS[name]
            dist = math.hypot(rx - ex, ry - ey)
            self.get_logger().info(
                f"[FIRE EXIT] {name}: {dist:.2f} m from robot ({rx:.2f}, {ry:.2f})"
            )
            if dist < best_dist:
                best_dist = dist
                best_name = name

        self.get_logger().info(f"[FIRE EXIT] Nearest exit: {best_name} ({best_dist:.2f} m)")
        return best_name

    def _ui_goto_callback(self, msg: String):
        label    = msg.data.strip().lower()
        waypoint = UI_TO_WAYPOINT.get(label)
        if waypoint is None:
            self.get_logger().warn(
                f"[UI] No waypoint mapped for '{label}'. "
                f"Available: {list(UI_TO_WAYPOINT.keys())}"
            )
            return

        if waypoint == "nearest_fire_exit":
            if not self.FIRE_EXITS:
                self.get_logger().warn("[UI] No fire exit waypoints defined.")
                return
            waypoint = self._nearest_fire_exit()

        self.get_logger().info(f"[UI] Go To '{label}' -> waypoint '{waypoint}'")
        x, y, yaw = self.WAYPOINTS[waypoint]
        self._navigate_to(make_pose(x, y, yaw), label=waypoint, goal_x=x, goal_y=y)

    # ── Navigation logic ─────────────────────────────────────────────────────

    def _navigate_to(self, pose: PoseStamped, label: str, goal_x: float, goal_y: float):
        if self._navigating:
            self.get_logger().warn("Already navigating — cancelling current goal first.")
            self.navigator.cancelTask()
            self._stop_robot()

        pose.header.stamp = self.get_clock().now().to_msg()
        self._navigating  = True
        self._rotating    = False
        self._nav_thread  = threading.Thread(
            target=self._navigation_thread,
            args=(pose, label, goal_x, goal_y),
            daemon=True,
        )
        self._nav_thread.start()

    def _rotate_to_face_goal(self, goal_x: float, goal_y: float, label: str) -> bool:
        """
        Spin the robot in-place until it faces the goal direction.

        Returns True if rotation completed (or was unnecessary).
        Returns False if cancelled externally or timed out badly.

        Steps:
          1. Compute bearing to goal from current position.
          2. Compute angular error = bearing - current yaw.
          3. If error < ROTATE_THRESHOLD_DEG, skip rotation entirely.
          4. Otherwise publish /cmd_vel angular commands until within
             ROTATE_TOLERANCE_DEG or ROTATE_TIMEOUT_S is reached.
        """
        with self._pose_lock:
            rx, ry = self._robot_x, self._robot_y

        bearing_rad = math.atan2(goal_y - ry, goal_x - rx)
        current_yaw = self._get_robot_yaw()
        error_rad   = angle_diff(bearing_rad, current_yaw)
        error_deg   = math.degrees(abs(error_rad))

        if error_deg <= ROTATE_THRESHOLD_DEG:
            self.get_logger().info(
                f"[ROTATE] '{label}': heading error {error_deg:.1f}° — within threshold, "
                f"skipping pre-rotation."
            )
            return True

        self.get_logger().info(
            f"[ROTATE] '{label}': heading error {error_deg:.1f}° — rotating to face goal "
            f"(bearing {math.degrees(bearing_rad):.1f}°)."
        )
        self._publish_status("ROTATING")
        self._rotating = True

        start_time = time.time()
        cmd = Twist()

        while True:
            if not self._navigating:
                # Cancelled externally
                self._stop_robot()
                self._rotating = False
                return False

            if time.time() - start_time > ROTATE_TIMEOUT_S:
                self.get_logger().warn(
                    f"[ROTATE] '{label}': rotation timed out after {ROTATE_TIMEOUT_S}s — "
                    f"proceeding anyway."
                )
                break

            current_yaw = self._get_robot_yaw()
            error_rad   = angle_diff(bearing_rad, current_yaw)
            error_deg   = math.degrees(abs(error_rad))

            if error_deg <= ROTATE_TOLERANCE_DEG:
                self.get_logger().info(
                    f"[ROTATE] '{label}': facing goal ({error_deg:.1f}° error) — done."
                )
                break

            # Spin toward goal — sign of error_rad determines CW vs CCW
            cmd.angular.z = math.copysign(ROTATE_SPEED_RAD, error_rad)
            cmd.linear.x  = 0.0
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self._stop_robot()
        self._rotating = False
        time.sleep(0.1)   # brief settle before Nav2 takes over
        return True

    def _navigation_thread(self, pose: PoseStamped, label: str, goal_x: float, goal_y: float):
        """
        Background thread.

        Phase 1 — Rotate-first:
          If the robot is facing more than ROTATE_THRESHOLD_DEG away from the
          goal, spin in-place until facing it. This prevents driving forward
          into a wall when transitioning between waypoints on opposite sides
          of the map.

        Phase 2 — Nav2 navigation:
          Sends the goal to Nav2 (goToPose) and polls AMCL pose until within
          XY_ARRIVAL_THRESHOLD, then cancels and declares REACHED.

        Pose source:
          - Before first AMCL message: raw odom (startup only).
          - After first AMCL message:  last known AMCL, never reverts to odom.

        Obstacle behaviour:
          - When blocked: holds zero velocity, Nav2 goal stays active.
          - When cleared: resumes.
        """

        # ── Phase 1: rotate to face the goal ─────────────────────────────────
        ok = self._rotate_to_face_goal(goal_x, goal_y, label)
        if not ok:
            # Cancelled during rotation
            self._navigating = False
            self._publish_status("IDLE")
            return

        # ── Phase 2: Nav2 navigation ──────────────────────────────────────────
        self._publish_status("NAVIGATING")
        self.get_logger().info(f"[NAV] Navigating to '{label}' at ({goal_x}, {goal_y})...")

        pose.header.stamp = self.get_clock().now().to_msg()
        self.navigator.goToPose(pose)

        retry_count   = 2
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

            # ── Nav2 failure / success check (secondary) ──────────────────
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
                            f"[NAV] '{label}': Retrying ({retry_count} left)..."
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

    # _rotating is checked by _scan_callback to suppress obstacle detection
    # while the robot is spinning (otherwise the forward-arc may see the wall
    # the robot is turning away from and false-trigger the emergency stop).
    _rotating: bool = False

    def _stop_robot(self):
        """Publish a zero-velocity Twist to halt the robot immediately."""
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.ui_status_pub.publish(msg)

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