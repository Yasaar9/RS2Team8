#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 2: Motion Planning and Control
Author: Jerry Sun

Navigation node — receives a waypoint name or raw pose and drives the
TurtleBot3 to it using Nav2 + Regulated Pure Pursuit controller.

Topics consumed:
  /navigation/go_to_waypoint  (std_msgs/String)           — POI name e.g. "artifact_1"
  /navigation/go_to_pose      (geometry_msgs/PoseStamped) — raw coordinate goal
  /navigation/cancel          (std_msgs/String)           — cancels current goal
  /artifact_goto              (std_msgs/String)           — UI "Go To" button

Topics published:
  /navigation/status           (std_msgs/String)
      "IDLE" | "ROTATING" | "NAVIGATING" | "OBSTACLE" | "REROUTING" | "STUCK" | "FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True on arrival, False on failure
  /navigation_status           (std_msgs/String) — mirror of above for ui_node

ROS2 parameter:
  waypoints_file (string) — absolute path to a waypoints .txt file.
  Default: /home/jsunne/git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt

Launch commands:
  # Simulation
  ros2 run rs2_team8 navigation_node \\
      --ros-args -p waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt

  # Real gallery robot
  ros2 run rs2_team8 navigation_node \\
      --ros-args -p waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_waypoints.txt

Debug output (every 3 s when DEBUG_POSE / DEBUG_OBSTACLE are True):
  [DEBUG POSE]     odom vs amcl coordinates, source, age, divergence
  [DEBUG OBSTACLE] forward scan minimum distance, blocked state
"""

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
# DEFAULT WAYPOINTS FILE PATH
# Overridden at launch with:  --ros-args -p waypoints_file:=<path>
# ===========================================================================
DEFAULT_WAYPOINTS_FILE = os.path.join(
    os.path.expanduser("~"),
    "git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt",
)


# ===========================================================================
# INITIAL POSE CONFIGURATION
# Set to match the robot's position when Nav2 first launches.
# Update once the gallery map is finalised.
# ===========================================================================
INITIAL_POSE_X   = 0.0
INITIAL_POSE_Y   = 0.0
INITIAL_POSE_YAW = 0.0   # degrees


# ===========================================================================
# UI LABEL -> WAYPOINT KEY MAP
#
# Maps the lowercase string sent by ui_node on /artifact_goto to a waypoint
# name in the loaded waypoints file, or to a special resolver token.
#
# Special tokens (resolved dynamically at runtime):
#   "nearest_fire_exit" — resolved to the closest waypoint whose name starts
#                         with "fire_exit_"
#   "nearest_toilet"    — resolved to the closest waypoint whose name starts
#                         with "toilet_"
#
# Only artefacts 1-4 have UI buttons. Everything else is command-only for now.
# ===========================================================================
UI_TO_WAYPOINT = {
    "modern art":          "artifact_1",
    "sculpture":           "artifact_2",
    "portrait":            "artifact_3",
    "historical artefact": "artifact_4",
    "toilet":              "nearest_toilet",
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
# Before handing a goal to Nav2, if the robot's heading error to the goal
# exceeds ROTATE_THRESHOLD_DEG the node spins in-place on /cmd_vel until
# within ROTATE_TOLERANCE_DEG, then Nav2 takes over.
#
# This prevents driving forward into a wall when transitioning between
# waypoints on opposite sides of the map.
#
# /scan obstacle detection is suppressed during rotation so the forward arc
# does not false-trigger against the wall the robot is turning away from.
# ===========================================================================
ROTATE_THRESHOLD_DEG = 45.0   # rotate-first if error exceeds this
ROTATE_TOLERANCE_DEG = 10.0   # stop rotating when within this
ROTATE_SPEED_RAD     = 0.6    # rad/s — keep below max_angular_accel in nav2_params
ROTATE_TIMEOUT_S     = 10.0   # safety cap — give up and proceed if exceeded


# ===========================================================================
# OBSTACLE DETECTION CONFIGURATION
#
# OBSTACLE_STOP_DIST  — range (m) in the forward arc that triggers emergency
#   stop. Set at 0.5 m to meet D-grade criterion (<0.5 m).
# OBSTACLE_CLEAR_DIST — hysteresis clear threshold (slightly above STOP).
# OBSTACLE_ARC_DEG    — total forward arc scanned (+/- 30 deg from forward).
# ===========================================================================
OBSTACLE_STOP_DIST  = 0.5    # metres
OBSTACLE_CLEAR_DIST = 0.6    # metres
OBSTACLE_ARC_DEG    = 60     # degrees


# ===========================================================================
# UNSTUCK CONFIGURATION
#
# When Nav2 fails after all retries (robot is wedged), the node runs an
# unstuck manoeuvre:
#   1. Scan full 360° for the clearest direction.
#   2. Rotate to face it (reuses rotate-first logic).
#   3. Drive forward UNSTUCK_DRIVE_DIST m at UNSTUCK_DRIVE_SPEED m/s.
#   4. Re-issue the original Nav2 goal with UNSTUCK_RETRIES more attempt(s).
#
# UNSTUCK_CLEAR_THRESHOLD — minimum LiDAR range (m) a candidate direction
#   must have to be considered "open". 1.2 m chosen because:
#     - costmap inflation_radius = 0.75 m, so 1.2 m gives 0.45 m of margin
#       outside the inflated footprint — enough for the planner to route through.
#     - At desired_linear_vel = 0.2 m/s, 0.45 m margin = ~2.25 s reaction time.
#     - Values above 1.5 m risk finding no valid direction in corridor maps.
#
# UNSTUCK_DRIVE_DIST  — short forward nudge to clear the robot from the
#   obstacle before replanning. 0.4 m at 0.1 m/s takes ~4 s.
# ===========================================================================
UNSTUCK_CLEAR_THRESHOLD = 1.2    # metres — minimum clearance for a "safe" direction
UNSTUCK_DRIVE_DIST      = 0.4    # metres — forward nudge distance after rotating
UNSTUCK_DRIVE_SPEED     = 0.1    # m/s    — slow forward speed during nudge
UNSTUCK_RETRIES         = 1      # Nav2 re-attempts after a successful unstuck


# ===========================================================================
# RETRY CONFIGURATION
#
# NAV2_RETRIES — number of times to re-send the identical Nav2 goal before
#   escalating to the unstuck manoeuvre. Handles transient planner failures.
# ===========================================================================
NAV2_RETRIES = 2


# ===========================================================================
# DEBUG CONFIGURATION
# Set both to False before the final real-robot demonstration.
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
    """Extract yaw (radians) from the z/w components of a unit quaternion."""
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def angle_diff(a: float, b: float) -> float:
    """Signed difference a - b, wrapped to [-pi, pi]."""
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
# Waypoint file loader
# ---------------------------------------------------------------------------

def load_waypoints(filepath: str) -> dict:
    """
    Parse a waypoints .txt file and return a dict:
        { name: (x, y, yaw_deg) }

    File format (whitespace-separated, # comments, blank lines ignored):
        name   x   y   yaw_degrees

    Raises FileNotFoundError or ValueError on bad input.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(
            f"Waypoints file not found: {filepath}\n"
            f"Check the 'waypoints_file' ROS2 parameter."
        )

    waypoints: dict = {}
    with open(filepath, "r") as fh:
        for lineno, raw in enumerate(fh, start=1):
            line = raw.split("#")[0].strip()   # strip comments
            if not line:
                continue
            parts = line.split()
            if len(parts) != 4:
                raise ValueError(
                    f"{filepath}:{lineno}: expected 'name x y yaw_deg', "
                    f"got {len(parts)} token(s): '{line}'"
                )
            name = parts[0].lower()
            try:
                x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError as exc:
                raise ValueError(
                    f"{filepath}:{lineno}: could not parse numbers: {exc}"
                ) from exc
            waypoints[name] = (x, y, yaw)

    if not waypoints:
        raise ValueError(f"Waypoints file is empty: {filepath}")

    return waypoints


# ===========================================================================
# NavigationNode
# ===========================================================================

class NavigationNode(Node):

    def __init__(self):
        super().__init__("navigation_node")

        # ── Waypoints file parameter ──────────────────────────────────────────
        self.declare_parameter("waypoints_file", DEFAULT_WAYPOINTS_FILE)
        waypoints_file = (
            self.get_parameter("waypoints_file")
            .get_parameter_value()
            .string_value
        )

        try:
            self.WAYPOINTS = load_waypoints(waypoints_file)
            self.get_logger().info(
                f"Loaded {len(self.WAYPOINTS)} waypoints from: {waypoints_file}"
            )
            for name, (x, y, yaw) in self.WAYPOINTS.items():
                self.get_logger().info(f"  {name:<20} ({x:+.3f}, {y:+.3f}, {yaw:+.1f}°)")
        except (FileNotFoundError, ValueError) as exc:
            self.get_logger().fatal(f"Failed to load waypoints: {exc}")
            raise SystemExit(1) from exc

        # ── Nav2 ─────────────────────────────────────────────────────────────
        self.navigator = BasicNavigator()
        self._set_initial_pose()

        # ── Publishers ───────────────────────────────────────────────────────
        self.status_pub    = self.create_publisher(String, "/navigation/status",           10)
        self.ui_status_pub = self.create_publisher(String, "/navigation_status",           10)
        self.reached_pub   = self.create_publisher(Bool,   "/navigation/waypoint_reached", 10)
        self.cmd_vel_pub   = self.create_publisher(Twist,  "/cmd_vel",                     10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,      "/navigation/go_to_waypoint", self._waypoint_name_callback, 10)
        self.create_subscription(PoseStamped, "/navigation/go_to_pose",     self._pose_callback,          10)
        self.create_subscription(String,      "/navigation/cancel",         self._cancel_callback,        10)
        self.create_subscription(String,      "/artifact_goto",             self._ui_goto_callback,       10)

        # ── Pose tracking ────────────────────────────────────────────────────
        # AMCL is primary (map frame, globally consistent).
        # Odom is only used before AMCL has ever published.
        # Once AMCL publishes once, we freeze on the last known AMCL value
        # rather than falling back to drifting odom — odom error near the
        # goal exceeds XY_ARRIVAL_THRESHOLD and causes false misses.
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

        # ── Yaw tracking (always from odom) ──────────────────────────────────
        # Odom yaw is used for rotate-first and unstuck direction logic.
        # It's always available and short-duration in-place spins produce
        # negligible drift.
        self._robot_yaw: float = 0.0
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

        # Full scan snapshot for unstuck direction search
        self._latest_scan: LaserScan | None = None
        self._latest_scan_lock = threading.Lock()

        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._navigating: bool = False
        self._rotating:   bool = False   # True during rotate-first or unstuck spin
        self._nav_thread: threading.Thread | None = None

        # ── Debug timer ──────────────────────────────────────────────────────
        if DEBUG_POSE or DEBUG_OBSTACLE:
            self.create_timer(DEBUG_INTERVAL_SEC, self._debug_log)

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
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w

        with self._odom_lock:
            self._odom_x = x
            self._odom_y = y

        with self._yaw_lock:
            self._robot_yaw = quaternion_to_yaw(qz, qw)

        if not self._amcl_ever_received:
            with self._pose_lock:
                self._robot_x = x
                self._robot_y = y

    def _get_pose(self) -> tuple[float, float]:
        with self._pose_lock:
            return self._robot_x, self._robot_y

    def _get_yaw(self) -> float:
        with self._yaw_lock:
            return self._robot_yaw

    def _distance_to_goal(self, goal_x: float, goal_y: float) -> float:
        rx, ry = self._get_pose()
        return math.hypot(rx - goal_x, ry - goal_y)

    # ── Debug logging ─────────────────────────────────────────────────────────

    def _debug_log(self):
        if DEBUG_POSE:
            with self._odom_lock:
                ox, oy = self._odom_x, self._odom_y
            with self._amcl_lock:
                ax, ay = self._amcl_x, self._amcl_y
            rx, ry = self._get_pose()
            yaw_deg = math.degrees(self._get_yaw())
            amcl_age = time.time() - self._last_amcl_time if self._amcl_ever_received else float("inf")
            source   = "amcl" if self._amcl_ever_received else "odom(pre-amcl)"
            odom_amcl_err = math.hypot(ox - ax, oy - ay) if self._amcl_ever_received else float("nan")
            err_str = f"{odom_amcl_err:.3f} m"
            if self._amcl_ever_received and odom_amcl_err > 0.5:
                err_str += "  <-- large odom/amcl divergence"
            self.get_logger().info(
                f"\n[DEBUG POSE]"
                f"\n  odom   : ({ox:+.3f}, {oy:+.3f})  (odom frame — drifts)"
                f"\n  amcl   : ({ax:+.3f}, {ay:+.3f})  (map frame  — used for nav)"
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
                f"stop_threshold={OBSTACLE_STOP_DIST} m  blocked={blocked}"
            )

    # ── Obstacle detection ───────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        """
        Check the forward arc for obstacles and store the full scan for the
        unstuck direction search.

        TurtleBot3 waffle_pi: 360 readings, index 0 = forward, CCW increasing.
        Obstacle detection is suppressed while _rotating is True (robot is
        spinning on purpose — suppress false triggers against nearby walls).
        """
        # Always store the latest full scan for unstuck use
        with self._latest_scan_lock:
            self._latest_scan = msg

        if self._rotating:
            return   # suppress obstacle detection during intentional spin

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
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(INITIAL_POSE_YAW)
        pose.pose.position.x    = INITIAL_POSE_X
        pose.pose.position.y    = INITIAL_POSE_Y
        pose.pose.position.z    = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.navigator.setInitialPose(pose)
        self.get_logger().info(
            f"Initial pose set: ({INITIAL_POSE_X}, {INITIAL_POSE_Y}, {INITIAL_POSE_YAW}°)"
        )

    # ── Waypoint / nearest-of-type helpers ───────────────────────────────────

    def _nearest_of_type(self, prefix: str) -> str | None:
        """
        Return the name of the closest waypoint whose name starts with
        `prefix`, measured from the robot's current pose.

        Returns None if no waypoints with that prefix exist.

        This is the single generalised method for all nearest-X queries:
          _nearest_of_type("fire_exit_") -> nearest fire exit
          _nearest_of_type("toilet_")    -> nearest toilet
        Add new waypoint types to the .txt file with matching prefixes and
        this method handles them automatically — no code changes needed.
        """
        candidates = {
            name: coords
            for name, coords in self.WAYPOINTS.items()
            if name.startswith(prefix)
        }
        if not candidates:
            self.get_logger().warn(
                f"[NEAREST] No waypoints found with prefix '{prefix}'."
            )
            return None

        rx, ry = self._get_pose()
        best_name = None
        best_dist = float("inf")
        for name, (wx, wy, _) in candidates.items():
            dist = math.hypot(rx - wx, ry - wy)
            self.get_logger().info(
                f"[NEAREST:{prefix}] {name}: {dist:.2f} m from robot ({rx:.2f}, {ry:.2f})"
            )
            if dist < best_dist:
                best_dist = dist
                best_name = name

        self.get_logger().info(
            f"[NEAREST:{prefix}] Selected: {best_name} ({best_dist:.2f} m)"
        )
        return best_name

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _waypoint_name_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in self.WAYPOINTS:
            self.get_logger().warn(
                f"Unknown waypoint '{name}'. Available: {list(self.WAYPOINTS.keys())}"
            )
            return
        x, y, yaw = self.WAYPOINTS[name]
        self.get_logger().info(f"Received waypoint command: '{name}' -> ({x}, {y}, {yaw}°)")
        self._navigate_to(make_pose(x, y, yaw), label=name, goal_x=x, goal_y=y)

    def _pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Received raw pose goal: ({x:.2f}, {y:.2f})")
        msg.header.frame_id = "map"
        self._navigate_to(msg, label="raw_pose", goal_x=x, goal_y=y)

    def _cancel_callback(self, _msg: String):
        self.get_logger().info("Cancel received — stopping navigation.")
        self.navigator.cancelTask()
        self._stop_robot()
        self._navigating = False
        self._rotating   = False
        self._publish_status("IDLE")

    def _ui_goto_callback(self, msg: String):
        """
        Receives 'Go To' button presses from ui_node via /artifact_goto.
        Resolves special tokens ('nearest_fire_exit', 'nearest_toilet') to
        the closest matching waypoint at request time.
        """
        label    = msg.data.strip().lower()
        waypoint = UI_TO_WAYPOINT.get(label)

        if waypoint is None:
            self.get_logger().warn(
                f"[UI] No mapping for '{label}'. "
                f"Known UI labels: {list(UI_TO_WAYPOINT.keys())}"
            )
            return

        # Resolve nearest-X tokens dynamically
        if waypoint == "nearest_fire_exit":
            waypoint = self._nearest_of_type("fire_exit_")
        elif waypoint == "nearest_toilet":
            waypoint = self._nearest_of_type("toilet_")

        if waypoint is None:
            self.get_logger().warn(
                f"[UI] Could not resolve destination for '{label}' — no matching waypoints loaded."
            )
            return

        if waypoint not in self.WAYPOINTS:
            self.get_logger().warn(
                f"[UI] Resolved waypoint '{waypoint}' not in loaded waypoints."
            )
            return

        self.get_logger().info(f"[UI] Go To '{label}' -> '{waypoint}'")
        x, y, yaw = self.WAYPOINTS[waypoint]
        self._navigate_to(make_pose(x, y, yaw), label=waypoint, goal_x=x, goal_y=y)

    # ── Navigation dispatch ───────────────────────────────────────────────────

    def _navigate_to(self, pose: PoseStamped, label: str, goal_x: float, goal_y: float):
        if self._navigating:
            self.get_logger().warn("Already navigating — cancelling current goal.")
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

    # ── Rotate-first ─────────────────────────────────────────────────────────

    def _rotate_to_bearing(self, bearing_rad: float, label: str) -> bool:
        """
        Spin the robot in-place to face `bearing_rad` (radians, map frame).
        Sets self._rotating = True for the duration (suppresses obstacle detect).

        Returns True when done (or unnecessary), False if cancelled externally.
        """
        current_yaw = self._get_yaw()
        error_rad   = angle_diff(bearing_rad, current_yaw)
        error_deg   = math.degrees(abs(error_rad))

        if error_deg <= ROTATE_THRESHOLD_DEG:
            self.get_logger().info(
                f"[ROTATE] '{label}': heading error {error_deg:.1f}° — within threshold, skipping."
            )
            return True

        self.get_logger().info(
            f"[ROTATE] '{label}': heading error {error_deg:.1f}° — rotating to "
            f"{math.degrees(bearing_rad):.1f}°."
        )
        self._rotating = True
        self._publish_status("ROTATING")
        start = time.time()
        cmd   = Twist()

        while True:
            if not self._navigating:
                self._stop_robot()
                self._rotating = False
                return False

            if time.time() - start > ROTATE_TIMEOUT_S:
                self.get_logger().warn(
                    f"[ROTATE] '{label}': timed out after {ROTATE_TIMEOUT_S}s — proceeding."
                )
                break

            current_yaw = self._get_yaw()
            error_rad   = angle_diff(bearing_rad, current_yaw)
            if abs(error_rad) <= math.radians(ROTATE_TOLERANCE_DEG):
                self.get_logger().info(
                    f"[ROTATE] '{label}': facing goal ({math.degrees(abs(error_rad)):.1f}° error) — done."
                )
                break

            cmd.angular.z = math.copysign(ROTATE_SPEED_RAD, error_rad)
            cmd.linear.x  = 0.0
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self._stop_robot()
        self._rotating = False
        time.sleep(0.1)   # brief settle before Nav2 or next action
        return True

    def _rotate_to_face_goal(self, goal_x: float, goal_y: float, label: str) -> bool:
        """Compute bearing to goal then call _rotate_to_bearing."""
        rx, ry = self._get_pose()
        bearing = math.atan2(goal_y - ry, goal_x - rx)
        return self._rotate_to_bearing(bearing, label)

    # ── Unstuck manoeuvre ─────────────────────────────────────────────────────

    def _find_clearest_direction(self) -> float | None:
        """
        Scan the full 360° LiDAR reading and return the bearing (radians,
        robot frame) of the direction with the most clearance, provided it
        is above UNSTUCK_CLEAR_THRESHOLD.

        Prefers directions within ±90° of the current heading to minimise
        unnecessary spinning, but will take any direction if nothing closer
        meets the threshold.

        Returns None if no direction clears the threshold (very tight space).
        """
        with self._latest_scan_lock:
            scan = self._latest_scan

        if scan is None:
            self.get_logger().warn("[UNSTUCK] No scan data available for direction search.")
            return None

        num = len(scan.ranges)
        if num == 0:
            return None

        current_yaw = self._get_yaw()
        best_angle  = None
        best_range  = 0.0

        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue
            # TurtleBot3: index 0 = forward, CCW = increasing index
            beam_angle_robot = 2.0 * math.pi * i / num          # robot frame, CCW
            beam_angle_map   = angle_diff(current_yaw + beam_angle_robot, 0.0) + current_yaw

            if r > best_range:
                best_range = r
                best_angle = beam_angle_map

        if best_range < UNSTUCK_CLEAR_THRESHOLD:
            self.get_logger().warn(
                f"[UNSTUCK] Best clearance {best_range:.2f} m < threshold "
                f"{UNSTUCK_CLEAR_THRESHOLD} m — no safe direction found."
            )
            return None

        self.get_logger().info(
            f"[UNSTUCK] Clearest direction: {math.degrees(best_angle):.1f}° "
            f"with {best_range:.2f} m clearance."
        )
        return best_angle

    def _run_unstuck(self, label: str) -> bool:
        """
        Full unstuck manoeuvre:
          1. Find clearest direction.
          2. Rotate to face it.
          3. Drive forward UNSTUCK_DRIVE_DIST metres at UNSTUCK_DRIVE_SPEED.

        Returns True if the manoeuvre completed (robot moved).
        Returns False if no clear direction was found or cancelled.
        """
        self.get_logger().warn(f"[UNSTUCK] Starting unstuck manoeuvre for '{label}'.")
        self._publish_status("REROUTING")

        bearing = self._find_clearest_direction()
        if bearing is None:
            self.get_logger().warn("[UNSTUCK] Cannot find clear direction — declaring STUCK.")
            self._publish_status("STUCK")
            return False

        # Rotate to face the clear direction
        self._rotating = True
        ok = self._rotate_to_bearing(bearing, label=f"{label}(unstuck)")
        if not ok:
            return False   # cancelled

        # Drive forward slowly, watching for obstacles
        self.get_logger().info(
            f"[UNSTUCK] Driving forward {UNSTUCK_DRIVE_DIST} m at {UNSTUCK_DRIVE_SPEED} m/s."
        )
        cmd = Twist()
        cmd.linear.x = UNSTUCK_DRIVE_SPEED
        start_x, start_y = self._get_pose()
        drive_start = time.time()

        while True:
            if not self._navigating:
                self._stop_robot()
                return False

            cx, cy = self._get_pose()
            driven = math.hypot(cx - start_x, cy - start_y)

            if driven >= UNSTUCK_DRIVE_DIST:
                self.get_logger().info(f"[UNSTUCK] Drove {driven:.2f} m — done.")
                break

            if time.time() - drive_start > (UNSTUCK_DRIVE_DIST / UNSTUCK_DRIVE_SPEED) * 3:
                self.get_logger().warn("[UNSTUCK] Drive timed out — proceeding.")
                break

            if self._obstacle_blocked:
                self.get_logger().warn("[UNSTUCK] Obstacle during nudge — aborting drive.")
                break

            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self._stop_robot()
        time.sleep(0.2)
        return True

    # ── Main navigation thread ────────────────────────────────────────────────

    def _navigation_thread(
        self, pose: PoseStamped, label: str, goal_x: float, goal_y: float
    ):
        """
        Background thread managing the full navigation lifecycle:

        Phase 1 — Rotate-first:
          Check heading error to goal. If > ROTATE_THRESHOLD_DEG, spin in-place
          before handing off to Nav2. Prevents driving into a wall when the robot
          arrives at a waypoint facing away from the next destination.

        Phase 2 — Nav2 navigation:
          Send the goal via goToPose and poll AMCL distance each loop.
          Declares REACHED when within XY_ARRIVAL_THRESHOLD (Euclidean, not arc).

        Phase 3 — Retry on Nav2 failure:
          Re-send the same goal up to NAV2_RETRIES times for transient failures.

        Phase 4 — Unstuck manoeuvre (if retries exhausted):
          Scan for clearest direction, rotate, nudge forward, then re-issue the
          Nav2 goal up to UNSTUCK_RETRIES more times. If it still fails,
          publishes STUCK then FAILED.

        Status sequence (published to /navigation/status and /navigation_status):
          ROTATING -> NAVIGATING -> [OBSTACLE ->] NAVIGATING -> REACHED
          ROTATING -> NAVIGATING -> REROUTING -> NAVIGATING -> REACHED
          ROTATING -> NAVIGATING -> REROUTING -> STUCK -> FAILED
        """

        # ── Phase 1: rotate-first ─────────────────────────────────────────────
        ok = self._rotate_to_face_goal(goal_x, goal_y, label)
        if not ok:
            self._navigating = False
            self._publish_status("IDLE")
            return

        # ── Phase 2 & 3: Nav2 navigation with retries ─────────────────────────
        nav2_retries_left  = NAV2_RETRIES
        unstuck_done       = False

        def send_goal():
            pose.header.stamp = self.get_clock().now().to_msg()
            self.navigator.goToPose(pose)

        self._publish_status("NAVIGATING")
        self.get_logger().info(
            f"[NAV] Navigating to '{label}' at ({goal_x:.3f}, {goal_y:.3f})..."
        )
        send_goal()
        last_log_time = 0.0

        while True:
            now = time.time()

            # Obstacle hold — keep publishing zero vel until clear
            if self._obstacle_blocked:
                self._stop_robot()
                time.sleep(0.1)
                continue

            dist = self._distance_to_goal(goal_x, goal_y)

            # Periodic progress log
            if now - last_log_time > 2.0:
                feedback = self.navigator.getFeedback()
                arc_dist = feedback.distance_remaining if feedback else float("nan")
                source   = "amcl" if self._amcl_ever_received else "odom"
                amcl_age = now - self._last_amcl_time if self._amcl_ever_received else float("inf")
                self.get_logger().info(
                    f"[NAV] '{label}': straight={dist:.3f} m  path={arc_dist:.3f} m  "
                    f"source={source}(age={amcl_age:.1f}s)"
                )
                last_log_time = now

            # ── Arrival (primary check) ───────────────────────────────────────
            if dist <= XY_ARRIVAL_THRESHOLD:
                self.get_logger().info(
                    f"[NAV] '{label}': arrived ({dist:.3f} m) — REACHED."
                )
                self.navigator.cancelTask()
                self._stop_robot()
                self._navigating = False
                self._publish_status("REACHED")
                self._publish_reached(True)
                return

            # ── Nav2 task complete (secondary check) ──────────────────────────
            if not self.navigator.isTaskComplete():
                time.sleep(0.1)
                continue

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                # Nav2 declared success — accept if we're close enough
                self.get_logger().info(
                    f"[NAV] '{label}': Nav2 SUCCEEDED at dist={dist:.3f} m — accepting."
                )
                self._stop_robot()
                self._navigating = False
                self._publish_status("REACHED")
                self._publish_reached(True)
                return

            # Nav2 failed
            self.get_logger().warn(
                f"[NAV] '{label}': Nav2 failed (result={result}). "
                f"retries_left={nav2_retries_left}  unstuck_done={unstuck_done}"
            )

            # ── Phase 3: simple retry ─────────────────────────────────────────
            if nav2_retries_left > 0:
                self.get_logger().warn(
                    f"[NAV] '{label}': Retrying ({nav2_retries_left} left)..."
                )
                self._stop_robot()
                time.sleep(0.5)
                nav2_retries_left -= 1
                self._publish_status("NAVIGATING")
                send_goal()
                last_log_time = 0.0
                continue

            # ── Phase 4: unstuck manoeuvre ────────────────────────────────────
            if not unstuck_done:
                self.get_logger().warn(
                    f"[NAV] '{label}': All retries exhausted — attempting unstuck."
                )
                unstuck_ok = self._run_unstuck(label)
                unstuck_done = True

                if not unstuck_ok or not self._navigating:
                    # Either no clear direction found, or cancelled
                    self._stop_robot()
                    self._navigating = False
                    self._publish_status("FAILED")
                    self._publish_reached(False)
                    return

                # Re-rotate toward the actual goal and try once more
                ok = self._rotate_to_face_goal(goal_x, goal_y, label)
                if not ok:
                    self._navigating = False
                    self._publish_status("IDLE")
                    return

                nav2_retries_left = UNSTUCK_RETRIES
                self._publish_status("NAVIGATING")
                send_goal()
                last_log_time = 0.0
                continue

            # Unstuck was already attempted and Nav2 still failed
            self.get_logger().warn(
                f"[NAV] '{label}': Unstuck failed to recover — FAILED."
            )
            self._stop_robot()
            self._navigating = False
            self._publish_status("STUCK")
            time.sleep(1.0)
            self._publish_status("FAILED")
            self._publish_reached(False)
            return

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _stop_robot(self):
        """Publish a zero-velocity Twist to halt the robot immediately."""
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.ui_status_pub.publish(msg)

    def _publish_reached(self, reached: bool):
        msg      = Bool()
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