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

import datetime
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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False


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
# CLEAR CORRIDOR CONFIGURATION
#
# Used for BOTH pre-navigation heading selection AND unstuck direction search.
# The single method _find_best_clear_bearing(goal_bearing) handles both cases:
#   - Pre-nav:  goal_bearing = atan2(goal_y - ry, goal_x - rx)
#   - Unstuck:  goal_bearing = current robot heading (face best open direction)
#
# CORRIDOR_MIN_RANGE_M — minimum LiDAR range every beam in the corridor must
#   clear. Derived from robot geometry:
#     TurtleBot3 waffle_pi half-width = 0.15 m
#     Safety margin                   = 0.15 m
#     Minimum per-beam range          = 0.30 m  (raw obstacle avoidance floor)
#   For corridor acceptance we use UNSTUCK_CLEAR_THRESHOLD (1.2 m) as the
#   range floor so the selected direction has planner-usable headroom.
#
# CORRIDOR_MIN_ARC_DEG — minimum consecutive arc that must clear the range
#   threshold for a direction to count as a valid passable corridor.
#   Derived from robot width at UNSTUCK_CLEAR_THRESHOLD range:
#     Required gap width  = 2 × 0.30 m = 0.60 m  (robot half-width + margin)
#     chord = 2r·sin(θ/2)  →  0.60 = 2 × 1.2 × sin(θ/2)
#     θ/2 = arcsin(0.25) = 14.5°  →  θ ≈ 29°  → rounded up to 30°
#
# CORRIDOR_GOAL_TOLERANCE_DEG — if the best corridor centre is within this
#   many degrees of the goal bearing, face the goal directly (exact bearing)
#   rather than the corridor centre to avoid unnecessary offset.
#
# ROTATE_SPEED_RAD / ROTATE_TOLERANCE_DEG / ROTATE_TIMEOUT_S — spin params,
#   unchanged from before.
# ===========================================================================
CORRIDOR_MIN_RANGE_M        = 1.2    # metres — same as UNSTUCK_CLEAR_THRESHOLD
CORRIDOR_MIN_ARC_DEG        = 30     # degrees — minimum corridor width (see maths above)
CORRIDOR_GOAL_TOLERANCE_DEG = 5.0   # degrees — snap to exact goal bearing if this close

ROTATE_TOLERANCE_DEG = 10.0   # degrees — stop rotating when within this
ROTATE_SPEED_RAD     = 0.6    # rad/s
ROTATE_TIMEOUT_S     = 10.0   # seconds


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
# OBSTACLE HOLD TIMEOUT
#
# If the path does not clear within this many seconds after an emergency stop,
# the node escalates directly to the unstuck manoeuvre rather than waiting
# forever. This is the primary trigger for unstuck when the robot is physically
# wedged against an obstacle (the path will never clear on its own).
#
# 8 s is long enough to let a person walk past but short enough to recover
# before a demonstration audience loses patience.
# ===========================================================================
OBSTACLE_HOLD_TIMEOUT_S = 8.0   # seconds before escalating to unstuck


# ===========================================================================
# OBSTACLE PHOTO CONFIGURATION
#
# When an obstacle triggers an emergency stop, the node captures one image
# from the camera topic and saves it with the robot's AMCL coordinates
# burned into the image as a text overlay.
#
# OBSTACLE_PHOTO_DIR   — directory to save images and coordinate files.
#                        Created automatically if missing.
# CAMERA_TOPIC         — ROS2 image topic.
#                        Simulation : /camera/image_raw  (Gazebo camera plugin)
#                        Real robot : /camera/image_raw  (camera_ros node)
# CAMERA_TIMEOUT_S     — seconds to wait for a camera frame before giving up.
#                        If no frame arrives (camera not running in sim without
#                        the Gazebo camera plugin), the save is skipped silently.
# ===========================================================================
OBSTACLE_PHOTO_DIR  = "/home/jsunne/git/RS2Team8/r2_dTour/0_obstacles"
CAMERA_TOPIC        = "/camera/image_raw"
CAMERA_TIMEOUT_S    = 2.0


# ===========================================================================
# UNSTUCK CONFIGURATION
#
# When Nav2 fails after all retries (robot is wedged), the node runs an
# unstuck manoeuvre:
#   1. Find the best clear corridor closest to the goal bearing via
#      _find_best_clear_bearing(goal_bearing). This is the same method used
#      pre-navigation, so the two code paths are unified.
#   2. Rotate to face that corridor bearing.
#   3. Drive forward UNSTUCK_DRIVE_DIST m at UNSTUCK_DRIVE_SPEED m/s.
#   4. Re-issue the original Nav2 goal with UNSTUCK_RETRIES more attempt(s).
#
# UNSTUCK_CLEAR_THRESHOLD == CORRIDOR_MIN_RANGE_M (1.2 m) — both use the
#   same clearance threshold so behaviour is consistent.
# ===========================================================================
UNSTUCK_CLEAR_THRESHOLD = CORRIDOR_MIN_RANGE_M   # kept for backward compat in logs
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

        # Full scan snapshot for corridor search
        self._latest_scan: LaserScan | None = None
        self._latest_scan_lock = threading.Lock()

        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)

        # ── Camera — obstacle photo capture ──────────────────────────────────
        # Stores the most recent raw image message for saving on obstacle event.
        # Silently inactive if no publisher is present (e.g. sim without camera).
        self._latest_image: Image | None = None
        self._image_lock = threading.Lock()
        self._photo_taken_this_block: bool = False   # one photo per obstacle event

        self.create_subscription(Image, CAMERA_TOPIC, self._image_callback, 1)

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

    # ── Camera callback ───────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Store the latest camera frame (keep only most recent)."""
        with self._image_lock:
            self._latest_image = msg

    def _save_obstacle_photo(self):
        """
        Save a timestamped JPEG of the current camera frame to OBSTACLE_PHOTO_DIR
        with the robot's AMCL coordinates burned into the image as a text overlay.
        Falls back to a sidecar .txt file if PIL is unavailable.
        Runs in a daemon thread so it never blocks the navigation loop.
        """
        def _worker():
            # Snapshot pose now (called from the scan callback thread)
            rx, ry = self._get_pose()
            yaw_deg = math.degrees(self._get_yaw())
            source  = "amcl" if self._amcl_ever_received else "odom"

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
            stem      = f"obstacle_{timestamp}"
            os.makedirs(OBSTACLE_PHOTO_DIR, exist_ok=True)

            img_path = os.path.join(OBSTACLE_PHOTO_DIR, f"{stem}.jpg")
            txt_path = os.path.join(OBSTACLE_PHOTO_DIR, f"{stem}.txt")

            coord_text = (
                f"x={rx:+.3f} m  y={ry:+.3f} m  yaw={yaw_deg:+.1f} deg  [{source}]"
            )

            with self._image_lock:
                ros_img = self._latest_image

            if ros_img is None:
                # Camera not running (sim without plugin) — write coords only
                self.get_logger().info(
                    f"[PHOTO] No camera frame available — writing coords only: {txt_path}"
                )
                with open(txt_path, "w") as f:
                    f.write(f"Obstacle detected: {coord_text}\n")
                return

            # Convert ROS Image (BGR8 or RGB8) to numpy array
            try:
                import numpy as np
                dtype  = np.uint8
                arr    = np.frombuffer(ros_img.data, dtype=dtype).reshape(
                    ros_img.height, ros_img.width, -1
                )

                if _PIL_AVAILABLE:
                    # Convert to PIL, burn coords as text overlay, save JPEG
                    if ros_img.encoding in ("bgr8", "bgra8"):
                        arr = arr[:, :, ::-1]   # BGR → RGB (drop alpha if present)
                    pil_img = PILImage.fromarray(arr[:, :, :3])
                    draw    = ImageDraw.Draw(pil_img)

                    # Semi-transparent black banner at the bottom
                    banner_h = 36
                    banner   = PILImage.new("RGBA", (pil_img.width, banner_h), (0, 0, 0, 160))
                    pil_img.paste(banner, (0, pil_img.height - banner_h), banner)

                    try:
                        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
                    except Exception:
                        font = ImageFont.load_default()

                    draw.text((8, pil_img.height - banner_h + 6), coord_text, fill=(255, 255, 0), font=font)
                    pil_img.save(img_path, "JPEG", quality=85)
                    self.get_logger().info(f"[PHOTO] Saved: {img_path}  coords: {coord_text}")

                else:
                    # PIL not available — save raw JPEG via numpy only and write txt
                    import struct, zlib
                    # Fallback: write as PPM then rely on user to convert
                    ppm_path = img_path.replace(".jpg", ".ppm")
                    with open(ppm_path, "wb") as f:
                        f.write(f"P6\n{ros_img.width} {ros_img.height}\n255\n".encode())
                        f.write(arr[:, :, :3].tobytes())
                    with open(txt_path, "w") as f:
                        f.write(f"Obstacle detected: {coord_text}\n")
                        f.write(f"Image saved as PPM: {ppm_path}\n")
                    self.get_logger().info(f"[PHOTO] PIL unavailable — PPM saved: {ppm_path}")

            except Exception as exc:
                self.get_logger().warn(f"[PHOTO] Failed to save image: {exc}")
                try:
                    with open(txt_path, "w") as f:
                        f.write(f"Obstacle detected: {coord_text}\n")
                        f.write(f"Image save failed: {exc}\n")
                except Exception:
                    pass

        threading.Thread(target=_worker, daemon=True).start()

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
            # Take one photo per obstacle event
            if not self._photo_taken_this_block:
                self._photo_taken_this_block = True
                self._save_obstacle_photo()

        elif self._obstacle_blocked and min_dist > OBSTACLE_CLEAR_DIST:
            self._obstacle_blocked = False
            self._photo_taken_this_block = False   # reset for next event
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

    # ── Corridor-based bearing selection and rotation ─────────────────────────

    def _find_best_clear_bearing(self, goal_bearing_rad: float) -> float | None:
        """
        Unified method for both pre-navigation heading and unstuck direction.

        Scans the latest 360° LiDAR reading for valid passable corridors:
          - A corridor is a contiguous arc of beams all >= CORRIDOR_MIN_RANGE_M.
          - The arc must span at least CORRIDOR_MIN_ARC_DEG (30°, derived from
            robot body width at 1.2 m range — see constant block for maths).
          - Each valid corridor is scored by its angular distance from
            goal_bearing_rad (smaller = better).
          - The centre bearing of the best-scoring corridor is returned.
          - If that centre is within CORRIDOR_GOAL_TOLERANCE_DEG of the goal
            bearing, the exact goal bearing is returned so the robot faces
            the goal precisely when the path is clear.

        For unstuck use: pass the current robot yaw as goal_bearing_rad so
        the corridor closest to the current heading is preferred, minimising
        unnecessary spinning.

        Returns None if no corridor meets the minimum width requirement.
        """
        with self._latest_scan_lock:
            scan = self._latest_scan

        if scan is None:
            self.get_logger().warn("[CORRIDOR] No scan data available.")
            return None

        num = len(scan.ranges)
        if num == 0:
            return None

        min_beams   = max(1, int(CORRIDOR_MIN_ARC_DEG / 360.0 * num))
        current_yaw = self._get_yaw()

        # Boolean mask: True where beam clears the range threshold
        clear = [
            scan.range_min < r < scan.range_max and r >= CORRIDOR_MIN_RANGE_M
            for r in scan.ranges
        ]

        # Scan for contiguous clear runs (wrap-around handled by doubling the list)
        doubled     = clear + clear
        best_score  = float("inf")
        best_bearing = None

        i = 0
        while i < num:
            if not doubled[i]:
                i += 1
                continue
            run_start = i
            j = i
            while j < run_start + num and doubled[j]:
                j += 1
            run_len = j - run_start
            if run_len >= min_beams:
                centre_idx      = (run_start + run_len // 2) % num
                beam_angle_robot = 2.0 * math.pi * centre_idx / num
                corridor_bearing = current_yaw + beam_angle_robot
                # Normalise to [-pi, pi]
                while corridor_bearing >  math.pi: corridor_bearing -= 2 * math.pi
                while corridor_bearing < -math.pi: corridor_bearing += 2 * math.pi
                score = abs(angle_diff(corridor_bearing, goal_bearing_rad))
                self.get_logger().info(
                    f"[CORRIDOR] corridor: centre={math.degrees(corridor_bearing):.1f}°  "
                    f"width={run_len}/{num} beams ({run_len/num*360:.0f}°)  "
                    f"dist_to_goal={math.degrees(score):.1f}°"
                )
                if score < best_score:
                    best_score    = score
                    best_bearing  = corridor_bearing
            i = j if j > i else i + 1

        if best_bearing is None:
            self.get_logger().warn(
                f"[CORRIDOR] No corridor >= {CORRIDOR_MIN_ARC_DEG}° wide found."
            )
            return None

        if math.degrees(best_score) <= CORRIDOR_GOAL_TOLERANCE_DEG:
            self.get_logger().info(
                f"[CORRIDOR] Best corridor within {CORRIDOR_GOAL_TOLERANCE_DEG}° of goal "
                f"— snapping to exact goal bearing."
            )
            return goal_bearing_rad

        self.get_logger().info(
            f"[CORRIDOR] Best bearing: {math.degrees(best_bearing):.1f}°  "
            f"({math.degrees(best_score):.1f}° from goal {math.degrees(goal_bearing_rad):.1f}°)"
        )
        return best_bearing

    def _rotate_to_bearing(self, bearing_rad: float, label: str) -> bool:
        """
        Spin the robot in-place to face bearing_rad (radians, map frame).
        Sets _rotating = True for the duration (suppresses obstacle detection).
        Returns True when aligned, False if cancelled externally.
        """
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
                    f"[ROTATE] '{label}': aligned ({math.degrees(abs(error_rad)):.1f}° error)."
                )
                break

            cmd.angular.z = math.copysign(ROTATE_SPEED_RAD, error_rad)
            cmd.linear.x  = 0.0
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self._stop_robot()
        self._rotating = False
        time.sleep(0.1)
        return True

    def _rotate_to_clear_bearing_toward_goal(
        self, goal_x: float, goal_y: float, label: str
    ) -> bool:
        """
        Pre-navigation: find the best passable corridor closest to the goal
        bearing and rotate to face it. If the direct path is clear the robot
        faces the goal exactly. If blocked, it faces the nearest open corridor.
        Falls back to facing the goal directly if no corridor is found.
        """
        rx, ry       = self._get_pose()
        goal_bearing = math.atan2(goal_y - ry, goal_x - rx)
        bearing      = self._find_best_clear_bearing(goal_bearing)

        if bearing is None:
            self.get_logger().warn(
                f"[ROTATE] '{label}': no clear corridor found — facing goal directly."
            )
            bearing = goal_bearing

        self.get_logger().info(
            f"[ROTATE] '{label}': rotating to {math.degrees(bearing):.1f}° "
            f"(goal bearing {math.degrees(goal_bearing):.1f}°)"
        )
        return self._rotate_to_bearing(bearing, label)

    # ── Unstuck manoeuvre ─────────────────────────────────────────────────────

    def _run_unstuck(self, label: str) -> bool:
        """
        Full unstuck manoeuvre:
          1. Find the best clear corridor using _find_best_clear_bearing,
             passing current yaw as the goal so the closest open direction
             to current heading is preferred (minimises spin).
          2. Rotate to face it.
          3. Drive forward UNSTUCK_DRIVE_DIST m at UNSTUCK_DRIVE_SPEED.

        Returns True if completed, False if no clear direction or cancelled.
        """
        self.get_logger().warn(f"[UNSTUCK] Starting unstuck manoeuvre for '{label}'.")
        self._publish_status("REROUTING")

        current_yaw = self._get_yaw()
        bearing     = self._find_best_clear_bearing(current_yaw)

        if bearing is None:
            self.get_logger().warn("[UNSTUCK] No clear corridor found — declaring STUCK.")
            self._publish_status("STUCK")
            return False

        ok = self._rotate_to_bearing(bearing, label=f"{label}(unstuck)")
        if not ok:
            return False

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

            cx, cy  = self._get_pose()
            driven  = math.hypot(cx - start_x, cy - start_y)

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

        # ── Phase 1: corridor-based pre-navigation rotation ───────────────────
        ok = self._rotate_to_clear_bearing_toward_goal(goal_x, goal_y, label)
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

        obstacle_blocked_since: float = 0.0   # wall-clock time when blocking started

        while True:
            now = time.time()

            # ── Obstacle hold with timeout ────────────────────────────────────
            # While blocked, hold zero velocity. If the path does not clear
            # within OBSTACLE_HOLD_TIMEOUT_S, the robot is physically wedged
            # (e.g. against a pillar) and waiting is futile — escalate directly
            # to the unstuck manoeuvre by cancelling the Nav2 task and breaking
            # out of the hold so the retry/unstuck logic below can run.
            if self._obstacle_blocked:
                self._stop_robot()

                # Record when blocking started (first iteration only)
                if obstacle_blocked_since == 0.0:
                    obstacle_blocked_since = now

                held_for = now - obstacle_blocked_since
                if held_for >= OBSTACLE_HOLD_TIMEOUT_S:
                    self.get_logger().warn(
                        f"[OBSTACLE] Blocked for {held_for:.1f}s — path not clearing. "
                        f"Escalating to unstuck manoeuvre."
                    )
                    # Cancel the active Nav2 task so isTaskComplete() returns
                    # True on the next iteration with CANCELED result, which
                    # feeds into the existing retry → unstuck flow below.
                    self.navigator.cancelTask()
                    # Clear the flag so the hold loop exits and the main loop
                    # can reach isTaskComplete() on the next iteration.
                    self._obstacle_blocked = False
                    obstacle_blocked_since = 0.0
                    # Also exhaust normal retries so we go straight to unstuck.
                    nav2_retries_left = 0
                    time.sleep(0.15)   # let cancelTask propagate
                else:
                    time.sleep(0.1)
                continue

            # Path just cleared — reset the hold timer
            obstacle_blocked_since = 0.0

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

                # Re-rotate toward actual goal using corridor logic then retry
                ok = self._rotate_to_clear_bearing_toward_goal(goal_x, goal_y, label)
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