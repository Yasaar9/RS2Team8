#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 2: Motion Planning and Control
Author: Jerry Sun

Navigation node that receives a waypoint (POI name or raw coordinates)
and drives the TurtleBot to it using Nav2.

Topics consumed:
  /navigation/go_to_waypoint  (std_msgs/String)  — POI name e.g. "artifact_1"
  /navigation/go_to_pose      (geometry_msgs/PoseStamped) — raw coordinate goal
  /navigation/cancel          (std_msgs/String)  — cancels current goal

Topics published:
  /navigation/status           (std_msgs/String) — "IDLE" | "NAVIGATING" | "REACHED" | "FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True when goal is reached
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import threading
import math
import time


# ===========================================================================
# INITIAL POSE CONFIGURATION
# This is where the robot starts on the map (i.e. its spawn point in Gazebo).
# If your robot spawns somewhere other than (0, 0), update these values.
# x, y  — position in metres in the map frame
# yaw   — starting heading in degrees (0 = East)
# ===========================================================================
INITIAL_POSE_X   = 0.0
INITIAL_POSE_Y   = 0.0
INITIAL_POSE_YAW = 0.0   # degrees
# ===========================================================================


# ===========================================================================
# WAYPOINT CONFIGURATION
# Add or edit POIs here. Each entry is:
#   "name": (x, y, yaw_degrees)
#
# x, y  — position in the MAP frame (metres).
#          To find coordinates: ros2 topic echo /odom
#          Drive to each location with teleop and note the x/y values.
#
# yaw   — the heading the robot faces on arrival (degrees).
#          0 = East, 90 = North, 180/-180 = West, -90 = South.
#          All set to 0.0 for now to prevent arrival wiggling.
#          Once Nav2 goal tolerances are tuned you can set these freely.
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


def yaw_to_quaternion(yaw_deg: float) -> tuple:
    """Convert a yaw angle (degrees) to a quaternion (x, y, z, w)."""
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    """Build a PoseStamped goal in the map frame."""
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
    """
    Wraps BasicNavigator and exposes simple ROS 2 topics so other
    subsystems (voice, buttons, behaviour tree) can command navigation
    without knowing Nav2 internals.

    On startup, automatically publishes the initial pose to AMCL so
    you don't have to set it manually every session.
    """

    def __init__(self):
        super().__init__("navigation_node")

        # Nav2 simple commander — handles action client lifecycle
        self.navigator = BasicNavigator()

        # ── Set initial pose automatically ───────────────────────────────────
        # Tells AMCL where the robot starts on the map so it can localise.
        self._set_initial_pose()

        # ── Publishers ───────────────────────────────────────────────────────
        self.status_pub  = self.create_publisher(String, "/navigation/status", 10)
        self.reached_pub = self.create_publisher(Bool,   "/navigation/waypoint_reached", 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        # Command by POI name (e.g. "artifact_1")
        self.create_subscription(
            String,
            "/navigation/go_to_waypoint",
            self._waypoint_name_callback,
            10,
        )
        # Command by raw PoseStamped (from GUI or other nodes)
        self.create_subscription(
            PoseStamped,
            "/navigation/go_to_pose",
            self._pose_callback,
            10,
        )
        # Cancel current goal
        self.create_subscription(
            String,
            "/navigation/cancel",
            self._cancel_callback,
            10,
        )

        # ── Internal state ───────────────────────────────────────────────────
        self._navigating  = False
        self._nav_thread: threading.Thread | None = None

        self._publish_status("IDLE")
        self.get_logger().info("NavigationNode ready. Waiting for Nav2...")

        # Block until Nav2 is fully active before accepting commands
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active — ready to navigate.")

    # ── Initial pose ─────────────────────────────────────────────────────────

    def _set_initial_pose(self):
        """
        Publish the robot's starting position to AMCL automatically.
        Called once on node startup — removes the need to set the pose
        manually via RViz or the command line every session.

        BasicNavigator.setInitialPose() expects a PoseStamped and
        wraps it into a PoseWithCovarianceStamped internally.
        """
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
        self.get_logger().info(f"Received waypoint name: '{name}' → ({x}, {y}, {yaw}°)")
        self._navigate_to(make_pose(x, y, yaw), label=name)

    def _pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received raw pose goal: ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f})"
        )
        msg.header.frame_id = "map"
        self._navigate_to(msg, label="raw_pose")

    def _cancel_callback(self, _msg: String):
        self.get_logger().info("Cancel received — cancelling current goal.")
        self.navigator.cancelTask()
        self._navigating = False
        self._publish_status("IDLE")

    # ── Navigation logic ─────────────────────────────────────────────────────

    def _navigate_to(self, pose: PoseStamped, label: str):
        """
        Kick off navigation in a background thread so the ROS 2 executor
        (and therefore all other subscribers) keeps running while moving.
        """
        if self._navigating:
            self.get_logger().warn(
                "Already navigating — cancelling current goal first."
            )
            self.navigator.cancelTask()

        pose.header.stamp = self.get_clock().now().to_msg()

        self._navigating = True
        self._nav_thread = threading.Thread(
            target=self._navigation_thread,
            args=(pose, label),
            daemon=True,
        )
        self._nav_thread.start()

    def _navigation_thread(self, pose: PoseStamped, label: str):
        """Runs in a background thread — blocks until goal is done."""
        self._publish_status("NAVIGATING")
        self.get_logger().info(f"Navigating to '{label}'...")

        self.navigator.goToPose(pose)

        last_feedback_time = 0.0
        while not self.navigator.isTaskComplete():
            now = time.time()
            if now - last_feedback_time > 2.0:
                feedback = self.navigator.getFeedback()
                if feedback:
                    dist = feedback.distance_remaining
                    self.get_logger().info(
                        f"  → '{label}': {dist:.2f} m remaining"
                    )
                last_feedback_time = now
            time.sleep(0.1)

        result = self.navigator.getResult()
        self._navigating = False

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached '{label}'")
            self._publish_status("REACHED")
            self._publish_reached(True)
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f"Navigation to '{label}' was cancelled.")
            self._publish_status("IDLE")
            self._publish_reached(False)
        else:
            self.get_logger().error(f"Failed to reach '{label}' (result={result})")
            self._publish_status("FAILED")
            self._publish_reached(False)

    # ── Helpers ──────────────────────────────────────────────────────────────

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

    # MultiThreadedExecutor lets the nav thread and ROS callbacks run together
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()