#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 2: Motion Planning and Control
Author: Jerry Sun

Navigation node that receives a waypoint (POI name or raw coordinates)
and drives the TurtleBot to it using Nav2.

Topics consumed:
  /navigation/go_to_waypoint  (std_msgs/String)           — POI name e.g. "artifact_1"
  /navigation/go_to_pose      (geometry_msgs/PoseStamped) — raw coordinate goal
  /navigation/cancel          (std_msgs/String)           — cancels current goal

Topics published:
  /navigation/status           (std_msgs/String) — "IDLE"|"NAVIGATING"|"REACHED"|"FAILED"
  /navigation/waypoint_reached (std_msgs/Bool)   — True when goal is reached

"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_simple_commander.robot_navigator import BasicNavigator

import threading
import math
import time


# ===========================================================================
# INITIAL POSE CONFIGURATION
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
# XY_ARRIVAL_THRESHOLD — straight-line distance (metres) from the robot's
#   AMCL pose to the goal at which we declare arrival and cancel the Nav2
#   action. Nav2's distance_remaining reports arc length, not Euclidean
#   distance, so we perform our own pose-based check instead.
#   Rule of thumb: ~half the robot footprint radius (TurtleBot3 ~0.10 m).
#
# NAV2_XY_TOLERANCE / NAV2_YAW_TOLERANCE — pushed to the controller server
#   as a secondary measure. YAW set to π to prevent the final heading-
#   correction spin caused by stateful goal checking.
# ===========================================================================
XY_ARRIVAL_THRESHOLD = 0.15    # metres
NAV2_XY_TOLERANCE    = 0.30    # metres
NAV2_YAW_TOLERANCE   = math.pi # radians
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

        # AMCL pose — updated continuously for pose-based arrival check
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._pose_lock = threading.Lock()
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )

        # ── Internal state ───────────────────────────────────────────────────
        self._navigating = False
        self._nav_thread: threading.Thread | None = None

        self._publish_status("IDLE")
        self.get_logger().info("NavigationNode ready. Waiting for Nav2...")

        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active — ready to navigate.")

        self._apply_goal_tolerances()

    # ── AMCL pose tracking ───────────────────────────────────────────────────

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._pose_lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y

    def _distance_to_goal(self, goal_x: float, goal_y: float) -> float:
        with self._pose_lock:
            dx = self._robot_x - goal_x
            dy = self._robot_y - goal_y
        return math.hypot(dx, dy)

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

    # ── Goal tolerance override ───────────────────────────────────────────────

    def _apply_goal_tolerances(self):
        """
        Push nav2 goal checker parameters to the live controller_server.
        Primary purpose is disabling the stateful yaw-correction phase that
        causes oscillation. The main arrival check is pose-based, not these.
        Parameter paths confirmed via: ros2 param list /controller_server | grep goal
        """
        from rcl_interfaces.msg import Parameter as ParamMsg
        from rcl_interfaces.msg import ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters

        client = self.create_client(SetParameters, "/controller_server/set_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("[TOLERANCE] controller_server unavailable — skipping.")
            return

        def _double(name, value):
            p = ParamMsg()
            p.name  = name
            p.value = ParameterValue()
            p.value.type         = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = value
            return p

        def _bool(name, value):
            p = ParamMsg()
            p.name  = name
            p.value = ParameterValue()
            p.value.type       = ParameterType.PARAMETER_BOOL
            p.value.bool_value = value
            return p

        request = SetParameters.Request()
        request.parameters = [
            _double("general_goal_checker.xy_goal_tolerance",  NAV2_XY_TOLERANCE),
            _double("general_goal_checker.yaw_goal_tolerance", NAV2_YAW_TOLERANCE),
            _bool(  "general_goal_checker.stateful",           False),
        ]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result():
            names = [
                "general_goal_checker.xy_goal_tolerance",
                "general_goal_checker.yaw_goal_tolerance",
                "general_goal_checker.stateful",
            ]
            for name, result in zip(names, future.result().results):
                if result.successful:
                    self.get_logger().info(f"[TOLERANCE]   ✓ {name}")
                else:
                    self.get_logger().warn(f"[TOLERANCE]   ✗ {name}: {result.reason}")
        else:
            self.get_logger().warn("[TOLERANCE] set_parameters timed out.")

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _waypoint_name_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in WAYPOINTS:
            self.get_logger().warn(f"Unknown waypoint '{name}'. Available: {list(WAYPOINTS.keys())}")
            return
        x, y, yaw = WAYPOINTS[name]
        self.get_logger().info(f"Received waypoint name: '{name}' → ({x}, {y}, {yaw}°)")
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
        Background thread. Sends the Nav2 goal then polls /amcl_pose until
        the robot is within XY_ARRIVAL_THRESHOLD of the goal, then cancels
        the Nav2 action and declares REACHED.

        Nav2's isTaskComplete() is not used as a completion condition because
        distance_remaining reports arc length rather than Euclidean distance,
        causing it to never fire on this platform.
        """
        self._publish_status("NAVIGATING")
        self.get_logger().info(f"[NAV] Navigating to '{label}'...")

        self.navigator.goToPose(pose)

        last_log_time = 0.0

        while True:
            now  = time.time()
            dist = self._distance_to_goal(goal_x, goal_y)

            # ── Periodic progress log ─────────────────────────────────────
            if now - last_log_time > 2.0:
                feedback = self.navigator.getFeedback()
                arc_dist = feedback.distance_remaining if feedback else float("nan")
                self.get_logger().info(
                    f"[NAV] '{label}': straight={dist:.3f} m  "
                    f"path={arc_dist:.3f} m  (threshold={XY_ARRIVAL_THRESHOLD:.2f} m)"
                )
                last_log_time = now

            # ── Arrival check ─────────────────────────────────────────────
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

            time.sleep(0.1)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _stop_robot(self):
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