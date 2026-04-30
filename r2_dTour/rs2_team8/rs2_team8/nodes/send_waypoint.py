#!/usr/bin/env python3
"""
RS2 Team 8 - Quick waypoint sender for testing navigation.py
Usage:
    ros2 run rs2_team8 send_waypoint artifact_1
    ros2 run rs2_team8 send_waypoint artifact_2
    ros2 run rs2_team8 send_waypoint artifact_3
    ros2 run rs2_team8 send_waypoint artifact_4
    ros2 run rs2_team8 send_waypoint toilets
    ros2 run rs2_team8 send_waypoint fire_exit_1
    ros2 run rs2_team8 send_waypoint fire_exit_2
    ros2 run rs2_team8 send_waypoint home
    ros2 run rs2_team8 send_waypoint cancel
    ros2 run rs2_team8 send_waypoint pose 2.5 -1.0 90   # x y yaw_degrees
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class WaypointSender(Node):
    def __init__(self):
        super().__init__("waypoint_sender")
        self.waypoint_pub = self.create_publisher(String, "/navigation/go_to_waypoint", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/navigation/go_to_pose", 10)
        self.cancel_pub = self.create_publisher(String, "/navigation/cancel", 10)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    rclpy.init()
    node = WaypointSender()

    # Give publisher time to connect
    import time
    time.sleep(0.5)

    command = sys.argv[1].lower()

    if command == "cancel":
        msg = String()
        msg.data = "cancel"
        node.cancel_pub.publish(msg)
        print("Sent cancel command.")

    elif command == "pose":
        if len(sys.argv) < 5:
            print("Usage: send_waypoint.py pose <x> <y> <yaw_degrees>")
            return
        x, y, yaw = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        node.pose_pub.publish(pose)
        print(f"Sent raw pose goal: ({x}, {y}, {yaw}°)")

    else:
        msg = String()
        msg.data = command
        node.waypoint_pub.publish(msg)
        print(f"Sent waypoint command: '{command}'")

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()