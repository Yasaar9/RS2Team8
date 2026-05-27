#!/usr/bin/env python3
 
"""
RS2 Team 8 - Tour Guide Robot
Clean HSV Artefact Detector
 
Purpose:
    Detect colour-coded artefacts using the RGB camera.
    Estimate their real map position using camera bearing + LiDAR range.
    Publish named artefact detections for navigation.py.
 
Artefact naming convention:
    artifact_1 = modern art           = black artefact 1
    artifact_2 = sculpture            = light blue water bottle
    artifact_3 = portrait             = black artefact 2
    artifact_4 = historical artefact  = pink water bottle
 
Published topics:
    /perception/artifact_position   geometry_msgs/PointStamped
    /perception/artifact_detection  std_msgs/String
    /camera/image_detections        sensor_msgs/Image
 
Detection message format:
    artifact_2,x,y,z
"""
 
import math
 
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
 
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
 
 
class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
 
        # ── ROS parameters ────────────────────────────────────────────────
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("camera_hfov_deg", 62.2)
        self.declare_parameter("scan_window_deg", 4.0)
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("publish_vis", True)
 
        self.camera_topic = self.get_parameter("camera_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.hfov_rad = math.radians(
            self.get_parameter("camera_hfov_deg").value
        )
        self.scan_window_rad = math.radians(
            self.get_parameter("scan_window_deg").value
        )
        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value
        self.publish_vis = self.get_parameter("publish_vis").value
 
        # ── State ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_scan = None
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )
 
        # ── HSV artefact definitions ──────────────────────────────────────
        # OpenCV HSV scale:
        # H = 0..180, S = 0..255, V = 0..255
        #
        # NOTE:
        # artifact_1 and artifact_3 are both black here.
        # HSV alone cannot reliably distinguish them if both are visible.
        # If needed, make one a different colour or add size/shape filtering.
        self.hsv_artifacts = {
            "artifact_1": {  # modern art = black artefact 1
                "lower": np.array([0, 0, 0], dtype=np.uint8),
                "upper": np.array([180, 255, 55], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (40, 40, 40),
            },
            "artifact_2": {  # sculpture = light blue water bottle
                "lower": np.array([85, 40, 120], dtype=np.uint8),
                "upper": np.array([110, 180, 255], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (255, 200, 0),
            },
            "artifact_3": {  # portrait = black artefact 2
                "lower": np.array([0, 0, 0], dtype=np.uint8),
                "upper": np.array([180, 255, 55], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (80, 80, 80),
            },
            "artifact_4": {  # historical artefact = pink water bottle
                "lower": np.array([160, 15, 160], dtype=np.uint8),
                "upper": np.array([180, 120, 255], dtype=np.uint8),
                "lower2": np.array([0, 15, 160], dtype=np.uint8),
                "upper2": np.array([15, 120, 255], dtype=np.uint8),
                "use_second": True,
                "min_area": 1500,
                "draw_color": (255, 0, 255),
            },
        }
 
        # ── TF2 ───────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 
        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_cb,
            3,
        )
 
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )
 
        # ── Publishers ────────────────────────────────────────────────────
        self.pub_artifact_position = self.create_publisher(
            PointStamped,
            "/perception/artifact_position",
            10,
        )
 
        self.pub_artifact_detection = self.create_publisher(
            String,
            "/perception/artifact_detection",
            10,
        )
 
        self.pub_vis = self.create_publisher(
            Image,
            "/camera/image_detections",
            10,
        )
 
        self.get_logger().info(
            "\nDetector ready."
            f"\n camera_topic: {self.camera_topic}"
            f"\n scan_topic: {self.scan_topic}"
            f"\n target_frame: {self.target_frame}"
            f"\n camera_hfov: {math.degrees(self.hfov_rad):.1f} deg"
        )
 
    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
 
    def scan_cb(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        self.latest_scan = msg
 
    def image_cb(self, msg: Image):
        """Run HSV artefact detection on each camera frame."""
        if self.latest_scan is None:
            return
 
        try:
            cv_img = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
 
        image_height, image_width = cv_img.shape[:2]
        vis = cv_img.copy() if self.publish_vis else None
 
        self.detect_hsv_artifacts(cv_img, image_width, vis)
 
        if self.publish_vis and vis is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                out_msg.header.stamp = self.get_clock().now().to_msg()
                out_msg.header.frame_id = msg.header.frame_id
                self.pub_vis.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f"Visualisation publish failed: {e}")
 
    # ──────────────────────────────────────────────────────────────────────
    # HSV detection
    # ──────────────────────────────────────────────────────────────────────
 
    def detect_hsv_artifacts(self, cv_img, image_width, vis):
        """Detect every configured HSV artefact."""
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
 
        for artefact_name, cfg in self.hsv_artifacts.items():
            mask = cv2.inRange(hsv, cfg["lower"], cfg["upper"])
 
            if cfg.get("use_second", False):
                mask2 = cv2.inRange(hsv, cfg["lower2"], cfg["upper2"])
                mask = cv2.bitwise_or(mask, mask2)
 
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                self.morph_kernel,
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                self.morph_kernel,
            )
 
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
 
            best_contour = None
            best_area = cfg["min_area"]
 
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > best_area:
                    best_contour = contour
                    best_area = area
 
            if best_contour is None:
                continue
 
            x, y, w, h = cv2.boundingRect(best_contour)
            u_centre = x + w / 2.0
 
            result = self.process_detection(
                artefact_name=artefact_name,
                u_pixel=u_centre,
                image_width=image_width,
            )
 
            if result is None:
                continue
 
            point_map, lidar_range, bearing = result
 
            self.pub_artifact_position.publish(point_map)
 
            detection_msg = String()
            detection_msg.data = (
                f"{artefact_name},"
                f"{point_map.point.x:.3f},"
                f"{point_map.point.y:.3f},"
                f"{point_map.point.z:.3f}"
            )
            self.pub_artifact_detection.publish(detection_msg)
 
            self.get_logger().info(
                f"[DETECTED] {artefact_name} "
                f"range={lidar_range:.2f}m "
                f"bearing={math.degrees(bearing):+.1f}deg "
                f"map=({point_map.point.x:+.2f}, {point_map.point.y:+.2f})"
            )
 
            if vis is not None:
                draw_color = cfg["draw_color"]
                cv2.rectangle(
                    vis,
                    (x, y),
                    (x + w, y + h),
                    draw_color,
                    2,
                )
                cv2.putText(
                    vis,
                    f"{artefact_name} {lidar_range:.1f}m",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    draw_color,
                    2,
                )
 
    # ──────────────────────────────────────────────────────────────────────
    # Camera + LiDAR fusion
    # ──────────────────────────────────────────────────────────────────────
 
    def process_detection(self, artefact_name, u_pixel, image_width):
        """
        Convert a camera x-pixel position into a map-frame point.
 
        Steps:
            1. Pixel x-coordinate becomes camera bearing.
            2. LiDAR is sampled at that bearing.
            3. Local LiDAR point is transformed into map frame.
        """
        bearing = -((u_pixel - image_width / 2.0) / image_width) * self.hfov_rad
 
        lidar_range = self.sample_scan(bearing)
        if lidar_range is None:
            return None
 
        local_point = PointStamped()
        local_point.header.stamp = self.latest_scan.header.stamp
        local_point.header.frame_id = self.latest_scan.header.frame_id
        local_point.point.x = lidar_range * math.cos(bearing)
        local_point.point.y = lidar_range * math.sin(bearing)
        local_point.point.z = 0.0
 
        try:
            map_point = self.tf_buffer.transform(
                local_point,
                self.target_frame,
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warning(
                f"TF transform failed for {artefact_name}: "
                f"{local_point.header.frame_id} -> {self.target_frame}: {e}"
            )
            return None
 
        return map_point, lidar_range, bearing
 
    def sample_scan(self, bearing_rad):
        """Return median LiDAR range around a bearing angle."""
        scan = self.latest_scan
 
        if scan is None:
            return None
 
        number_of_ranges = len(scan.ranges)
 
        if number_of_ranges == 0 or scan.angle_increment == 0:
            return None
 
        centre_index = int(
            round((bearing_rad - scan.angle_min) / scan.angle_increment)
        ) % number_of_ranges
 
        half_window = max(
            1,
            int(round(self.scan_window_rad / scan.angle_increment / 2.0)),
        )
 
        valid_ranges = []
        max_allowed_range = min(self.max_range, scan.range_max)
 
        for offset in range(-half_window, half_window + 1):
            index = (centre_index + offset) % number_of_ranges
            value = scan.ranges[index]
 
            if math.isfinite(value) and self.min_range < value < max_allowed_range:
                valid_ranges.append(value)
 
        if not valid_ranges:
            return None
 
        return float(np.median(valid_ranges))
 
 
def main(args=None):
    rclpy.init(args=args)
 
    node = DetectorNode()
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()