#!/usr/bin/env python3
 
"""
RS2 Team 8 - Tour Guide Robot
Clean HSV Artefact Detector + Fire Exit Assignment + Waypoint Saver
 
Purpose:
    Detect colour-coded artefacts using the RGB camera.
    Estimate their real map position using camera bearing + LiDAR range.
    Publish named detections for navigation.py.
    Save discovered named positions into a waypoint file.
 
Naming convention:
    artifact_1 = green wooden truck body
    artifact_2 = bluish-purple picture
    artifact_3 = yellow elephant
    artifact_4 = pink bottle
 
Additional markers:
    fire_exit_1 = first stable red lunch box detected
    fire_exit_2 = second stable red lunch box detected
 
Toilet:
    Placeholder only for now. White-envelope code is left commented out.
 
Published topics:
    /perception/artifact_position   geometry_msgs/PointStamped
    /perception/artifact_detection  std_msgs/String
    /camera/image_detections        sensor_msgs/Image
 
Detection message format:
    artifact_2,x,y,z
    fire_exit_1,x,y,z
 
Saved waypoint file format:
    name x y yaw_deg
"""
 
import math
import os
 
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
 
        self.declare_parameter(
            "waypoints_output_file",
            os.path.join(
                os.path.expanduser("~"),
                "turtlebot3_ws/src/r2_dTour/0_maps/detector_waypoints.txt",
            ),
        )
        self.declare_parameter("waypoint_update_distance_m", 0.15)
 
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
        self.waypoints_output_file = self.get_parameter(
            "waypoints_output_file"
        ).value
        self.waypoint_update_distance_m = self.get_parameter(
            "waypoint_update_distance_m"
        ).value
 
        # ── State ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_scan = None
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )
 
        # Stable detection / assignment settings
        self.min_confirmations = 3
        self.assignment_distance_m = 0.80
        self.update_distance_m = 0.50
        self.pending_detections = {}
 
        # Saved named points for file output
        self.saved_named_points = {
            "artifact_1": None,
            "artifact_2": None,
            "artifact_3": None,
            "artifact_4": None,
            "fire_exit_1": None,
            "fire_exit_2": None,
            # "toilet_1": None,  # Placeholder only for now
        }
 
        self.write_waypoints_file()
 
        # ── HSV target definitions ────────────────────────────────────────
        # OpenCV HSV scale:
        # H = 0..180, S = 0..255, V = 0..255
        self.hsv_artifacts = {
            "artifact_1": {  # green wooden truck body
                "lower": np.array([65, 40, 60], dtype=np.uint8),
                "upper": np.array([95, 170, 210], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (0, 180, 0),
            },
            "artifact_2": {  # bluish-purple picture
                "lower": np.array([100, 35, 70], dtype=np.uint8),
                "upper": np.array([145, 170, 255], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (180, 80, 255),
            },
            "artifact_3": {  # yellow elephant
                "lower": np.array([18, 60, 90], dtype=np.uint8),
                "upper": np.array([38, 255, 255], dtype=np.uint8),
                "use_second": False,
                "min_area": 1500,
                "draw_color": (0, 255, 255),
            },
            "artifact_4": {  # pink bottle
                "lower": np.array([160, 15, 160], dtype=np.uint8),
                "upper": np.array([180, 120, 255], dtype=np.uint8),
                "lower2": np.array([0, 15, 160], dtype=np.uint8),
                "upper2": np.array([15, 120, 255], dtype=np.uint8),
                "use_second": True,
                "min_area": 1500,
                "draw_color": (255, 0, 255),
            },
            "fire_exit_marker": {  # red lunch box
                "lower": np.array([0, 90, 70], dtype=np.uint8),
                "upper": np.array([10, 255, 255], dtype=np.uint8),
                "lower2": np.array([170, 90, 70], dtype=np.uint8),
                "upper2": np.array([180, 255, 255], dtype=np.uint8),
                "use_second": True,
                "min_area": 1800,
                "draw_color": (0, 0, 255),
            },
            # Placeholder only for now:
            # "toilet_marker": {  # white envelope against black wall
            #     "lower": np.array([0, 0, 150], dtype=np.uint8),
            #     "upper": np.array([180, 55, 255], dtype=np.uint8),
            #     "use_second": False,
            #     "min_area": 1800,
            #     "draw_color": (220, 220, 220),
            # },
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
            f"\n waypoint_file: {self.waypoints_output_file}"
        )
 
    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
 
    def scan_cb(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        self.latest_scan = msg
 
    def image_cb(self, msg: Image):
        """Run HSV detection on each camera frame."""
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
 
        _, image_width = cv_img.shape[:2]
        vis = cv_img.copy() if self.publish_vis else None
 
        self.detect_hsv_targets(cv_img, image_width, vis)
 
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
 
    def detect_hsv_targets(self, cv_img, image_width, vis):
        """Detect every configured HSV target."""
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
 
        for detection_name, cfg in self.hsv_artifacts.items():
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
                detection_name=detection_name,
                u_pixel=u_centre,
                image_width=image_width,
            )
 
            if result is None:
                continue
 
            point_map, lidar_range, bearing = result
 
            assigned_name = self.handle_named_detection(
                detection_name,
                point_map,
            )
 
            if assigned_name is None:
                continue
 
            self.pub_artifact_position.publish(point_map)
 
            detection_msg = String()
            detection_msg.data = (
                f"{assigned_name},"
                f"{point_map.point.x:.3f},"
                f"{point_map.point.y:.3f},"
                f"{point_map.point.z:.3f}"
            )
            self.pub_artifact_detection.publish(detection_msg)
 
            self.save_named_waypoint(assigned_name, point_map)
 
            self.get_logger().info(
                f"[DETECTED] raw={detection_name} final={assigned_name} "
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
                    f"{assigned_name} {lidar_range:.1f}m",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    draw_color,
                    2,
                )
 
    # ──────────────────────────────────────────────────────────────────────
    # Detection naming / temporal confirmation
    # ──────────────────────────────────────────────────────────────────────
 
    def handle_named_detection(self, detection_name, point_map):
        """
        Apply temporal confirmation and final naming rules.
        """
        if not self.confirm_detection_stable(detection_name, point_map):
            return None
 
        if detection_name.startswith("artifact_"):
            self.saved_named_points[detection_name] = point_map
            return detection_name
 
        if detection_name == "fire_exit_marker":
            return self.assign_fire_exit(point_map)
 
        # Placeholder only for now:
        # if detection_name == "toilet_marker":
        #     self.saved_named_points["toilet_1"] = point_map
        #     return "toilet_1"
 
        return None
 
    def confirm_detection_stable(self, detection_name, point_map):
        """
        Require several consistent detections before accepting a target.
        """
        candidate = self.pending_detections.get(detection_name)
 
        if candidate is None:
            self.pending_detections[detection_name] = {
                "count": 1,
                "point": point_map,
            }
            return False
 
        previous_point = candidate["point"]
        distance = self.distance_between_points(previous_point, point_map)
 
        if distance <= self.update_distance_m:
            candidate["count"] += 1
            candidate["point"] = point_map
        else:
            candidate["count"] = 1
            candidate["point"] = point_map
 
        return candidate["count"] >= self.min_confirmations
 
    def assign_fire_exit(self, point_map):
        """
        Assign red lunch boxes in order of first stable unique detection:
          first distinct red target  -> fire_exit_1
          second distinct red target -> fire_exit_2
        Later detections update the nearest existing exit.
        """
        fire_exit_1 = self.saved_named_points["fire_exit_1"]
        fire_exit_2 = self.saved_named_points["fire_exit_2"]
 
        if fire_exit_1 is None:
            self.saved_named_points["fire_exit_1"] = point_map
            return "fire_exit_1"
 
        d1 = self.distance_between_points(point_map, fire_exit_1)
 
        if fire_exit_2 is None:
            if d1 > self.assignment_distance_m:
                self.saved_named_points["fire_exit_2"] = point_map
                return "fire_exit_2"
 
            self.saved_named_points["fire_exit_1"] = point_map
            return "fire_exit_1"
 
        d2 = self.distance_between_points(point_map, fire_exit_2)
 
        if d1 <= d2:
            self.saved_named_points["fire_exit_1"] = point_map
            return "fire_exit_1"
 
        self.saved_named_points["fire_exit_2"] = point_map
        return "fire_exit_2"
 
    def distance_between_points(self, point_a, point_b):
        """Euclidean distance between two PointStamped map points."""
        dx = point_a.point.x - point_b.point.x
        dy = point_a.point.y - point_b.point.y
        dz = point_a.point.z - point_b.point.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)
 
    # ──────────────────────────────────────────────────────────────────────
    # Camera + LiDAR fusion
    # ──────────────────────────────────────────────────────────────────────
 
    def process_detection(self, detection_name, u_pixel, image_width):
        """
        Convert a camera x-pixel position into a map-frame point.
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
                f"TF transform failed for {detection_name}: "
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
 
    # ──────────────────────────────────────────────────────────────────────
    # Waypoint file saving
    # ──────────────────────────────────────────────────────────────────────
 
    def save_named_waypoint(self, name, point_map):
        """Store/update a discovered named point, then rewrite the file."""
        if name not in self.saved_named_points:
            return
 
        new_x = float(point_map.point.x)
        new_y = float(point_map.point.y)
        new_yaw = 0.0
 
        previous = self.saved_named_points[name]
        if previous is not None and isinstance(previous, tuple):
            old_x, old_y, _ = previous
            dist = math.hypot(new_x - old_x, new_y - old_y)
            if dist < self.waypoint_update_distance_m:
                return
 
        self.saved_named_points[name] = (new_x, new_y, new_yaw)
        self.write_waypoints_file()
 
        self.get_logger().info(
            f"[WAYPOINTS] Updated {name} -> "
            f"({new_x:+.3f}, {new_y:+.3f}, {new_yaw:+.1f})"
        )
 
    def write_waypoints_file(self):
        """Rewrite the full detector waypoint file in strict 4-column format."""
        directory = os.path.dirname(self.waypoints_output_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
 
        lines = [
            "# RS2 Team 8 detector-generated waypoints",
            "# Format: name x y yaw_deg",
            "",
        ]
 
        ordered_names = [
            "artifact_1",
            "artifact_2",
            "artifact_3",
            "artifact_4",
            "fire_exit_1",
            "fire_exit_2",
            # "toilet_1",  # Placeholder only for now
        ]
 
        for name in ordered_names:
            saved = self.saved_named_points[name]
 
            if saved is None:
                x, y, yaw = 0.0, 0.0, 0.0
            elif isinstance(saved, tuple):
                x, y, yaw = saved
            else:
                x = float(saved.point.x)
                y = float(saved.point.y)
                yaw = 0.0
 
            lines.append(f"{name} {x:.3f} {y:.3f} {yaw:.1f}")
 
        temp_file = self.waypoints_output_file + ".tmp"
        with open(temp_file, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        os.replace(temp_file, self.waypoints_output_file)
 
 
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