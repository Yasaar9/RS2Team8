#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 1: Perception and Mapping
Camera + LiDAR fusion detector.

Pipeline:
  RGB image  -> HOG (people) + ArUco (artifacts) gives 2D bounding boxes.
  Bbox centre u-pixel -> bearing in camera frame (using known HFOV).
  Bearing -> index into latest LaserScan -> median range over a small window.
  (range, bearing) in LiDAR frame -> PointStamped -> tf2 transform -> map frame.

Topics consumed:
  /camera/image_raw  (sensor_msgs/Image)     - RGB frames from TurtleBot camera
  /scan              (sensor_msgs/LaserScan) - 360 deg LDS-01 scan

Topics published:
  /perception/person_position     (geometry_msgs/PointStamped) in map frame
  /perception/artifact_position   (geometry_msgs/PointStamped) in map frame
  /perception/detection_label     (std_msgs/String)            for logging
  /camera/image_detections        (sensor_msgs/Image)          annotated frame
"""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from cv_bridge import CvBridge

import tf2_ros
# Required side-effect import: registers PointStamped <-> TransformStamped
# so tf_buffer.transform() works on PointStamped messages.
import tf2_geometry_msgs  # noqa: F401


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('camera_topic',     '/camera/image_raw')
        self.declare_parameter('scan_topic',       '/scan')
        self.declare_parameter('target_frame',     'map')
        self.declare_parameter('camera_hfov_deg',  62.2)   # Pi Cam v2; v1 is ~53.5
        self.declare_parameter('scan_window_deg',  4.0)    # +/- 2 deg around bearing
        self.declare_parameter('min_range',        0.15)
        self.declare_parameter('max_range',        8.0)
        self.declare_parameter('publish_vis',      True)

        self.camera_topic    = self.get_parameter('camera_topic').value
        self.scan_topic      = self.get_parameter('scan_topic').value
        self.target_frame    = self.get_parameter('target_frame').value
        self.hfov_rad        = math.radians(self.get_parameter('camera_hfov_deg').value)
        self.scan_window_rad = math.radians(self.get_parameter('scan_window_deg').value)
        self.min_range       = self.get_parameter('min_range').value
        self.max_range       = self.get_parameter('max_range').value
        self.publish_vis     = self.get_parameter('publish_vis').value

        self.bridge = CvBridge()

        # ── HOG people detector ─────────────────────────────────────────────
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # ── ArUco artifact detector (handles old + new OpenCV APIs) ─────────
        self.aruco_detector = None
        self.aruco_dict = None
        self.aruco_params = None
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
            self.get_logger().info('ArUco enabled (modern API, DICT_4X4_50)')
        except AttributeError:
            try:
                self.aruco_dict = cv2.aruco.Dictionary_get(
                    cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.get_logger().info('ArUco enabled (legacy API, DICT_4X4_50)')
            except Exception as e:
                self.get_logger().warning(f'ArUco unavailable: {e}')
                self.aruco_dict = None

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Latest scan cache (no tight sync needed at gallery speeds) ──────
        self.latest_scan: LaserScan | None = None

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Image, self.camera_topic, self.image_cb, 3)
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_person   = self.create_publisher(
            PointStamped, '/perception/person_position', 10)
        self.pub_artifact = self.create_publisher(
            PointStamped, '/perception/artifact_position', 10)
        self.pub_label    = self.create_publisher(
            String, '/perception/detection_label', 10)
        self.pub_vis      = self.create_publisher(
            Image, '/camera/image_detections', 10)

        self.get_logger().info(
            f'Detector ready.\n'
            f'  RGB     : {self.camera_topic}\n'
            f'  scan    : {self.scan_topic}\n'
            f'  HFOV    : {math.degrees(self.hfov_rad):.1f} deg\n'
            f'  out frame: {self.target_frame}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge img conversion error: {e}')
            return

        if self.latest_scan is None:
            return  # need a scan to fuse

        h, w = cv_img.shape[:2]
        vis = cv_img.copy() if self.publish_vis else None
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # ── People (HOG) ────────────────────────────────────────────────────
        rects, weights = self.hog.detectMultiScale(
            gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
        for (x, y, bw, bh), conf in zip(rects, weights):
            u = x + bw / 2.0
            self._process_detection(
                u_pixel=u, image_width=w,
                label='person', confidence=float(conf),
                vis=vis, bbox=(x, y, bw, bh), color=(0, 255, 0))

        # ── Artifacts (ArUco) ───────────────────────────────────────────────
        if self.aruco_dict is not None:
            try:
                if self.aruco_detector is not None:
                    corners, ids, _ = self.aruco_detector.detectMarkers(gray)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(
                        gray, self.aruco_dict, parameters=self.aruco_params)
            except Exception as e:
                self.get_logger().warning(f'ArUco error: {e}')
                corners, ids = [], None

            if ids is not None and len(ids) > 0:
                if vis is not None:
                    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    pts = marker_corners.reshape(-1, 2)
                    u_centre = float(np.mean(pts[:, 0]))
                    self._process_detection(
                        u_pixel=u_centre, image_width=w,
                        label=f'artifact_{int(marker_id)}', confidence=1.0,
                        vis=vis, bbox=None, color=(255, 200, 0))

        # ── Publish annotated frame ─────────────────────────────────────────
        if self.publish_vis and vis is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                out_msg.header.stamp = self.get_clock().now().to_msg()
                out_msg.header.frame_id = msg.header.frame_id
                self.pub_vis.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f'cv_bridge publish error: {e}')

    # ── Core fusion ──────────────────────────────────────────────────────────

    def _process_detection(self, u_pixel, image_width, label, confidence,
                           vis, bbox, color):
        """
        Convert a 2D detection centroid into a 3D point in target_frame.

        Bearing convention:
            Right side of image (u > W/2) -> object is to the robot's RIGHT.
            In ROS REP-103 (x forward, y left, +yaw CCW), right = NEGATIVE
            bearing. Hence the leading minus sign below.

        Range source:
            We sample the LiDAR at the computed bearing — this means the
            range originates from the LiDAR's frame (typically base_scan),
            NOT the camera. The camera and LiDAR on TurtleBot3 are close to
            coaxial in the horizontal plane (a few cm apart), so for objects
            beyond ~0.5 m the parallax error is negligible. We tag the
            PointStamped with the LiDAR's frame_id to be honest about
            where the range measurement came from, and let tf2 handle the
            transform to map.
        """
        bearing = -((u_pixel - image_width / 2.0) / image_width) * self.hfov_rad

        r = self._sample_scan(bearing)
        if r is None:
            return

        ps = PointStamped()
        ps.header.stamp    = self.latest_scan.header.stamp
        ps.header.frame_id = self.latest_scan.header.frame_id  # e.g. base_scan
        ps.point.x = r * math.cos(bearing)
        ps.point.y = r * math.sin(bearing)
        ps.point.z = 0.0

        try:
            transformed = self.tf_buffer.transform(
                ps, self.target_frame,
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warning(
                f'TF {ps.header.frame_id} -> {self.target_frame} failed: {e}')
            return

        if label == 'person':
            self.pub_person.publish(transformed)
        elif label.startswith('artifact'):
            self.pub_artifact.publish(transformed)

        info = (
            f'{label:>14s}  conf={confidence:.2f}  range={r:.2f} m  '
            f'bearing={math.degrees(bearing):+6.1f} deg  '
            f'map=({transformed.point.x:+.2f}, {transformed.point.y:+.2f})'
        )
        self.pub_label.publish(String(data=info))
        self.get_logger().info(info)

        if vis is not None and bbox is not None:
            x, y, bw, bh = bbox
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(vis, f'{label} {r:.1f}m',
                        (x, max(15, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def _sample_scan(self, bearing_rad):
        """
        Median range over a small angular window around the given bearing.
        Returns None if no valid samples (inf, nan, or outside [min,max]).
        """
        scan = self.latest_scan
        if scan is None:
            return None
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0:
            return None

        idx_centre = int(round(
            (bearing_rad - scan.angle_min) / scan.angle_increment)) % n
        half_window = max(1, int(round(
            self.scan_window_rad / scan.angle_increment / 2)))

        ranges = scan.ranges
        rmax = min(self.max_range, scan.range_max)
        valid = []
        for k in range(-half_window, half_window + 1):
            i = (idx_centre + k) % n
            r = ranges[i]
            if (math.isfinite(r)
                    and self.min_range < r < rmax):
                valid.append(r)

        if not valid:
            return None
        return float(np.median(valid))


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


if __name__ == '__main__':
    main()