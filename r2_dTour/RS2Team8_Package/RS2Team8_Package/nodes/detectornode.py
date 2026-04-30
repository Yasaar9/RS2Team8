#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 1: Perception and Mapping
Camera + LiDAR fusion detector with pink-bottle target tracking.

Detection pipeline (all toggleable via parameters):
  1. HOG       -> people    (publishes /perception/person_position)
  2. ArUco     -> artifacts (publishes /perception/artifact_position)
  3. HSV pink  -> bottle    (publishes /perception/bottle_position)

For each detection: bbox centre u-pixel -> bearing in robot frame
-> sample LiDAR at that bearing -> PointStamped in scan frame
-> tf2 transform to map frame.

Optional auto-navigation:
  When auto_navigate_to_bottle is true AND a pink bottle is detected
  within [nav_min_trigger_range_m, nav_max_trigger_range_m], the node
  publishes a goal on /navigation/go_to_pose with a configurable
  stand-off distance. The goal points the robot at the bottle. A
  cooldown prevents spamming Nav2 with goals every frame.
"""

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

from cv_bridge import CvBridge

import tf2_ros
# Side-effect import: registers PointStamped with the tf2 buffer.
import tf2_geometry_msgs  # noqa: F401


def yaw_to_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # ── Topic / frame parameters ────────────────────────────────────────
        self.declare_parameter('camera_topic',     '/camera/image_raw')
        self.declare_parameter('scan_topic',       '/scan')
        self.declare_parameter('target_frame',     'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('camera_hfov_deg',  62.2)   # Pi Cam v2; v1=53.5
        self.declare_parameter('scan_window_deg',  4.0)
        self.declare_parameter('min_range',        0.15)
        self.declare_parameter('max_range',        8.0)
        self.declare_parameter('publish_vis',      True)

        # ── Detection toggles ───────────────────────────────────────────────
        self.declare_parameter('detect_people',      True)
        self.declare_parameter('detect_aruco',       True)
        self.declare_parameter('detect_pink_bottle', True)

        # ── Pink bottle HSV thresholds ──────────────────────────────────────
        # Defaults tuned for a PALE pastel bottle under typical indoor light.
        # OpenCV HSV ranges: H 0-180, S 0-255, V 0-255.
        # Pastel pinks straddle the red wraparound: magenta-leaning pink sits
        # at H=160-180, peachy/coral pink sits at H=0-15. Set
        # `pink_use_second_range=true` to OR both bands together. Saturation
        # and value are shared between the two bands.
        # See TUNING comments at the bottom of this file if detection misses.
        self.declare_parameter('pink_h_min',           160)
        self.declare_parameter('pink_h_max',           180)
        self.declare_parameter('pink_s_min',           15)   # pastels = LOW sat
        self.declare_parameter('pink_s_max',           120)
        self.declare_parameter('pink_v_min',           160)  # pastels = HIGH val
        self.declare_parameter('pink_v_max',           255)
        self.declare_parameter('pink_min_area_pixels', 1500)
        # Optional second hue band on the other side of the red wraparound:
        self.declare_parameter('pink_use_second_range', True)
        self.declare_parameter('pink2_h_min',          0)
        self.declare_parameter('pink2_h_max',          15)

        # ── Auto-navigation ─────────────────────────────────────────────────
        self.declare_parameter('auto_navigate_to_bottle', False)
        self.declare_parameter('nav_standoff_m',          0.5)
        self.declare_parameter('nav_cooldown_s',          5.0)
        self.declare_parameter('nav_min_trigger_range_m', 0.4)
        self.declare_parameter('nav_max_trigger_range_m', 3.0)

        # ── Read params into members ────────────────────────────────────────
        self.camera_topic     = self.get_parameter('camera_topic').value
        self.scan_topic       = self.get_parameter('scan_topic').value
        self.target_frame     = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.hfov_rad         = math.radians(
            self.get_parameter('camera_hfov_deg').value)
        self.scan_window_rad  = math.radians(
            self.get_parameter('scan_window_deg').value)
        self.min_range        = self.get_parameter('min_range').value
        self.max_range        = self.get_parameter('max_range').value
        self.publish_vis      = self.get_parameter('publish_vis').value

        self.detect_people      = self.get_parameter('detect_people').value
        self.detect_aruco       = self.get_parameter('detect_aruco').value
        self.detect_pink_bottle = self.get_parameter('detect_pink_bottle').value

        self.pink_lower = np.array([
            self.get_parameter('pink_h_min').value,
            self.get_parameter('pink_s_min').value,
            self.get_parameter('pink_v_min').value], dtype=np.uint8)
        self.pink_upper = np.array([
            self.get_parameter('pink_h_max').value,
            self.get_parameter('pink_s_max').value,
            self.get_parameter('pink_v_max').value], dtype=np.uint8)
        self.pink_min_area = self.get_parameter('pink_min_area_pixels').value

        # Second hue band — shares S/V with the primary range.
        self.use_second_range = self.get_parameter('pink_use_second_range').value
        self.pink2_lower = np.array([
            self.get_parameter('pink2_h_min').value,
            self.get_parameter('pink_s_min').value,
            self.get_parameter('pink_v_min').value], dtype=np.uint8)
        self.pink2_upper = np.array([
            self.get_parameter('pink2_h_max').value,
            self.get_parameter('pink_s_max').value,
            self.get_parameter('pink_v_max').value], dtype=np.uint8)

        self.auto_nav      = self.get_parameter('auto_navigate_to_bottle').value
        self.nav_standoff  = self.get_parameter('nav_standoff_m').value
        self.nav_cooldown  = self.get_parameter('nav_cooldown_s').value
        self.nav_min_range = self.get_parameter('nav_min_trigger_range_m').value
        self.nav_max_range = self.get_parameter('nav_max_trigger_range_m').value

        # ── State ───────────────────────────────────────────────────────────
        self.bridge              = CvBridge()
        self.latest_scan         = None
        self.last_goal_publish_t = 0.0
        self.morph_kernel        = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5))

        # ── HOG people detector ─────────────────────────────────────────────
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # ── ArUco artifact detector ─────────────────────────────────────────
        self.aruco_detector = None
        self.aruco_dict = None
        self.aruco_params = None
        try:
            self.aruco_dict   = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
        except AttributeError:
            try:
                self.aruco_dict   = cv2.aruco.Dictionary_get(
                    cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            except Exception as e:
                self.get_logger().warning(f'ArUco unavailable: {e}')
                self.aruco_dict = None

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Image, self.camera_topic, self.image_cb, 3)
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_person   = self.create_publisher(
            PointStamped, '/perception/person_position', 10)
        self.pub_artifact = self.create_publisher(
            PointStamped, '/perception/artifact_position', 10)
        self.pub_bottle   = self.create_publisher(
            PointStamped, '/perception/bottle_position', 10)
        self.pub_label    = self.create_publisher(
            String, '/perception/detection_label', 10)
        self.pub_vis      = self.create_publisher(
            Image, '/camera/image_detections', 10)
        self.pub_nav_goal = self.create_publisher(
            PoseStamped, '/navigation/go_to_pose', 10)

        self.get_logger().info(
            f'\nDetector ready.'
            f'\n  RGB         : {self.camera_topic}'
            f'\n  scan        : {self.scan_topic}'
            f'\n  HFOV        : {math.degrees(self.hfov_rad):.1f} deg'
            f'\n  out frame   : {self.target_frame}'
            f'\n  people      : {self.detect_people}'
            f'\n  aruco       : {self.detect_aruco}'
            f'\n  pink bottle : {self.detect_pink_bottle}'
            f'\n  auto-nav    : {self.auto_nav} '
            f'(standoff={self.nav_standoff} m, cooldown={self.nav_cooldown}s)'
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
            return

        h, w = cv_img.shape[:2]
        vis = cv_img.copy() if self.publish_vis else None

        # ── People (HOG) ────────────────────────────────────────────────────
        if self.detect_people:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            rects, weights = self.hog.detectMultiScale(
                gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
            for (x, y, bw, bh), conf in zip(rects, weights):
                u = x + bw / 2.0
                self._process_detection(
                    u_pixel=u, image_width=w,
                    label='person', confidence=float(conf),
                    vis=vis, bbox=(x, y, bw, bh), color=(0, 255, 0))

        # ── Artifacts (ArUco) ───────────────────────────────────────────────
        if self.detect_aruco and self.aruco_dict is not None:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
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

        # ── Pink bottle (HSV) ───────────────────────────────────────────────
        if self.detect_pink_bottle:
            self._detect_pink_bottle(cv_img, w, vis)

        # ── Publish annotated frame ─────────────────────────────────────────
        if self.publish_vis and vis is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                out_msg.header.stamp = self.get_clock().now().to_msg()
                out_msg.header.frame_id = msg.header.frame_id
                self.pub_vis.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f'cv_bridge publish error: {e}')

    # ── Pink bottle detection ────────────────────────────────────────────────

    def _detect_pink_bottle(self, cv_img, image_width, vis):
        """Find the single largest pink blob above area threshold."""
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)
        if self.use_second_range:
            # Second hue band on the other side of the red wraparound
            # (e.g. peachy/coral pinks at H=0..15). OR'd with the primary
            # mask so a single contour can span both.
            mask2 = cv2.inRange(hsv, self.pink2_lower, self.pink2_upper)
            mask = cv2.bitwise_or(mask, mask2)
        # Open kills speckle noise; close fills small holes from specular
        # highlights on the bottle body.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Pick the single largest contour above area threshold. Picking
        # one best blob (rather than all matches) is robust against
        # background pink noise — clothing, posters, etc.
        best = None
        best_area = self.pink_min_area
        for c in contours:
            area = cv2.contourArea(c)
            if area > best_area:
                best = c
                best_area = area

        if best is None:
            # Tint the matched region red on the vis image so users can
            # see what their HSV range is actually selecting, even when
            # nothing is large enough to be a real detection. Critical
            # for live HSV tuning.
            if vis is not None and mask.any():
                tint = np.zeros_like(vis)
                tint[:, :, 2] = mask
                cv2.addWeighted(vis, 1.0, tint, 0.25, 0, dst=vis)
            return

        x, y, bw, bh = cv2.boundingRect(best)
        u_centre = x + bw / 2.0

        if vis is not None:
            cv2.drawContours(vis, [best], -1, (255, 0, 255), 2)

        result = self._process_detection(
            u_pixel=u_centre, image_width=image_width,
            label='pink_bottle', confidence=1.0,
            vis=vis, bbox=(x, y, bw, bh), color=(255, 0, 255),
            return_data=True)

        if result is None:
            return
        bottle_map, bottle_range = result
        self.pub_bottle.publish(bottle_map)

        if self.auto_nav:
            self._maybe_publish_nav_goal(bottle_map, bottle_range)

    # ── Auto-navigation ──────────────────────────────────────────────────────

    def _maybe_publish_nav_goal(self, bottle_map_point: PointStamped,
                                bottle_range: float):
        """
        Publish a Nav2 goal at `nav_standoff_m` in front of the bottle,
        oriented toward it. Rate-limited by `nav_cooldown_s`.
        """
        if not (self.nav_min_range <= bottle_range <= self.nav_max_range):
            return  # too close or too far to safely target

        now = time.time()
        if now - self.last_goal_publish_t < self.nav_cooldown:
            return  # still in cooldown after a recent goal

        # Robot's current pose in map frame.
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warning(
                f'auto-nav: cannot get robot pose: {e}')
            return

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        bx = bottle_map_point.point.x
        by = bottle_map_point.point.y
        dx = bx - rx
        dy = by - ry
        d  = math.hypot(dx, dy)
        if d < 1e-3:
            return

        # Goal is `standoff` metres back from the bottle along the
        # robot-to-bottle line. Yaw points from goal toward bottle.
        gx  = bx - self.nav_standoff * dx / d
        gy  = by - self.nav_standoff * dy / d
        yaw = math.atan2(dy, dx)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.target_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.pub_nav_goal.publish(goal)
        self.last_goal_publish_t = now
        self.get_logger().info(
            f'[AUTO-NAV] bottle at map=({bx:+.2f},{by:+.2f}) '
            f'-> goal=({gx:+.2f},{gy:+.2f}) yaw={math.degrees(yaw):+.1f}deg '
            f'standoff={self.nav_standoff:.2f}m'
        )

    # ── Core fusion ──────────────────────────────────────────────────────────

    def _process_detection(self, u_pixel, image_width, label, confidence,
                           vis, bbox, color, return_data=False):
        """
        Convert a 2D detection centroid into a 3D point in target_frame.

        Returns (PointStamped, range) when return_data=True, else None.

        Bearing convention:
            Right side of image (u > W/2) -> object is to the robot's RIGHT.
            In ROS REP-103 (x forward, y left, +yaw CCW), right = NEGATIVE
            bearing. Hence the leading minus sign.

        Range source:
            Range is sampled from the LiDAR at the bearing direction. The
            camera and LiDAR on TurtleBot3 are roughly coaxial in the
            horizontal plane, so for objects beyond ~0.5 m parallax error
            is negligible. The PointStamped is tagged with the LiDAR's
            frame_id (typically base_scan) and tf2 transforms it to map.
        """
        bearing = -((u_pixel - image_width / 2.0) / image_width) * self.hfov_rad

        r = self._sample_scan(bearing)
        if r is None:
            return None

        ps = PointStamped()
        ps.header.stamp    = self.latest_scan.header.stamp
        ps.header.frame_id = self.latest_scan.header.frame_id
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
            return None

        if label == 'person':
            self.pub_person.publish(transformed)
        elif label.startswith('artifact'):
            self.pub_artifact.publish(transformed)
        # bottle is published by the caller (it needs return_data first)

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

        if return_data:
            return transformed, r
        return None

    def _sample_scan(self, bearing_rad):
        """Median range over a small angular window. None if no valid samples."""
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
            if math.isfinite(r) and self.min_range < r < rmax:
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


# ─────────────────────────────────────────────────────────────────────────────
# TUNING THE PINK HSV RANGE
# ─────────────────────────────────────────────────────────────────────────────
# OpenCV HSV scale: H = 0..180, S = 0..255, V = 0..255.
# Pinks straddle the red wraparound:
#   - Magenta-leaning pink   -> H = 160..180  (primary range default)
#   - Peachy/coral-leaning   -> H = 0..15     (second range default)
# Pastel = LOW saturation (try 15-120). Brighter pink = higher (100-255).
# Lighting affects V the most. Dim room: V_min ~ 100. Bright room: V_min ~ 160.
#
# Tune live without restarting the node:
#   ros2 param set /detector_node pink_h_min 160
#   ros2 param set /detector_node pink_h_max 180
#   ros2 param set /detector_node pink_s_min 15
#   ros2 param set /detector_node pink_v_min 160
#
# If the bottle doesn't show up, it may be on the OTHER side of the
# wraparound. The dual-range mode is on by default and covers both bands;
# disable it with `ros2 param set /detector_node pink_use_second_range false`
# if a background object on the other band is creating false positives.
# Adjust the second band with `pink2_h_min` and `pink2_h_max`. Saturation
# and value are shared between the two bands.
#
# Watch /camera/image_detections in rqt_image_view. When no contour is
# large enough, the node TINTS RED the pixels matched by your current
# range — point the camera at the bottle and tweak until the bottle
# (and only the bottle) lights up red. Then a magenta outline appears
# once the matched region exceeds pink_min_area_pixels.
# ─────────────────────────────────────────────────────────────────────────────