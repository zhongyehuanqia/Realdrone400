#!/usr/bin/env python3
"""Detection-only ROS node. It never talks to a flight controller."""
import json
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class VisionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.state = 'IDLE'
        self.last_center = None
        self.stable = 0
        self.pattern_color = str(rospy.get_param('~pattern_color', 'black')).lower()

        # 黑色图案参数。
        self.adaptive_block = int(rospy.get_param('~adaptive_block', 41)) | 1
        self.adaptive_c = float(rospy.get_param('~adaptive_c', 7.0))
        self.clahe_clip = float(rospy.get_param('~clahe_clip', 2.0))
        self.min_area = float(rospy.get_param('~min_area', 120.0))
        self.min_side = int(rospy.get_param('~min_side_px', 12))
        self.max_side_ratio = float(rospy.get_param('~max_side_ratio', 0.85))
        self.aspect_min = float(rospy.get_param('~aspect_min', 0.45))
        self.aspect_max = float(rospy.get_param('~aspect_max', 2.2))
        self.extent_min = float(rospy.get_param('~extent_min', 0.08))
        self.extent_max = float(rospy.get_param('~extent_max', 0.70))
        self.clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(8, 8))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # 红色备用图案参数。
        self.ranges = rospy.get_param('~red_hue_ranges',
                                      [[0, 90, 55, 10, 255, 255],
                                       [170, 90, 55, 179, 255, 255]])
        self.min_r = float(rospy.get_param('~min_outer_radius_px', 35))
        self.max_r = float(rospy.get_param('~max_outer_radius_px', 300))
        self.center_tol = float(rospy.get_param('~pair_center_tolerance_ratio', 0.18))
        self.ratio = float(rospy.get_param('~expected_radius_ratio', 0.60))
        self.ratio_tol = float(rospy.get_param('~radius_ratio_tolerance', 0.18))

        # 稳定判定与降落建议。
        self.need_stable = int(rospy.get_param('~stable_frames', 8))
        self.max_jump = float(rospy.get_param('~max_center_jump_px', 80))
        self.land_err = float(rospy.get_param('~landing_max_error_normalized', 0.08))
        self.land_min_r = float(rospy.get_param('~landing_min_outer_radius_px', 75))

        self.rotate = int(rospy.get_param('~rotate', 0))
        self.flip_h = bool(rospy.get_param('~flip_horizontal', False))
        self.flip_v = bool(rospy.get_param('~flip_vertical', False))
        self.debug = bool(rospy.get_param('~debug', False))

        self.overlay_pub = rospy.Publisher('/vision/overlay/image_raw', Image, queue_size=1)
        self.mask_pub = rospy.Publisher('/vision/debug_mask', Image, queue_size=1)
        self.error_pub = rospy.Publisher('/vision/target_error', PointStamped, queue_size=1)
        self.info_pub = rospy.Publisher('/vision/target_info', String, queue_size=1)
        self.land_pub = rospy.Publisher('/vision/landing_recommendation', Bool, queue_size=1)
        rospy.Subscriber(rospy.get_param('~mission_state_topic', '/mission/state'),
                         String, self.state_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo('uav_vision: pattern=%s, waiting for %s',
                      self.pattern_color, self.image_topic)

    def state_cb(self, msg):
        value = msg.data.strip().upper()
        self.state = value if value in ('IDLE', 'SEARCH', 'FOLLOW', 'DROP', 'LAND') else 'IDLE'

    def orient(self, frame):
        table = {90: cv2.ROTATE_90_CLOCKWISE,
                 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        if self.rotate in table:
            frame = cv2.rotate(frame, table[self.rotate])
        if self.flip_h:
            frame = cv2.flip(frame, 1)
        if self.flip_v:
            frame = cv2.flip(frame, 0)
        return frame

    def black_mask(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)
        mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, self.adaptive_block, self.adaptive_c)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def red_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo_h, lo_s, lo_v, hi_h, hi_s, hi_v in self.ranges:
            mask |= cv2.inRange(hsv,
                                np.array([lo_h, lo_s, lo_v]),
                                np.array([hi_h, hi_s, hi_v]))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)

    def pattern_mask(self, frame):
        if self.pattern_color == 'red':
            return self.red_mask(frame)
        return self.black_mask(frame)

    @staticmethod
    def touches_border(x, y, w, h, width, height):
        return x <= 1 or y <= 1 or x + w >= width - 2 or y + h >= height - 2

    def find_black_pattern(self, mask):
        height, width = mask.shape[:2]
        count, _labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = []
        for i in range(1, count):
            x, y, w, h, area = [float(v) for v in stats[i]]
            if area < self.min_area or min(w, h) < self.min_side:
                continue
            if max(w, h) > max(width, height) * self.max_side_ratio:
                continue
            if self.touches_border(x, y, w, h, width, height):
                continue
            aspect = w / max(h, 1.0)
            extent = area / max(w * h, 1.0)
            if not (self.aspect_min <= aspect <= self.aspect_max):
                continue
            if not (self.extent_min <= extent <= self.extent_max):
                continue
            distance = 0.0 if self.last_center is None else math.hypot(
                centers[i][0] - self.last_center[0],
                centers[i][1] - self.last_center[1])
            shape = max(0.0, 1.0 - abs(aspect - 1.0) / 0.5)
            confidence = min(1.0, 0.35 + 0.35 * shape +
                             0.30 * min(1.0, area / (self.min_area * 15.0)))
            score = area / (1.0 + distance / 100.0)
            candidates.append((score, float(centers[i][0]), float(centers[i][1]),
                               (w + h) / 4.0, confidence))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        _score, cx, cy, radius, confidence = candidates[0]
        return cx, cy, radius, confidence

    def find_red_ring_pair(self, mask):
        blur = cv2.GaussianBlur(mask, (7, 7), 1.5)
        circles = cv2.HoughCircles(
            blur, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=self.min_r * 0.45, param1=80, param2=16,
            minRadius=int(self.min_r * 0.45), maxRadius=int(self.max_r))
        if circles is None:
            return None
        cs = sorted([tuple(map(float, c)) for c in circles[0]], key=lambda c: c[2])
        best, best_score = None, -1.0
        for inner in cs:
            for outer in cs:
                if outer[2] <= inner[2] or not (self.min_r <= outer[2] <= self.max_r):
                    continue
                distance = math.hypot(inner[0] - outer[0], inner[1] - outer[1])
                scale = outer[2]
                ratio = inner[2] / outer[2]
                if distance > self.center_tol * scale or abs(ratio - self.ratio) > self.ratio_tol:
                    continue
                x, y = int(round(outer[0])), int(round(outer[1]))
                h, w = mask.shape
                d = max(4, int(outer[2] * 0.18))
                if not (d < x < w - d and d < y < h - d):
                    continue
                cross = (mask[y-2:y+3, x-d:x+d].mean() +
                         mask[y-d:y+d, x-2:x+3].mean()) / (2.0 * 255.0)
                score = (1.0 - distance / (self.center_tol * scale + 1e-6) -
                         abs(ratio - self.ratio) / self.ratio_tol + 0.3 * cross)
                if score > best_score:
                    best_score = score
                    best = ((outer[0] + inner[0]) / 2.0,
                            (outer[1] + inner[1]) / 2.0,
                            outer[2],
                            min(1.0, max(0.0, score / 1.3)))
        return best

    def find_ring_pair(self, mask):
        if self.pattern_color == 'red':
            return self.find_red_ring_pair(mask)
        return self.find_black_pattern(mask)

    def image_cb(self, msg):
        try:
            frame = self.orient(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        except CvBridgeError as err:
            rospy.logwarn_throttle(2.0, 'cv_bridge: %s', err)
            return
        mask = self.pattern_mask(frame)
        result = self.find_ring_pair(mask)
        h, w = frame.shape[:2]
        valid = result is not None
        if valid:
            x, y, radius, confidence = result
            if self.last_center is not None and math.hypot(
                    x - self.last_center[0], y - self.last_center[1]) <= self.max_jump:
                self.stable += 1
            else:
                self.stable = 1
            self.last_center = (x, y)
            ex, ey = (x - w / 2.0) / (w / 2.0), (y - h / 2.0) / (h / 2.0)
        else:
            self.stable, self.last_center = 0, None
            x = y = radius = confidence = ex = ey = float('nan')

        land_ok = bool(valid and self.state == 'LAND' and
                       self.stable >= self.need_stable and
                       abs(ex) <= self.land_err and abs(ey) <= self.land_err and
                       radius >= self.land_min_r)

        overlay = frame.copy()
        if valid:
            cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.drawMarker(overlay, (int(x), int(y)), (0, 255, 0),
                           cv2.MARKER_CROSS, 26, 2)
        label = 'STATE=%s valid=%s stable=%d landing_advice=%s' % (
            self.state, valid, self.stable, land_ok)
        cv2.putText(overlay, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 255, 0) if valid else (0, 0, 255), 2)
        if valid:
            cv2.putText(overlay, 'error=(%.3f, %.3f) conf=%.2f' % (ex, ey, confidence),
                        (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
            e = PointStamped(header=msg.header)
            e.point.x, e.point.y, e.point.z = ex, ey, confidence
            self.error_pub.publish(e)

        values = [x, y, radius, confidence, ex, ey] if valid else [None] * 6
        info = {
            'stamp': msg.header.stamp.to_sec(),
            'mission_state': self.state,
            'valid': valid,
            'center_px': values[0:2],
            'outer_radius_px': values[2],
            'error_normalized': values[4:6],
            'confidence': values[3],
            'stable_frames': self.stable,
            'landing_recommendation': land_ok,
        }
        self.info_pub.publish(String(data=json.dumps(info, allow_nan=False)))
        self.land_pub.publish(Bool(data=land_ok))
        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, 'bgr8'))
        if self.debug:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))


if __name__ == '__main__':
    rospy.init_node('vision_node')
    VisionNode()
    rospy.spin()
