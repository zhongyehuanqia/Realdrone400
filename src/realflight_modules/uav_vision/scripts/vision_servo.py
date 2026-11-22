#!/usr/bin/env python3
"""Turn uav_vision image error into this aircraft's position command.

Detection stays in vision_node.py. This node only:
  reads /Odom_high_freq and /vision/target_error,
  publishes /mission/state and /position_cmd,
  and, after the vision node allows it, stops the position stream and
  publishes /px4ctrl/takeoff_land.

/Odom_high_freq attitude is already in the airframe, with the lidar mount
pitch removed, so horizontal velocity is along the nose and the wing, not
the tilted lidar axis.
"""
import math

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Bool, String


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z)


class VisionServo(object):
    IDLE, SEARCH, FOLLOW, ALIGN, COMMIT = range(5)

    def __init__(self):
        self.odom = None
        self.ex = self.ey = None
        self.error_stamp = None
        self.land_ok = False
        self.land_stamp = None
        self.phase = self.IDLE
        self.setpoint = None
        self.hold_z = None
        self.aligned_since = None
        self.commit_time = None
        self.land_sent = False
        self.last_tick = None
        self.cmd_vel = (0.0, 0.0)

        self.kp = float(rospy.get_param('~kp', 0.5))
        self.max_speed = float(rospy.get_param('~max_speed', 0.35))
        self.deadzone = float(rospy.get_param('~deadzone', 0.02))
        self.max_leash = float(rospy.get_param('~max_leash', 0.6))
        self.sign_ex = float(rospy.get_param('~sign_ex', -1.0))
        self.sign_ey = float(rospy.get_param('~sign_ey', -1.0))
        self.error_timeout = float(rospy.get_param('~error_timeout', 0.4))
        self.search_height = float(rospy.get_param('~search_height', 0.8))
        self.align_error = float(rospy.get_param('~align_error', 0.08))
        self.align_hold = float(rospy.get_param('~align_hold_sec', 1.0))
        self.cmd_stop = float(rospy.get_param('~cmd_stop_sec', 0.7))

        self.mission_pub = rospy.Publisher(
            rospy.get_param('~mission_topic', '/mission/state'), String, queue_size=1)
        self.cmd_pub = rospy.Publisher(
            rospy.get_param('~cmd_topic', '/position_cmd'), PositionCommand, queue_size=1)
        self.land_pub = rospy.Publisher(
            rospy.get_param('~takeoff_land_topic', '/px4ctrl/takeoff_land'),
            TakeoffLand, queue_size=1)

        rospy.Subscriber(rospy.get_param('~odom_topic', '/Odom_high_freq'),
                         Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~error_topic', '/vision/target_error'),
                         PointStamped, self.error_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~land_topic', '/vision/landing_recommendation'),
                         Bool, self.land_cb, queue_size=1)

        rate = float(rospy.get_param('~publish_rate', 20.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo('vision_servo: odom=%s, position command on %s',
                      rospy.get_param('~odom_topic', '/Odom_high_freq'),
                      rospy.get_param('~cmd_topic', '/position_cmd'))

    def odom_cb(self, msg):
        self.odom = msg

    def error_cb(self, msg):
        self.ex = msg.point.x
        self.ey = msg.point.y
        self.error_stamp = rospy.Time.now()

    def land_cb(self, msg):
        self.land_ok = bool(msg.data)
        self.land_stamp = rospy.Time.now()

    def set_phase(self, phase, mission):
        if phase != self.phase:
            rospy.loginfo('vision_servo: %s -> %s',
                          self.phase_name(self.phase), self.phase_name(phase))
            self.phase = phase
        self.mission_pub.publish(String(data=mission))

    @staticmethod
    def phase_name(phase):
        return ('IDLE', 'SEARCH', 'FOLLOW', 'ALIGN', 'COMMIT')[phase]

    def error_fresh(self, now):
        return (self.error_stamp is not None and
                (now - self.error_stamp).to_sec() <= self.error_timeout)

    def land_fresh(self, now):
        return (self.land_ok and self.land_stamp is not None and
                (now - self.land_stamp).to_sec() <= self.error_timeout)

    def tick(self, _event):
        now = rospy.Time.now()
        if self.odom is None:
            self.set_phase(self.IDLE, 'IDLE')
            return

        if self.phase == self.COMMIT:
            self.mission_pub.publish(String(data='LAND'))
            if (not self.land_sent and self.commit_time is not None and
                    (now - self.commit_time).to_sec() >= self.cmd_stop):
                cmd = TakeoffLand()
                cmd.takeoff_land_cmd = TakeoffLand.LAND
                self.land_pub.publish(cmd)
                self.land_sent = True
                rospy.loginfo('vision_servo: sent px4ctrl LAND')
            return

        height = self.odom.pose.pose.position.z
        if height < self.search_height:
            self.aligned_since = None
            self.setpoint = None
            self.set_phase(self.IDLE, 'IDLE')
            return

        if not self.error_fresh(now):
            self.aligned_since = None
            self.setpoint = None
            self.set_phase(self.SEARCH, 'SEARCH')
            return

        small = abs(self.ex) <= self.align_error and abs(self.ey) <= self.align_error
        if small:
            if self.aligned_since is None:
                self.aligned_since = now
        else:
            self.aligned_since = None

        aligned = (self.aligned_since is not None and
                   (now - self.aligned_since).to_sec() >= self.align_hold)
        if aligned:
            self.set_phase(self.ALIGN, 'LAND')
            if self.land_fresh(now):
                self.commit_time = now
                self.set_phase(self.COMMIT, 'LAND')
                rospy.loginfo('vision_servo: stop position commands, land in %.2fs', self.cmd_stop)
                return
        else:
            self.set_phase(self.FOLLOW, 'FOLLOW')

        self.servo(now)

    def servo(self, now):
        p = self.odom.pose.pose.position
        if self.setpoint is None:
            self.setpoint = [p.x, p.y, p.z]
            self.hold_z = p.z
            self.last_tick = now
            self.cmd_vel = (0.0, 0.0)

        dt = 0.05 if self.last_tick is None else (now - self.last_tick).to_sec()
        self.last_tick = now
        dt = min(max(dt, 0.0), 0.1)

        ex = 0.0 if abs(self.ex) < self.deadzone else self.ex
        ey = 0.0 if abs(self.ey) < self.deadzone else self.ey
        v_fwd = self.sign_ey * self.kp * ey
        v_left = self.sign_ex * self.kp * ex
        speed = math.hypot(v_fwd, v_left)
        if speed > self.max_speed and speed > 1e-6:
            scale = self.max_speed / speed
            v_fwd *= scale
            v_left *= scale

        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * v_fwd - s * v_left
        vy = s * v_fwd + c * v_left
        self.setpoint[0] += vx * dt
        self.setpoint[1] += vy * dt
        self.setpoint[2] = self.hold_z

        dx = self.setpoint[0] - p.x
        dy = self.setpoint[1] - p.y
        dist = math.hypot(dx, dy)
        if dist > self.max_leash and dist > 1e-6:
            scale = self.max_leash / dist
            self.setpoint[0] = p.x + dx * scale
            self.setpoint[1] = p.y + dy * scale

        self.cmd_vel = (vx, vy)
        cmd = PositionCommand()
        cmd.header.stamp = now
        cmd.header.frame_id = 'world'
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.position.x = self.setpoint[0]
        cmd.position.y = self.setpoint[1]
        cmd.position.z = self.hold_z
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = 0.0
        cmd.yaw = yaw
        cmd.yaw_dot = 0.0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('vision_servo')
    VisionServo()
    rospy.spin()
