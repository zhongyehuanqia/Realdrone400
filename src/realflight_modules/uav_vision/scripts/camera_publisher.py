#!/usr/bin/env python3
"""UVC 摄像头采集节点：直接读 /dev/videoX 并发布到 /camera/image_raw。

不依赖 usb_cam ROS 包（板子上没装），只用系统自带的 OpenCV + cv_bridge。
默认 640x480，与离线测试图保持一致，config/vision.yaml 里的参数可直接复用。
"""
import glob
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def find_uvc_device(preferred=None):
    """优先用指定设备；否则扫描 sysfs 找 UVC 摄像头的 video 节点。"""
    if preferred:
        return preferred
    for name_path in sorted(glob.glob('/sys/class/video4linux/video*/name')):
        try:
            with open(name_path) as fh:
                name = fh.read().strip()
        except OSError:
            continue
        low = name.lower()
        # RK3588 自带的 ISP/MIPI 节点名含 rkisp/rkcif/isp/vdec/venc，排除掉
        if any(k in low for k in ('rkisp', 'rkcif', 'isp', 'vdec', 'venc', 'rkvip')):
            continue
        # UVC 摄像头通常叫 'USB Camera' / 'UVC Camera' / 'uvcvideo'
        if 'usb camera' in low or 'uvc' in low:
            dev = '/dev/' + os.path.basename(os.path.dirname(name_path))
            rospy.loginfo('自动找到 UVC 摄像头: %s (%s)', dev, name)
            return dev
    return None


def main():
    rospy.init_node('camera_publisher')
    bridge = CvBridge()
    preferred = rospy.get_param('~device', '') or None
    width = int(rospy.get_param('~width', 640))
    height = int(rospy.get_param('~height', 480))
    fps = float(rospy.get_param('~fps', 15.0))

    device = find_uvc_device(preferred)
    if not device:
        rospy.logerr('没找到 UVC 摄像头。请手动指定 device:=/dev/videoX')
        return

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        rospy.logerr('无法打开 %s，试试换一个 /dev/videoX 或检查是否被占用', device)
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    rospy.loginfo('摄像头已打开: %s  %dx%d @ %.1ffps', device, width, height, fps)

    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(2.0, '读帧失败，摄像头可能掉线')
            rate.sleep()
            continue
        try:
            msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        except CvBridgeError as err:
            rospy.logwarn_throttle(2.0, 'cv_bridge: %s', err)
            rate.sleep()
            continue
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        pub.publish(msg)
        rate.sleep()
    cap.release()


if __name__ == '__main__':
    main()
