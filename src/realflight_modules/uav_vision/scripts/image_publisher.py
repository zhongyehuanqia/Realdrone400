#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线图片发布器：循环发布一张测试图到 /camera/image_raw，
让 vision_node 不用接真实相机也能测试整条检测链路。"""
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

rospy.init_node('image_publisher')
pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
bridge = CvBridge()

img_path = rospy.get_param('~image_path', '')
rate_hz = float(rospy.get_param('~rate', 15.0))
if not img_path or not os.path.exists(img_path):
    rospy.logerr('图片不存在: %s' % img_path)
    raise SystemExit(1)

img = cv2.imread(img_path)
if img is None:
    rospy.logerr('无法读取图片: %s' % img_path)
    raise SystemExit(1)

rospy.loginfo('发布测试图 %s (%dx%d) @ %.0fHz 到 /camera/image_raw' %
              (img_path, img.shape[1], img.shape[0], rate_hz))
r = rospy.Rate(rate_hz)
while not rospy.is_shutdown():
    pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    r.sleep()
