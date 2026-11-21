#!/usr/bin/env python3
"""把 /vision/target_info 检测结果打印到终端，方便单个串口窗口查看。"""
import json

import rospy
from std_msgs.msg import String


def callback(msg):
    try:
        info = json.loads(msg.data)
    except (ValueError, TypeError):
        rospy.loginfo('[RESULT] %s', msg.data)
        return
    if info.get('valid'):
        cx, cy = info.get('center_px', [float('nan')] * 2)[:2]
        ex, ey = info.get('error_normalized', [float('nan')] * 2)[:2]
        rospy.loginfo(
            '[RESULT] valid=True center=(%.1f, %.1f) r=%.1f error=(%.3f, %.3f) '
            'stable=%d landing=%s',
            cx, cy, info.get('outer_radius_px', 0.0), ex, ey,
            info.get('stable_frames', 0), info.get('landing_recommendation', False))
    else:
        rospy.loginfo('[RESULT] valid=False (未检测到图案)')


def main():
    rospy.init_node('info_logger', anonymous=True)
    rospy.Subscriber('/vision/target_info', String, callback, queue_size=1)
    rospy.loginfo('info_logger 已启动，监听 /vision/target_info')
    rospy.spin()


if __name__ == '__main__':
    main()
