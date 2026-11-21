#!/usr/bin/env python3
"""Simple ground-side view. Run on a desktop with ROS networking, not necessarily on Orange Pi."""
import json
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class GroundStation:
    def __init__(self):
        self.bridge, self.info = CvBridge(), 'waiting for /vision/target_info'
        rospy.Subscriber('/vision/target_info', String, self.info_cb, queue_size=1)
        rospy.Subscriber('/vision/overlay/image_raw', Image, self.image_cb, queue_size=1)

    def info_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.info = 'state=%s valid=%s stable=%s landing_advice=%s' % (data['mission_state'], data['valid'], data['stable_frames'], data['landing_recommendation'])
        except (ValueError, KeyError):
            self.info = msg.data[:120]

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.rectangle(frame, (0, frame.shape[0]-34), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)
            cv2.putText(frame, self.info, (8, frame.shape[0]-11), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
            cv2.imshow('UAV vision ground station (q to close)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('operator closed display')
        except Exception as err:
            rospy.logwarn_throttle(2.0, 'display: %s', err)


if __name__ == '__main__':
    rospy.init_node('uav_vision_ground_station')
    GroundStation()
    rospy.spin()
    cv2.destroyAllWindows()
