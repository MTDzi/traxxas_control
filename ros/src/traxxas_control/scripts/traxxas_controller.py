#!/usr/bin/env python

from math import asin, degrees, sqrt
import numpy as np
import cv2

import rospy
from std_msgs.msg import Int16, Time
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError


class JoyController:

    def __init__(self):
        self.joy_msg = None
        self.zed_depth_msg = None
        self.time_after_pub = None

        self.bridge = CvBridge()
	self.image_pub = rospy.Publisher("image_topic", Image)

        self.pub_servo = rospy.Publisher('/servo', Int16, queue_size=1)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=1)

        rospy.Subscriber('/joy', Joy, self._joy_cb)
        rospy.Subscriber('/zed/depth/depth_registered', Image, self._zed_depth_cb)
        rospy.Subscriber('/chatter', Time, self._time_cb)

        rospy.init_node('traxxas_controler')

        self.loop()

    def loop(self):
        rate = rospy.Rate(100)
        start_time = 0

        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time

            if self.zed_depth_msg is not None:
                ok_values = ~np.isnan(self.zed_depth_msg) & np.isfinite(self.zed_depth_msg)
                rospy.logwarn('{}'.format(self.zed_depth_msg[ok_values].mean()))

            if self.joy_msg is not None:
                axes = self.joy_msg.axes

                angle = axes[0]
                throt = axes[3]
                angle = int(90 + degrees(asin(angle)))
                throt = int(102*np.sign(throt)*sqrt(abs(throt)))

                self.pub_servo.publish(angle)
                self.pub_esc.publish(throt)

            if self.time_after_pub is not None:
                now = rospy.Time.now().to_sec()
                rospy.logwarn('time diff = {}'.format(now - self.time_after_pub))



    # Callbacks

    def _joy_cb(self, msg):
        self.joy_msg = msg

    def _zed_depth_cb(self, msg):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except CvBridgeError as e:
           print(e)
 
        self.zed_depth_msg = cv_image

    def _time_cb(self, msg):
        self.time_after_pub = msg.data.to_sec()


if __name__ == '__main__':
    try:
        JoyController()
    except rospy.ROSInterruptException:
        rospy.logerr('JoyController failed')
