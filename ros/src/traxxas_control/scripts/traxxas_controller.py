#!/usr/bin/env python

from math import asin, degrees, sqrt
import numpy as np
from random import normalvariate
#import cv2

import rospy
from std_msgs.msg import Int16, Time
from sensor_msgs.msg import Joy, Image
#from cv_bridge import CvBridge, CvBridgeError


class JoyController:

    def __init__(self):
        self.pub_servo = rospy.Publisher('/servo', Int16, queue_size=1)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=1)

        self.servo_neutral = 1479
        self.esc_neutral = 1500

        self.angle_mult = 600 #80 # 600
        self.throt_mult = 200 # 80 # 200

        rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=1)

        rospy.init_node('traxxas_controler')


    # Callbacks

    def _joy_cb(self, msg):
        angle_mult_inc = msg.buttons[4]
        angle_mult_dec = msg.buttons[6]
        throt_mult_inc = msg.buttons[5]
        throt_mult_dec = msg.buttons[7]

        if angle_mult_inc:
            self.angle_mult += 1
            rospy.logwarn('self.angle_mult = {}'.format(self.angle_mult))
        if angle_mult_dec:
            self.angle_mult -= 1
            rospy.logwarn('self.angle_mult = {}'.format(self.angle_mult))

        if throt_mult_inc:
            self.throt_mult += 1
            rospy.logwarn('self.throt_mult = {}'.format(self.throt_mult))
        if throt_mult_dec:
            self.throt_mult -= 1
            rospy.logwarn('self.throt_mult = {}'.format(self.throt_mult))

        servo_neutral_shift = msg.axes[4]
        esc_neutral_shift = msg.axes[5]
        if servo_neutral_shift:
            self.servo_neutral -= servo_neutral_shift
            rospy.logwarn('self.servo_neutral = {}'.format(self.servo_neutral))
        if esc_neutral_shift:
            self.esc_neutral -= esc_neutral_shift
            rospy.logwarn('self.esc_neutral = {}'.format(self.esc_neutral))

        angle = msg.axes[0]
        throt = msg.axes[3]
        angle = self.servo_neutral - int(self.angle_mult*angle)
        throt = self.esc_neutral + int(self.throt_mult*throt)

        self.pub_servo.publish(angle)
        self.pub_esc.publish(throt)


if __name__ == '__main__':
    JoyController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('JoyController failed')
