#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


THROTTLE_MAXREVERSE = 1000
THROTTLE_MAXFORWARD = 2000
THROTTLE_IDLE = 1500


def mover():
    pub_servo = rospy.Publisher('/servo', Int16)
    pub_esc = rospy.Publisher('/esc', Int16)

    sub_joy = rospy.Subscriber('/joy', Joy)

    rospy.init_node('/traxxas_controler')
    rate = rospy.Rate(50)
    start_time = 0

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time

        angle = sub_joy()
        pub_servo.publish(angle)
        pub_esc.publish(angle)

        rate.sleep()



if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
