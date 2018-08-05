#!/usr/bin/env python

from math import asin, degrees, sqrt
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


THROTTLE_MAXREVERSE = 1000
THROTTLE_MAXFORWARD = 2000
THROTTLE_IDLE = 1500


joy = None


def sgn(x):
    return 0 if x == 0 else x / abs(x)


def joy_cb(msg):
    global joy
    joy = msg


def loop():
    pub_servo = rospy.Publisher('/servo', Int16, queue_size=1)
    pub_esc = rospy.Publisher('/esc', Int16, queue_size=1)

    rospy.Subscriber('/joy', Joy, joy_cb)

    rospy.init_node('traxxas_controler')
    rate = rospy.Rate(50)
    start_time = 0

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time

        if joy is not None:
            axes = joy.axes

            angle = int(90 + degrees(asin(axes[0])))
            throt = int(102*sgn(axes[3])*sqrt(abs(axes[3])))

            pub_servo.publish(angle)
            pub_esc.publish(throt)

        rate.sleep()



if __name__ == '__main__':
    try:
        JoyController()
    except rospy.ROSInterruptException:
        pass
