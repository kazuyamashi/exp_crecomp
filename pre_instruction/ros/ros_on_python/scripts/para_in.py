#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from std_msgs.msg import String
from ros_on_python.msg import Adder

def para_in():
    pub = rospy.Publisher('input_data', Adder, queue_size=100)
    rospy.init_node('para_in', anonymous=True)

    r = rospy.Rate(5)
    para_x = 0
    para_y = 2
    msg = Adder()

    while not rospy.is_shutdown():

        msg.arg_x = para_x
        msg.arg_y = para_y

        pub.publish(msg)
        print "published arg_x=%d arg_y=%d"%(msg.arg_x,msg.arg_y)
        para_x += 1
        para_y += 1

        r.sleep()

if __name__ == '__main__':
    try:
        para_in()
    except rospy.ROSInterruptException: pass