#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from std_msgs.msg import String
from ros_on_python.msg import Adder

def callback(data):
    print data.arg_x + data.arg_y

def adder():
    rospy.init_node('adder', anonymous=True)
    rospy.Subscriber('input_data', Adder, callback)
    rospy.spin()

if __name__ == '__main__':
    adder()