#!/usr/bin/python

import rospy
from std_msgs.msg import String

def sub_callback(data):
    rospy.loginfo(data)

sub = rospy.Subscriber('input', String, sub_callback)

if __name__ == '__main__':
    rospy.init_node('printer')
    rospy.spin()
