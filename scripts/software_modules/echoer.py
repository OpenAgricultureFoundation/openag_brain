#!/usr/bin/python

import rospy
from std_msgs.msg import String
from openag_brain.srv import ChangeString

global msg
msg = "test"

def srv_callback(req):
    global msg
    msg = req.data
    return msg

pub = rospy.Publisher('output', String, queue_size=10)
srv = rospy.Service('change_msg', ChangeString, srv_callback)

if __name__ == '__main__':
    rospy.init_node('echoer')
    rate = rospy.get_param('~rate')
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()
