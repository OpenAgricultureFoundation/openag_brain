#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_direct_controller'

import sys
import unittest

import rospy
from std_msgs.msg import Float64


class TestDirectController(unittest.TestCase):
    """
    Test the direct_controller.py node to verify it is returning the topics
    correctly.
    """
    def setUp(self):
        self.namespace = "/tests/direct_controller/mock_controller"
        self.pub_desired = rospy.Publisher("{}/desired".format(self.namespace),
                                            Float64, queue_size=10)
        self._received = []

    def callback(self, item):
        self._received.append(item)

    def test_echo_topic(self):
        rospy.init_node(NAME, anonymous=True)
        msgs = [1, 3, 0, 40, 30, 7]
        self.sub_measured = rospy.Subscriber("{}/measured".format(self.namespace),
                                            Float64, self.callback)
        for msg in msgs:
            rospy.loginfo("Published: {:f}".format(msg))
            self.pub_desired.publish(msg)
        rospy.sleep(3)
        rospy.loginfo("Received: " + ' '.join(self._received) + ".")
        rospy.loginfo("There were " + str(len(self._received)) + " messages received.")
        self.assertTrue(msgs == self._received)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestDirectController)
