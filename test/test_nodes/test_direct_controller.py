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
                                            Float64, queue_size=None)
        self._received = []

    def callback(self, item):
        self._received.append(item)

    def test_echo_topic(self):
        msgs = [1, 3, 0, 40, 30, 7]
        self.sub_measured = rospy.Subscriber("{}/measured".format(self.namespace),
                                            Float64, self.callback)
        for msg in msgs:
            print(msg)
            self.pub_desired(msg)
        rospy.sleep(5)
        print(_received)
        self.assertTrue(all(msgs == self._received))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestDirectController)
