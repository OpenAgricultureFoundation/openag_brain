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
        self.namespace = "mock_controller"

        self.msgs = [99, 43, 0, 40, 30, 7]
        self._received = []

    def callback(self, item):
        rospy.loginfo("I heard %s", item)
        self._received.append(int(item.data))
        rospy.loginfo(self.msgs[:len(self._received)])
        rospy.loginfo(self._received)
        self.assertTrue(self.msgs[:len(self._received)] == self._received)

    def test_echo_topic(self):
        rospy.init_node(NAME, anonymous=True)
        self.pub_desired = rospy.Publisher("{}/desired".format(self.namespace),
                                                    Float64, queue_size=10)
        self.sub_measured = rospy.Subscriber("{}/measured".format(self.namespace),
                                            Float64, self.callback)
        rospy.sleep(15)  # Wait for subscriber to be ready to receive messages
        for msg in self.msgs:
            rospy.loginfo("Published: {:f}".format(msg))
            self.pub_desired.publish(msg)
        rospy.sleep(15)  # Wait until all messages are received.
        
        self.assertTrue(self.msgs == self._received)
        rospy.loginfo("Received: " + ' '.join([str(x) for x in self._received]))
        rospy.loginfo("There were " + str(len(self._received)) + " messages received.")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestDirectController)
