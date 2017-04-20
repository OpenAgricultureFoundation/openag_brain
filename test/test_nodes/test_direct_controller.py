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

        self.msgs = [1, 3, 0, 40, 30, 7]
        self._received = []

    def callback(self, item):
        rospy.loginfo("I heard %s", item)
        self._received.append(item)
        self.assertTrue(self.msgs[:len(self._received)] == self._received)

    def test_echo_topic(self):
        rospy.init_node(NAME, anonymous=True)
        self.pub_desired = rospy.Publisher("{}/desired".format(self.namespace),
                                                    Float64, queue_size=10)
        # rospy.sleep(10)
        self.sub_measured = rospy.Subscriber("{}/measured".format(self.namespace),
                                            Float64, self.callback)
        # self.sub_commanded = rospy.Subscriber("{}/commanded".format(self.namespace),
                                            # Float64, self.callback)
        for msg in msgs:
            rospy.loginfo("Published: {:f}".format(msg))
            self.pub_desired.publish(self.msg)
            rospy.sleep(1)
        # rospy.sleep(3)
        #
        rospy.spin() # Run until all messages are received.
        #self.assertTrue(msgs == self._received)
        rospy.loginfo("Received: " + ' '.join(self._received))
        rospy.loginfo("There were " + str(len(self._received)) + " messages received.")
        rospy.loginfo("New Message")
def publisher_node():
    pass

def listener_node():

    def callback(item):
        print("I Heard {}".format(item))
    namespace = "/tests/mock_controller"
    rospy.init_node(NAME, anonymous=True)
    sub_measured = rospy.Subscriber("{}/measured".format(namespace),
                                        Float64, callback)
    sub_commanded = rospy.Subscriber("{}/commanded".format(namespace),
                                        Float64, callback)
    rospy.spin()


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestDirectController)
    # listener_node()
