#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_recipe_handler'

import sys, os
import unittest

import rospy
from std_msgs.msg import Float64
DIR_NAME = os.path.dirname(__file__)
print(sys.path.append(os.path.join(DIR_NAME, '../..')))
print(os.getcwd())
from nodes.recipe_handler import interpret_simple_recipe

from time import time

MOCK_RECIPE_A = {
            "format": "simple",
            "version": "0.0.1",
            "operations": [
                            [0, "air_temperature", 24],
                            [0, "water_temperature", 22]
                          ]
            }


class TestRecipeInterpreter(unittest.TestCase):

    def test_interpret_simple_recipe():
        now_time = time()
        start_time = now_time - 100
        setpoints = interpet_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
        print("Recipe Handler returns expected number of setpoints")
        assert len(setpoints) == 2


class TestRecipeHandler(unittest.TestCase):
    """
    Test the direct_controller.py node to verify it is returning the topics
    correctly.
    """
    def setUp(self):
        self.namespace = "test_recipe"
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
    rostest.rosrun(PKG, NAME, TestRecipeHandler)

