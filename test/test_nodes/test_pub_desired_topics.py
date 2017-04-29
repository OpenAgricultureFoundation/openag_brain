#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_pub_desired_topics'

import sys
import unittest

import rospy
from std_msgs.msg import Float64


NUM_TIMES_TO_PUBLISH = 1

class TestPublishToDesiredTopics(unittest.TestCase):

    def setUp(self):
        self.namespace = rospy.get_namespace()
        rospy.logdebug("Initializing test_publish_to_topics in namespace:" +
                        self.namespace)
        self.variables = [("air_flush_on", 1),
                          ("air_temperature", 1),
                          ("light_intensity_blue", 1),
                          ("light_intensity_red", 1),
                          ("light_intensity_white", 1),
                          ("nutrient_flora_duo_a", 5),
                          ("nutrient_flora_duo_b", 5),
                          ("water_potential_hydrogen", 6)]
        # self.topic_ending = ["raw", "measured", "commanded", "desired"]
        rospy.init_node(NAME, anonymous=True)

    def test_publish_to_topics(self):
        topic_ending = "desired"
        for variable, value in self.variables:
            # Publish to each variable/desired topic to see if all of the
            # actuators come on as expected.
            topic_string = variable + "/" + topic_ending
            rospy.logdebug("Testing Publishing to " + topic_string)
            pub_desired = rospy.Publisher(topic_string,
                                               Float64, queue_size=10)
            for _ in range(NUM_TIMES_TO_PUBLISH):
                pub_desired.publish(value)
                rospy.sleep(1)
            pub_desired.publish(0)
            rospy.sleep(5.)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestPublishToDesiredTopics)
