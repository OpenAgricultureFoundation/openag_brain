#!/usr/bin/python
import rospy

from std_msgs.msg import Float64
from openag_brain.var_types import SENSOR_VARIABLES

class EWMA:
    def __init__(self, a):
        self.a = a
        self.average = None

    def __call__(self, sample):
        # Update the average
        if self.average is None:
            self.average = sample
            return
        self.average = self.a * sample + (1 - self.a) * self.average

def filter_topic(src_topic, dest_topic, topic_type):
    rospy.loginfo("Filtering topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    f = EWMA(0.3)
    def callback(src_item):
        f(src_item.data)
        dest_item = topic_type(f.average)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub

def filter_all_variable_topics(variables):
    """
    Given an iterator publishers, where each publisher is a two-tuple
    `(topic, type)`, publishes a filtered topic endpoint.
    """
    for env_var, MsgType in variables:
        src_topic = "{}/raw".format(env_var)
        dest_topic = "{}/measured".format(env_var)
        # Ignore type associated with environmental variable type and
        # coerce to Float64
        filter_topic(src_topic, dest_topic, Float64)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    # Make sure that we're under an environment namespace.
    filter_all_variable_topics(SENSOR_VARIABLES)
    rospy.spin()
