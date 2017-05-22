#!/usr/bin/python
import rospy

from std_msgs.msg import Float64
from openag_brain.load_env_var_types import create_variables
from openag_brain.constants import SENTINELS

# Filter a list of environmental variables that are specific to environment
# sensors and actuators
ENVIRONMENT_VARIABLES = create_variables(rospy.get_param('/var_types/environment_variables'))

class EWMA:
    """
    Calculate the Exponentially Weighted Moving Average (EWMA) for an input variable.
    EWMAi = alpha * Xi + (1 - alpha) * EWMAi-1
    Params:
        a : Alpha is the dapening coefficient. The lower the value the less damped. Should be between 0 and 1
    """
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
    """
    Publishes a measured data point given the raw value by applying the EWMA function to the data.
    :param src_topic: String? - Source topic signal to be filtered
    :param dest_topic: String? - Output topic to publish new data to
    :param topic_type: The data type of the topic, aka Float64, Int32, etc
    :return: sub, pub : subscribed topic (raw measured value) and published topic (filtered (smoothed) value)
    """
    rospy.loginfo("Filtering topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    f = EWMA(0.3)
    def callback(src_item):
        value = src_item.data
        # If the value is our magic number, leave it alone
        if value in SENTINELS:
            dest_item = value
        else:
            f(src_item.data)
            dest_item = topic_type(f.average)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub


def forward_topic(src_topic, dest_topic, topic_type):
    rospy.loginfo("Forwarding topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    def callback(src_item):
        dest_item = topic_type(src_item.data)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub


def filter_all_variable_topics(variables):
    """
    Given an iterator publishers, where each publisher is a two-tuple
    `(topic, type)`, publishes a filtered topic endpoint.
    """
    for env_var in variables:
        src_topic = "{}/raw".format(env_var)
        dest_topic = "{}/measured".format(env_var)
        # Ignore type associated with environmental variable type and
        # coerce to Float64

        # @FIXME this is a short-term fix for preventing boolean values from
        # being filtered by the EWMA filter.
        #
        # Explanation: right now all topics under `/environment/<id>` are
        # float64 type, with Boolean being 1 or 0. This was judged to be a
        # simpler architecture at the time. Values from sensors may be any
        # type, but are coerced to Float64. The same is true for actuators.
        # However, this assumption breaks down for filtering boolean values,
        # since the EWMA will produce fractional numbers that don't coerce
        # correctly back to boolean.
        #
        # In future, we should change the architecture of the system to support
        # standard ros types under `/environment/<id>`.
        if env_var.type is None or 'boolean' in env_var.type.lower():
            forward_topic(src_topic, dest_topic, Float64)
        else:
            filter_topic(src_topic, dest_topic, Float64)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    # Make sure that we're under an environment namespace.
    filter_all_variable_topics(ENVIRONMENT_VARIABLES.values())
    rospy.spin()
