#!/usr/bin/python
import rospy

from std_msgs.msg import Float64
from openag.var_types import EnvVar, GROUP_ENVIRONMENT, WATER_LEVEL_HIGH

# Filter a list of environmental variables that are specific to environment
# sensors and actuators
ENVIRONMENT_VARIABLES = tuple(
    var for var in EnvVar.items.values()
    if GROUP_ENVIRONMENT in var.groups
)

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
        if env_var == WATER_LEVEL_HIGH:
            forward_topic(src_topic, dest_topic, Float64)
        else:
            filter_topic(src_topic, dest_topic, Float64)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    # Make sure that we're under an environment namespace.
    filter_all_variable_topics(ENVIRONMENT_VARIABLES)
    rospy.spin()
