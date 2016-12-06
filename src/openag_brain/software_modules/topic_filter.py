#!/usr/bin/python
import rospy
import rosgraph

from re import match
from roslib.message import get_message_class

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

def gen_sensor_topic_desc(pubs):
    """
    Generate a description dict for topics that are sensor topics.
    Matches, filters and parses topic names to find sensor information.
    """
    for topic, topic_type in pubs:
        result = match("/sensors/(\w+)/(\w+)/raw", topic)
        if result:
            yield {
                "module_id": result.groups(1),
                "output_name": result.groups(2),
                "topic": topic,
                "type": topic_type
            }

def filter_all_topics(pubs):
    """
    Given an iterator publishers, where each publisher is a two-tuple
    `(topic, type)`, create a filtered topic endpoint.
    """
    for desc in gen_sensor_topic_desc(pubs):
        src_topic_type = get_message_class(desc["type"])
        src_topic = desc["topic"]
        module_id = desc["module_id"]
        output_name = desc["output_name"]
        dest_topic = "/sensors/{}/{}/filtered".format(module_id, output_name)
        filter_topic(src_topic, dest_topic, src_topic_type)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    rostopic_master = rosgraph.Master("/rostopic")
    pubs, subs, _ = rostopic_master.getSystemState()
    filter_all_topics(pubs)
    rospy.spin()
