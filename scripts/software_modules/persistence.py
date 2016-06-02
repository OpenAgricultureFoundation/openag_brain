#!/usr/bin/python

import sys
import time
import random
import threading
from importlib import import_module

import rospy
import rostopic
from couchdb import Server

from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.db_names import DbName

def resolve_message_type(msg_type):
    package, cls = msg_type.split('/')
    mod = import_module('.msg', package)
    return getattr(mod, cls)

class Persistence:
    def __init__(self):
        self.subscribers = {}
        rospy.init_node('persistence')
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            raise RuntimeError(
                "Persistence module cannot be run in the global namespace. "
                "Please designate an environment for this module."
            )
        self.environment = self.namespace.split('/')[-2]
        server = Server()
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT]
        self.update_subscribers()

    def update_subscribers(self):
        for topic, topic_type in rospy.get_published_topics(self.namespace):
            topic_type = resolve_message_type(topic_type)
            if topic in self.subscribers:
                continue
            variable = topic.split('/')[-1]
            if variable.startswith('desired_'):
                def callback(item, variable=variable[8:], mod=self):
                    return mod.on_desired_data(item, variable)
                self.subscribers[topic] = rospy.Subscriber(
                    topic, topic_type, callback
                )
            else:
                def callback(item, variable=variable, mod=self):
                    return mod.on_measured_data(item, variable)
                self.subscribers[topic] = rospy.Subscriber(
                    topic, topic_type, callback
                )
        threading.Timer(5, self.update_subscribers)

    def on_desired_data(self, item, variable):
        curr_time = time.time()
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=variable,
            is_desired=True,
            value=item.data,
            timestamp=curr_time
        )
        point["_id"] = self.gen_doc_id(curr_time)
        point.store(self.db)

    def on_measured_data(self, item, topic):
        curr_time = time.time()
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=topic,
            is_desired=False,
            value=item.data,
            timestamp=curr_time
        )
        point["_id"] = self.gen_doc_id(curr_time)
        point.store(self.db)

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))


if __name__ == '__main__':
    mod = Persistence()
    rospy.spin()
