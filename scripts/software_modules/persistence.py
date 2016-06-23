#!/usr/bin/python

import sys
import time
import random
import threading
from importlib import import_module

import rospy
import rostopic
from couchdb import Server

from openag_brain import params
from openag_brain.util import resolve_message_type
from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.db_names import DbName

class Persistence:
    def __init__(self, server):
        self.subscribers = {}
        rospy.init_node('persistence')
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            raise RuntimeError(
                "Persistence module cannot be run in the global namespace. "
                "Please designate an environment for this module."
            )
        self.environment = self.namespace.split('/')[-2]
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT]
        self.update_subscribers()
        self.last_desired_data = {}
        self.last_measured_data = {}

    def update_subscribers(self):
        if rospy.is_shutdown():
            return
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
        threading.Timer(5, self.update_subscribers).start()

    def on_desired_data(self, item, variable):
        curr_time = time.time()
        value = item.data
        if self.last_desired_data.get(variable, None) == value:
            return
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=variable,
            is_desired=True,
            value=value,
            timestamp=curr_time
        )
        point["_id"] = self.gen_doc_id(curr_time)
        point.store(self.db)
        self.last_desired_data[variable] = value

    def on_measured_data(self, item, variable):
        curr_time = time.time()
        value = item.data
        if self.last_measured_data.get(variable, None) == value:
            return
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=variable,
            is_desired=False,
            value=item.data,
            timestamp=curr_time
        )
        point["_id"] = self.gen_doc_id(curr_time)
        point.store(self.db)
        self.last_measured_data[variable] = value

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))


if __name__ == '__main__':
    server = Server(rospy.get_param('/' + params.DB_SERVER))
    mod = Persistence(server)
    rospy.spin()
