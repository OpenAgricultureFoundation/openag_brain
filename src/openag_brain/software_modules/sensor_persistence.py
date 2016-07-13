#!/usr/bin/python
"""
The `sensor_persistence.py` module listens for measurements of the ambient
conditions of an environment and writes those measurements to the CouchDB
instance. There should be exactly one instance of this module per environment
in the system.
"""

import sys
import time
import random
from importlib import import_module

import rospy
import rostopic
from couchdb import Server

from openag_brain import params
from openag_brain.util import resolve_message_type
from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.db_names import DbName
from openag_brain.var_types import EnvironmentalVariable

class Persistence:
    def __init__(self, server):
        rospy.init_node('persistence')
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            raise RuntimeError(
                "Persistence module cannot be run in the global namespace. "
                "Please designate an environment for this module."
            )
        self.environment = self.namespace.split('/')[-2]
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT]
        self.subscribers = {}
        self.valid_variables = [
            v for k,v in EnvironmentalVariable.__dict__.items() if k.isupper()
        ]
        self.last_desired_data = {}
        self.last_measured_data = {}

    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            time.sleep(5)

    def update_subscribers(self):
        # Look at all of the topics that are being published in the namespace
        for topic, topic_type in rospy.get_published_topics(self.namespace):
            topic_type = resolve_message_type(topic_type)

            # Ignore topics we are already listening to
            if topic in self.subscribers:
                continue
            variable = topic.split('/')[-1]

            # Ignore actuator commands
            if variable.endswith('_cmd'):
                continue

            # Identify topics for set points
            is_desired = False
            if variable.startswith('desired_'):
                variable = variable[8:]
                is_desired = True

            # Ignore topics that aren't named after valid variables
            if not variable in self.valid_variables:
                continue

            # Subscribe to the topics
            def callback(item, variable=variable, is_desired=is_desired):
                return self.on_data(item, variable, is_desired)
            self.subscribers[topic] = rospy.Subscriber(
                topic, topic_type, callback
            )

    def on_data(self, item, variable, is_desired):
        curr_time = time.time()
        value = item.data
        # This is kind of a hack to correctly interpret UInt8MultiArray
        # messages. There should be a better way to do this
        if item._slot_types[item.__slots__.index('data')] == "uint8[]":
            value = [ord(x) for x in value]
        if is_desired and self.last_desired_data.get(variable, None) == value:
            return
        elif self.last_measured_data.get(variable, None) == value:
            return
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=variable,
            is_desired=is_desired,
            value=value,
            timestamp=curr_time
        )
        point["_id"] = self.gen_doc_id(curr_time)
        point.store(self.db)
        if is_desired:
            self.last_desired_data[variable] = value
        else:
            self.last_measured_data[variable] = value

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))


if __name__ == '__main__':
    server = Server(rospy.get_param('/' + params.DB_SERVER))
    mod = Persistence(server)
    mod.run()
