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

from openag.cli.config import config as cli_config
from openag.models import EnvironmentalDataPoint
from openag.db_names import ENVIRONMENTAL_DATA_POINT
from openag.var_types import EnvVar

from openag_brain import params
from openag_brain.util import resolve_message_type

class Persistence:
    def __init__(self, server):
        rospy.init_node('persistence')
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            pass
            # raise RuntimeError(
            #     "Persistence module cannot be run in the global namespace. "
            #     "Please designate an environment for this module."
            # )
        self.environment = self.namespace.split('/')[-2]

        self.namespace = "/environment_1"
        self.environment = "environment_1"

        self.db = server[ENVIRONMENTAL_DATA_POINT]
        self.subscribers = {}
        self.valid_variables = [var.name for var in EnvVar.items]
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

            # Parse the topic name
            topic_parts = topic.split('/')
            if topic_parts[2] == "desired":
                is_desired = True
            elif topic_parts[2] == "measured":
                is_desired = False
            elif topic_parts[2] == "commanded":
                continue
            else:
                rospy.logwarn(
                    'Encountered invalid topic: "{}"'.format(topic)
                )
                continue
            variable = topic_parts[3]

            # Ignore topics that aren't named after valid variables
            if not variable in self.valid_variables:
                rospy.logwarn(
                    'Topic references invalid variable "{}"'.format(variable)
                )
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
        point = EnvironmentalDataPoint({
            "environment": self.environment,
            "variable": variable,
            "is_desired": is_desired,
            "value": value,
            "timestamp": curr_time
        })
        point_id = self.gen_doc_id(curr_time)
        self.db[point_id] = point
        if is_desired:
            self.last_desired_data[variable] = value
        else:
            self.last_measured_data[variable] = value

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))


if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    mod = Persistence(server)
    mod.run()
