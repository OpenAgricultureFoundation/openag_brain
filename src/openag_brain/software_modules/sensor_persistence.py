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

class SinglePersistence:
    def __init__(self, db, topic, topic_type, environment, variable, is_desired):
        self.db = db
        self.environment = environment
        self.variable = variable
        self.is_desired = is_desired
        self.last_value = None
        self.last_time = 0
        self.sub = rospy.Subscriber(topic, topic_type, self.on_data)
        self.min_update_interval = 5
        self.max_update_interval = 600

    def on_data(self, item):
        curr_time = time.time()
        value = item.data
        # This is kind of a hack to correctly interpret UInt8MultiArray
        # messages. There should be a better way to do this
        if item._slot_types[item.__slots__.index('data')] == "uint8[]":
            value = [ord(x) for x in value]

        # Throttle updates
        delta_time = curr_time - self.last_time
        if delta_time < self.min_update_interval:
            return
        if delta_time < self.max_update_interval and self.last_value:
            delta_val = value - self.last_value
            if abs(delta_val / self.last_value) <= 0.01:
                return

        # Save the data point
        point = EnvironmentalDataPoint({
            "environment": self.environment,
            "variable": self.variable,
            "is_desired": self.is_desired,
            "value": value,
            "timestamp": curr_time
        })
        point_id = self.gen_doc_id(curr_time)
        self.db[point_id] = point

        self.last_value = value
        self.last_time = curr_time

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))

class EnvironmentPersistence:
    def __init__(self, server, environment):
        self.db = server[ENVIRONMENTAL_DATA_POINT]
        self.environment = environment
        self.children = {}
        self.valid_variables = [var.name for var in EnvVar.items]

    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            time.sleep(5)

    def update_subscribers(self):
        # Look at all of the topics that are being published in the namespace
        namespace = "/{}/".format(self.environment)
        for topic, topic_type in rospy.get_published_topics(namespace):
            topic_type = resolve_message_type(topic_type)

            # Ignore topics we are already listening to
            if topic in self.children:
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
            self.children[topic] = SinglePersistence(
                self.db, topic, topic_type, self.environment, variable,
                is_desired
            )

if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    rospy.init_node('sensor_persistence')
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Persistence module cannot be run in the global namespace. "
            "Please designate an environment for this module."
        )
    environment = namespace.split('/')[-2]
    mod = EnvironmentPersistence(server, environment)
    mod.run()
