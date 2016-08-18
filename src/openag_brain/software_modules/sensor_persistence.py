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
from openag.models import (
    FirmwareModule, FirmwareModuleType, EnvironmentalDataPoint
)
from openag.db_names import (
    FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE, ENVIRONMENTAL_DATA_POINT
)
from openag.var_types import EnvVar

from openag_brain import params
from openag_brain.util import resolve_message_type

class TopicPersistence:
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

    def stop(self):
        self.sub.unregister()

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

def create_persistence_objects(server):
    env_var_db = server[ENVIRONMENTAL_DATA_POINT]
    module_db = server[FIRMWARE_MODULE]
    module_type_db = server[FIRMWARE_MODULE_TYPE]
    modules = {
        module_id: FirmwareModule(module_db[module_id]) for module_id in
        module_db if not module_id.startswith('_')
    }
    res = []
    valid_vars = [var.name for var in EnvVar.items]
    for module_id, module_info in modules.items():
        module_type = FirmwareModuleType(module_type_db[module_info["type"]])
        for output_name, output_info in module_type["outputs"].items():
            if not output_name in valid_vars:
                rospy.logwarn(
                    'Encountered module output "{}" whose name is not an '
                    "environmental variable".format(output_name)
                )
            topic = "/sensors/{}/{}/filtered".format(module_id, output_name)
            topic_type = resolve_message_type(output_info["type"])
            res.append(TopicPersistence(
                topic=topic, topic_type=topic_type,
                environment=module_info["environment"], variable=output_name,
                is_desired=False, db=env_var_db
            ))
    return res

if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    topic_persistence_objects = create_persistence_objects(server)
    rospy.init_node('sensor_persistence')
    module_db = server[FIRMWARE_MODULE]
    last_seq = module_db.changes(limit=1, descending=True)["last_seq"]
    while True:
        if rospy.is_shutdown():
            break
        time.sleep(5)
        changes = module_db.changes(since=last_seq)
        if len(changes["results"]):
            for obj in topic_persistence_objects:
                obj.stop()
            topic_persistence_objects = create_persistence_objects(server)
