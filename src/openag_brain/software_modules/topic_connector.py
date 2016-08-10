#!/usr/bin/python
"""
By convention, firmware modules publish sensor data to ROS topics in the
namespace `/sensors` and listen to actuator commands on ROS topics in the
namespace `/actuators`. This is very useful for low level tasks such as
debugging/testing your hardware but not so useful for getting a high level
overview of the environmental conditions of your system. For this, we would
like to use topics namespaced by the ID of the environment on which the piece
of hardware acts (e.g. /environment_1/air_temperature). This module connects
topics so as to ensure that both of these system views work as expected. There
should be exactly one instance of this module in the system
"""
import sys
import time
import rospy
import rosgraph
import rostopic
from couchdb import Server

from openag.cli.config import config as cli_config
from openag.models import FirmwareModule, FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE

from openag_brain import params
from openag_brain.srv import Empty
from openag_brain.util import resolve_message_type, get_database_changes

Float32 = resolve_message_type("std_msgs/Float32")
Float64 = resolve_message_type("std_msgs/Float64")

def connect_topics(src_topic, dest_topic, src_topic_type, dest_topic_type):
    pub = rospy.Publisher(dest_topic, dest_topic_type, queue_size=10)
    def callback(src_item, publisher=pub, src_type=src_topic_type,
            dest_type=dest_topic_type):
        dest_item = dest_type(*[
            getattr(src_item, slot) for slot in src_item.__slots__
        ])
        publisher.publish(dest_item)
    sub = rospy.Subscriber(src_topic, src_topic_type, callback)
    return sub, pub

def connect_all_topics(module_db, module_type_db):
    modules = {
        module_id: FirmwareModule(module_db[module_id]) for module_id in
        module_db if not module_id.startswith('_')
    }
    topics = []
    for module_id, module_info in modules.items():
        module_type = module_type_db[module_info["type"]]
        for input_name, input_info in module_type["inputs"].items():
            mapped_input_name = module.get("mappings",{}).get(
                input_name, input_name
            )
            src_topic = "/{}/{}".format(module.environment, mapped_input_name)
            dest_topic = "/actuators/{}_{}".format(module.id, input_name)
            dest_topic_type = resolve_message_type(input_info["type"])
            src_topic_type = Float64 if dest_topic_type is Float32 else \
                    dest_topic_type
            topics.extend(connect_topics(
                src_topic, dest_topic, src_topic_type, dest_topic_type
            ))
        for output_name, output_info in module_type.outputs.items():
            mapped_output_name = module_type.get("mappings", {}).get(
                output_name, output_name
            )
            src_topic = "/sensors/{}_{}".format(module.id, output_name)
            dest_topic = "/{}/{}".format(module.environment, mapped_output_name)
            src_topic_type = resolve_message_type(output_info["type"])
            dest_topic_type = Float64 if src_topic_type is Float32 else \
                    src_topic_type
            topics.extend(connect_topics(
                src_topic, dest_topic, src_topic_type, dest_topic_type
            ))
    return topics

if __name__ == '__main__':
    rospy.init_node("topic_connector")
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local server specified")
    server = Server(db_server)
    module_db = server[FIRMWARE_MODULE]
    module_type_db = server[FIRMWARE_MODULE_TYPE]
    topics = connect_all_topics(module_db, module_type_db)
    last_seq = get_database_changes(db_server, FIRMWARE_MODULE)['last_seq']
    while True:
        if rospy.is_shutdown():
            break
        time.sleep(5)
        changes = get_database_changes(db_server, FIRMWARE_MODULE, last_seq)
        last_seq = changes['last_seq']
        if len(changes['results']):
            for topic in topics:
                topic.unregister()
            topics = connect_all_topics(module_db, module_type_db)
