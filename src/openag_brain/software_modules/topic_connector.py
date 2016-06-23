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

from openag_brain import params
from openag_brain.srv import Empty
from openag_brain.util import resolve_message_type, get_database_changes
from openag_brain.models import FirmwareModuleModel, FirmwareModuleTypeModel
from openag_brain.db_names import DbName

def connect_topics(src_topic, dest_topic):
    master = rosgraph.Master('/rostopic')
    state = master.getSystemState()
    while True:
        try:
            topic_types = rostopic._master_get_topic_types(master)
            topic_type = next(
                topic_type for topic_name, topic_type in topic_types if topic_name ==
                src_topic
            )
            break
        except StopIteration:
            rospy.logwarn(
                "{} does not seem to be published yet. "
                "Waiting...".format(src_topic)
            )
            rospy.sleep(2)
            if rospy.is_shutdown():
                sys.exit()
    topic_type = resolve_message_type(topic_type)
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    def callback(item, publisher=pub):
        publisher.publish(item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub

def connect_all_topics(module_db, module_type_db):
    topics = []
    for module_id in module_db:
        if module_id.startswith('_'):
            continue
        module = FirmwareModuleModel.load(module_db, module_id)
        module_type = FirmwareModuleTypeModel.load(module_type_db, module.type)
        for input_name in module_type.inputs.keys():
            src_topic = "/{}/{}".format(module.environment, input_name)
            dest_topic = "/actuators/{}_{}".format(module_id, input_name)
            topics.extend(connect_topics(src_topic, dest_topic))
        for output_name in module_type.outputs:
            src_topic = "/sensors/{}_{}".format(module_id, output_name)
            dest_topic = "/{}/{}".format(module.environment, output_name)
            topics.extend(connect_topics(src_topic, dest_topic))
    return topics

if __name__ == '__main__':
    rospy.init_node("topic_connector")
    db_server = rospy.get_param(params.DB_SERVER)
    server = Server(db_server)
    module_db = server[DbName.FIRMWARE_MODULE]
    module_type_db = server[DbName.FIRMWARE_MODULE_TYPE]
    topics = connect_all_topics(module_db, module_type_db)
    last_seq = get_database_changes(
        db_server, DbName.FIRMWARE_MODULE
    )['last_seq']
    while True:
        time.sleep(5)
        changes = get_database_changes(
            db_server, DbName.FIRMWARE_MODULE, last_seq
        )
        last_seq = changes['last_seq']
        if len(changes['results']):
            for topic in topics:
                topic.unregister()
            topics = connect_all_topics(module_db, module_type_db)

