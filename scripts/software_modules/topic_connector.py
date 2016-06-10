#!/usr/bin/python
import rospy
import rosgraph
import rostopic
from couchdb import Server

from openag_brain.models import FirmwareModuleModel
from openag_brain.db_names import DbName

def map_topics(src_topic, dest_topic):
    master = rosgraph.Master('/rostopic')
    state = master.getSystemState()
    topic_types = rosgraph._master_get_topic_types(master)
    topic_type = rostopic.topic_type(src_topic, topic_types)

    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    def callback(item, publisher=pub):
        publisher.publish(item)
    rospy.Subscriber(src_topic, topic_type, callback)

rospy.init_node("firmware_translator")
server = Server()
module_db = server[DbName.FIRMWARE_MODULE]
module_type_db = server[DbName.FIRMWARE_MODULE_TYPE]
for module_id in module_db:
    module = FirmwareModuleModel.load(module_db, module_id)
    module_type = FirmwareModuleTypeModel.load(module_type_db, module.type)
    for input_name in module_type.inputs:
        src_topic = "/{}/{}".format(module.environment, input_name)
        dest_topic = "/actuators/{}:{}".format(input_name, module_id)
        map_topics(src_topic, dest_topic)
    for output_name in module_type.outputs:
        src_topic = "/sensors/{}:{}".format(output_name, module_id)
        dest_topic = "/{}/{}".format(module.environment, output_name)
        map_topics(src_topic, dest_topic)

rospy.spin()
