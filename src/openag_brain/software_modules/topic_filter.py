#!/usr/bin/python
import time
import rospy
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE
from couchdb import Server

from openag_brain.util import resolve_message_type

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

def filter_all_topics(module_db, module_type_db):
    modules = {
        module_id: FirmwareModule(module_db[module_id]) for module_id in
        module_db if not module_id.startswith('_')
    }
    module_types = {
        type_id: FirmwareModuleType(module_type_db[type_id]) for type_id in
        module_type_db if not type_id.startswith("_")
    }
    modules = synthesize_firmware_module_info(modules, module_types)
    for module_id, module_info in modules.items():
        for output_name, output_info in module_info["outputs"].items():
            src_topic = "/sensors/{}/{}/raw".format(module_id, output_name)
            dest_topic = "/sensors/{}/{}/filtered".format(
                module_id, output_name
            )
            topic_type = resolve_message_type(output_info["type"])
            filter_topic(src_topic, dest_topic, topic_type)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local server specified")
    server = Server(db_server)
    module_db = server[FIRMWARE_MODULE]
    module_type_db = server[FIRMWARE_MODULE_TYPE]
    filter_all_topics(module_db, module_type_db)
    rospy.spin()
