#!/usr/bin/env python

import rospy
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE
from couchdb import Server
from openag_brain.msg import SensorInfo

def publish_sensor_info(mod_id, variable, info):
    # Build the topic name
    topic = "/sensors/{}/{}/info".format(mod_id, variable)

    # Build the message
    msg = SensorInfo()
    msg.accuracy = info.get("accuracy", 0)
    msg.repeatability = info.get("repeatability", 0)

    # Publish the message
    pub = rospy.Publisher(topic, SensorInfo, queue_size=1, latch=True)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("sensor_info_publisher")

    # Connect to the DB server
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local server specified")
    server = Server(db_server)

    # Get info on all of the modules
    module_db = server[FIRMWARE_MODULE]
    module_type_db = server[FIRMWARE_MODULE_TYPE]
    modules = {
        module_id: FirmwareModule(module_db[module_id]) for module_id in
        module_db if not module_id.startswith('_')
    }
    module_types = {
        type_id: FirmwareModuleType(module_type_db[type_id]) for type_id in
        module_type_db if not type_id.startswith('_')
    }
    modules = synthesize_firmware_module_info(modules, module_types)

    for module_id, module_info in modules.items():
        for output_name, output_info in module_info["outputs"].items():
            if not "sensors" in output_info["categories"]:
                continue
            publish_sensor_info(module_id, output_name, output_info)
    rospy.spin()
