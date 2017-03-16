#!/usr/bin/env python

import rospy
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType
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
    firmware_module = rospy.get_param("/firmware_module")
    firmware_module_type = rospy.get_param("/firmware_module_type")
    modules = {
        record["_id"]: FirmwareModule(record) for record in
        firmware_module if not record["_id"].startswith('_')
    }
    module_types = {
        record["_id"]: FirmwareModuleType(record) for record in
        firmware_module_type if not record["_id"].startswith('_')
    }
    modules = synthesize_firmware_module_info(modules, module_types)

    for module_id, module_info in modules.items():
        for output_name, output_info in module_info["outputs"].items():
            if not "sensors" in output_info["categories"]:
                continue
            publish_sensor_info(module_id, output_name, output_info)
    rospy.spin()
