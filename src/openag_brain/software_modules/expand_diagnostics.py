#!/usr/bin/env python
"""
The arduino publishes openag_brain/DiagnosticArray messages to the
/internal_diagnostics topic in ROS through rosserial. These messages are
compacted to minimize the amount of data sent across the USB connection for
scalablility. In particular, these messages convey status information through
8-bit status codes instead of status messages. The firmware_module_type records
in the database contain dictionaries that define module-specific mappings from
these status codes to status messages. This module reads in the
openag_brain/DiagnosticArray messages, uses the information from the database
to "expand" the status codes to status messages, and republishes the status
information as diagnostic_msgs/DiagnosticArray messages to use with the ROS
diagnostic stack to ease debugging of hardware failures and displaying of
hardware error messages.
"""

import rospy
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE
from openag_brain.msg import DiagnosticArray as _DiagnosticArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from couchdb import Server

class DiagnosticsExpander:
    def __init__(self, modules):
        self.status_codes_by_module = {
            mod_id: {
                int(code) : msg 
                for code, msg in mod_info.get("status_codes", {}).items()
            } for mod_id, mod_info in modules.items()
        }
        self.sub = rospy.Subscriber(
            "/internal_diagnostics", _DiagnosticArray, self.callback
        )
        self.pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

    def callback(self, _msg):
        msg = DiagnosticArray()
        statuses = []
        for _status in _msg.status:
            status = DiagnosticStatus()
            status.level = _status.level
            status.name = _status.name
            if _status.code == 0:
                status.message = ""
            else:
                status.message = self.status_codes_by_module[_status.name].get(
                    _status.code, "Unknown error"
                )
            statuses.append(status)
        msg.status = statuses
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("expand_diagnostics")
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local server specified")
    server = Server(db_server)
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
    expander = DiagnosticsExpander(modules)
    rospy.spin()
