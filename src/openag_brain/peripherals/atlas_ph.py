"""
This module consists of code for interacting with the Atlas pH sensor.
"""

"""
IMPORTANT: Need to configure FTDI on raspi for this module to work properly.
See: https://github.com/OpenAgInitiative/openag_atlas_ph_python/blob/master/README.md
"""

import rospy
import time
from atlas_device import AtlasDevice

class AtlasPh:
    """
    Class that represents an Atlas pH sensor instance and provides functions
    for interfacing with the sensor.
    """

    def __init__(self, device_id = "DO009MQN"):
        self.device_id = device_id
        self.ph = None
        self.connect()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        return self

    def connect(self):
        try:
            self.device = AtlasDevice(self.device_id)
            self.device.send_cmd("C,0") # turn off continuous mode
            time.sleep(1)
            self.device.flush()
            rospy.loginfo("Connected to AtlasPh")
        except:
            rospy.logwarn("Failed to connect to AtlasPh")
            pass

    def poll(self):
        try:
            self.device.send_cmd("R")
            lines = self.device.read_lines()
            for i in range(len(lines)):
                if lines[i] != u'*OK\r':
                    self.ph = float(lines[i])
                    self.ph = float("{0:.2f}".format(self.ph))
        except:
            self.ph = None
