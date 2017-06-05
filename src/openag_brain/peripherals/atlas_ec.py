"""
This module consists of code for interacting with the Atlas electrical
conductivity (EC) sensor.
"""

"""
IMPORTANT: Need to configure FTDI on raspi for this module to work properly.
See: https://github.com/OpenAgInitiative/openag_atlas_ph_python/blob/master/README.md
"""

import rospy
import time
from atlas_device import AtlasDevice

class AtlasEc:
    """
    Class that represents an Atlas EC sensor instance and provides functions
    for interfacing with the sensor.
    """

    def __init__(self, device_id = "DO009N86"):
        self.device_id = device_id
        self.ec = None
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
            rospy.loginfo("Connected to AtlasEc")
        except:
            rospy.logwarn("Failed to connect to AtlasEc")
            pass

    def poll(self):
        try:
            self.device.send_cmd("R")
            lines = self.device.read_lines()
            for i in range(len(lines)):
                if lines[i] != u'*OK\r':
                    floats = [float(x) for x in lines[i].split(',')]
                    self.ec = floats[0] / 1000 # ms/cm
        except:
            self.ec = None
