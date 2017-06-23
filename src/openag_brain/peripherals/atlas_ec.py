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
from pylibftdi.examples import list_devices

class AtlasEc:
    """
    Class that represents an Atlas EC sensor instance and provides functions
    for interfacing with the sensor.
    """

    def __init__(self, device_id = None):
        self.device_id = device_id
        self.ec = None
        self.connect()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        return self

    def connect(self):
        try:
            if self.device_id is None:
                # Try to find pH device id automatically
                devices = list_devices.get_ftdi_device_list()

                # Check if any atlas devices are connected to device
                if len(devices) == 0:
                  raise IOError("No atlas device found on system")
                
                for device in devices:
                    # Extract id
                    device_id = (device.split("FTDI:FT230X Basic UART:"))[1]

                    # Check if device is ph sensor
                    with AtlasDevice(device_id) as atlas_device:
                        atlas_device.send_cmd("i") # get device information
                        time.sleep(1)
                        lines = atlas_device.read_lines()
                        if "EC" in lines[0]:
                            self.device_id = device_id
                            rospy.loginfo("Automatically found device id: {}".format(device_id))

            self.device = AtlasDevice(self.device_id)
            self.device.send_cmd("C,0") # turn off continuous mode
            time.sleep(1)
            self.device.flush()
            rospy.loginfo("Connected to AtlasEc")

        except Exception as e:
            rospy.logwarn("Failed to connect to AtlasEc. Error: {}".format(e))
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
