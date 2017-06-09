"""
This module consists of code for interacting with the USB HID 16-Channel
relay module.
"""

import rospy
import time

class HidRelay16:
    """
    Class that represents a USB Controlled HID 16-Channel relay module instance
    and provides functions for interfacing with the sensor.
    """

    def __init__(self):
        rospy.loginfo("Initialized HidRelay16")
        # self.connect()

    def set(self, value, relay_id):
        rospy.loginfo('Setting relay {} to {}'.format(relay_id, value))
