"""
This module consists of code for interacting with the DS18B20 temperature sensor.
"""

import rospy
import time
from w1thermsensor import W1ThermSensor

class DS18B20:
    """
    Class that represents a DS18B20 temperature sensor instance
    and provides functions for interfacing with the sensor.
    """

    def __init__(self):
        self.temperature = None
        self.connect()

    def __del__(self):
        return self

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        return self

    def connect(self):
        try:
            self.sensor = W1ThermSensor()
            rospy.loginfo("Connected to DS18B20")
        except I2CError:
            rospy.logwarn("Failed to connect to DS18B20")
            pass

    def poll(self):
        try:
            self.temperature = self.sensor.get_temperature()
        except:
            self.temperature = None
