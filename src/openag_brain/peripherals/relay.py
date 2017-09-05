"""
This module consists of code for interacting with an active low relay.
"""

import rospy
from periphery import GPIO

class Relay():
    def __init__(self, pin):
        rospy.loginfo("Initializing relay")
        self.pin = pin
        self.gpio_out = GPIO(self.pin, "out")
        self.gpio_out.write(True) # initialize relay to be OFF

    def __exit__(self):
        self.gpio_out.close()

    def set(self, state):
        try:
            if state:
                rospy.loginfo("Turning ON relay")
                self.gpio_out.write(False) # active low
            else:
                rospy.loginfo("Turning OFF relay")
                self.gpio_out.write(True)
        except Exception as e:
            rospy.logwarn("Unable to set relay. Error: {}".format(e))
