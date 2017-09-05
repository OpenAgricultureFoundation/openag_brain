#!/usr/bin/python
"""
This module consists of code for interacting with a Grove O2 sensor.
"""

import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.grove_o2 import GroveO2

if __name__ == "__main__":
    rospy.init_node("sensor_grove_o2")
    o2_pub = rospy.Publisher("air_oxygen/raw", Float64, queue_size=10)
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)

    with GroveO2() as grove_o2:
        while not rospy.is_shutdown():
            grove_o2.poll()
            if grove_o2.o2 is not None:
                o2_pub.publish(grove_o2.o2)
            r.sleep()
