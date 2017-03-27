#!/usr/bin/env python
"""
`sensor_am2315.py` handles communication with the
`AM2315 temperature and humidity sensor <https://www.adafruit.com/products/1293>`_.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.am2315 import AM2315

if __name__ == '__main__':
    i2c_addr = rospy.get_param("~i2c_addr", 0x5c)
    i2c_bus = rospy.get_param("~i2c_bus", 1)
    pseudo = rospy.get_param("~pseudo", True)
    am2315 = AM2315(i2c_addr=i2c_addr, i2c_bus=i2c_bus, pseudo=pseudo)
    rospy.spin()
