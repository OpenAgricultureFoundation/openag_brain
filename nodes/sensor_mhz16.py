#!/usr/bin/env python
"""
`sensor_mhz16.py` handles communication with the
`MHZ16 co2 sensor <http://sandboxelectronics.com/?product=mh-z16-ndir-co2-sensor-with-i2cuart-5v3-3v-interface-for-arduinoraspeberry-pi>`_.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.mhz16 import MHZ16

if __name__ == '__main__':
    rospy.init_node("sensor_mhz16")
    i2c_addr = 0x4d
    i2c_bus = rospy.get_param("~i2c_bus", "/dev/i2c-1")
    co2_pub = rospy.Publisher("air_carbon_dioxide/raw", Float64, queue_size=10)
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)

    with MHZ16(i2c_addr, i2c_bus) as mhz16:
        while not rospy.is_shutdown():
            mhz16.poll()
            if mhz16.co2:
                co2_pub.publish(mhz16.co2)

            # Use rate timer instance to sleep until next turn
            r.sleep()
