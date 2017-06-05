#!/usr/bin/env python
"""
`sensor_ds18b20.py` handles communication with the
`DS18B20 temperature sensor <https://www.adafruit.com/product/381>`_.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.ds18b20 import DS18B20

if __name__ == '__main__':
    rospy.init_node("sensor_ds18b20")
    temp_pub = rospy.Publisher("water_temperature/raw", Float64, queue_size=10)
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)

    with DS18B20() as ds18b20:
        while not rospy.is_shutdown():
            ds18b20.poll()
            if ds18b20.temperature is not None:
                temp_pub.publish(ds18b20.temperature)
            r.sleep()
