#!/usr/bin/env python
"""
`ui_touchscreen.py` handles communication with an raspberry pi 7" touchcreen
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.touchscreen import Touchscreen

if __name__ == '__main__':
    rospy.init_node("touchscreen")
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)
    touchscreen = Touchscreen()

    def set_air_temp(msg):
        touchscreen.air_temp = msg.data

    def set_humidity(msg):
        touchscreen.humidity= msg.data

    def set_co2(msg):
        touchscreen.co2 = msg.data

    def set_o2(msg):
        touchscreen.o2 = msg.data

    def set_water_temp(msg):
        touchscreen.water_temp = msg.data

    def set_ph(msg):
        touchscreen.ph = msg.data

    def set_ec(msg):
        touchscreen.ec = msg.data

    air_temp_sub = rospy.Subscriber("air_temperature/raw", Float64, callback=set_air_temp)
    humidity_sub = rospy.Subscriber("air_humidity/raw", Float64, callback=set_humidity)
    co2_sub = rospy.Subscriber("air_carbon_dioxide/raw", Float64, callback=set_co2)
    o2_sub = rospy.Subscriber("air_oxygen/raw", Float64, callback=set_o2)
    water_temp_sub = rospy.Subscriber("water_temperature/raw", Float64, callback=set_water_temp)
    ph_sub = rospy.Subscriber("water_potential_hydrogen/raw", Float64, callback=set_ph)
    ec_sub = rospy.Subscriber("water_electrical_conductivity/raw", Float64, callback=set_ec)

    while not rospy.is_shutdown():
        touchscreen.refresh()
        r.sleep()
