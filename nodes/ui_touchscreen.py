#!/usr/bin/env python
"""
`ui_touchscreen.py` handles communication with an raspberry pi 7" touchcreen
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.touchscreen import Touchscreen

if __name__ == '__main__':
    rospy.init_node("touchscreen")


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

    def set_cmd_temp(msg):
        touchscreen.cmd_temp = msg.data

    def set_cmd_hum(msg):
        touchscreen.cmd_hum = msg.data


    air_temp_sub = rospy.Subscriber("air_temperature/raw", Float64, callback=set_air_temp)
    humidity_sub = rospy.Subscriber("air_humidity/raw", Float64, callback=set_humidity)
    co2_sub = rospy.Subscriber("air_carbon_dioxide/raw", Float64, callback=set_co2)
    o2_sub = rospy.Subscriber("air_oxygen/raw", Float64, callback=set_o2)
    water_temp_sub = rospy.Subscriber("water_temperature/raw", Float64, callback=set_water_temp)
    ph_sub = rospy.Subscriber("water_potential_hydrogen/raw", Float64, callback=set_ph)
    ec_sub = rospy.Subscriber("water_electrical_conductivity/raw", Float64, callback=set_ec)

    temp_pub = rospy.Publisher("air_temperature/desired", Float64, queue_size=10)
    hum_pub  = rospy.Publisher("air_humidity/desired", Float64, queue_size=10)

    temp_cmd_sub = rospy.Subscriber("air_temperature/commanded", Float64, callback=set_cmd_temp)
    hum_cmd_sub  = rospy.Subscriber("air_humidity/commanded", Float64, callback=set_cmd_hum)

    # Closures are passed by reference such that any new substitutions are interpreted
    # as declarations, causing prev_time to be "Referenced before declaration".
    # This can be bypassed using an object reference
    # https://stackoverflow.com/questions/3190706/nonlocal-keyword-in-python-2-x
    def ros_next(rate_hz):
        ros_next.prev_time = rospy.get_time()
        timeout = 1 / rate_hz
        def closure():
            curr_time = rospy.get_time()
            if curr_time - ros_next.prev_time > timeout:
                ros_next.prev_time = curr_time
                return True
            else:
                return False
        return closure
    rate = rospy.get_param("~rate_hz", 1)
    is_pub = ros_next(rate)
    r = rospy.Rate(60) # Frame rate

    while not rospy.is_shutdown():
        if is_pub():
            temp_pub.publish(Float64(touchscreen.desired_temp))
            hum_pub.publish(Float64(touchscreen.desired_hum))
        touchscreen.refresh()
        r.sleep()
