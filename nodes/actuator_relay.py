#!/usr/bin/env python
"""
`actuator_relay.py` handles communication with an active low relay
driven by GPIO pins
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.relay import Relay

if __name__ == '__main__':
    rospy.init_node("relay")
    topic = rospy.get_param("~topic", "red_light_intensity/commanded")
    pin = rospy.get_param("~pin", 27) # BCM pin
    relay = Relay(pin)

    def on_set(msg):
        cmd = msg.data > 0.0
        relay.set(cmd)

    subscriber = rospy.Subscriber(topic, Float64, callback=on_set)

    rospy.spin()
