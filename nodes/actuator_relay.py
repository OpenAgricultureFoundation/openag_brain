#!/usr/bin/env python
"""
`actuator_relay.py` handles communication with an active low relay
driven by GPIO pins
"""
import rospy
from std_msgs.msg import Bool
from openag_brain.peripherals.relay import Relay

if __name__ == '__main__':
    rospy.init_node("relay")
    topic = rospy.get_param("~topic", "red_light_intensity/commanded")
    pin = rospy.get_param("~pin", 27) # BCM pin
    relay = Relay(pin)

    def on_set(msg):
        relay.set(msg.data)

    subscriber = rospy.Subscriber(topic, Bool, callback=on_set)

    rospy.spin()
