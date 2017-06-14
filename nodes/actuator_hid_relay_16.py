#!/usr/bin/env python
"""
`actuator_hid_relay_16.py` handles communication with the
`Sainsmart USB HID programmable control relay module
<https://www.sainsmart.com/sainsmart-16-channel-controller-usb-hid-programmable-control-relay-module.html>
that connects the the Sainsmart 16-channel relay module
<https://www.amazon.com/SainSmart-101-70-103-16-Channel-Relay-Module/dp/B0057OC66U/ref=sr_1_1?ie=UTF8&qid=1497021207&sr=8-1&keywords=sainsmart+relay+16>`_.
"""
import rospy
from std_msgs.msg import Bool
from openag_brain.peripherals.hid_relay_16 import HidRelay16

if __name__ == '__main__':
    rospy.init_node("actuator_hid_relay_16")
    default_relay_map = {"red_light_intensity/commanded": "2", # red light is connected to relay 1
                         "white_light_intensity/commanded": "3",
                         "blue_light_intensity/commanded": "4"}

    relay_map = rospy.get_param("~relay_map", default_relay_map)

    hid_relay_16 = HidRelay16()

    def on_set(msg, relay_id):
        hid_relay_16.set(int(relay_id), msg.data)

    for topic in relay_map:
        subscriber = rospy.Subscriber(topic, Bool, callback=on_set, callback_args=relay_map[topic])

    rospy.spin()
