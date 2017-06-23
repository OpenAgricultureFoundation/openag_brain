#!/usr/bin/env python
"""
`sensor_atlas_ph.py` handles communication with the Atlas pH sensor
`<https://www.atlas-scientific.com/product_pages/kits/ph-kit.html> connected
to the Atlas Isolated USB EZO Board <https://www.atlas-scientific.com/product_pages/components/usb-iso.html>`_.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.atlas_ph import AtlasPh

if __name__ == '__main__':
    rospy.init_node("sensor_atlas_ph")
    ph_pub = rospy.Publisher("water_potential_hydrogen/raw", Float64, queue_size=10)
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)

    while not rospy.get_param("atlas/ready", False):
        pass

    rospy.set_param("atlas/ready", False)
    with AtlasPh() as atlas_ph:

        while not rospy.is_shutdown():
            atlas_ph.poll()
            if atlas_ph.ph is not None:
                rospy.set_param("atlas/ready", True)
                ph_pub.publish(atlas_ph.ph)

            r.sleep()
