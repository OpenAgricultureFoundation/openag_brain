#!/usr/bin/env python
"""
`sensor_atlas_ec.py` handles communication with the Atlas EC sensor
`<https://www.atlas-scientific.com/product_pages/kits/ec_k1_0_kit.html> connected
to the Atlas Isolated USB EZO Board <https://www.atlas-scientific.com/product_pages/components/usb-iso.html>`_.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.peripherals.atlas_ec import AtlasEc

if __name__ == '__main__':
    rospy.init_node("sensor_atlas_ec")
    ec_pub = rospy.Publisher("water_electrical_conductivity/raw", Float64, queue_size=10)
    rate = rospy.get_param("~rate_hz", 1)
    r = rospy.Rate(rate)

    while not rospy.get_param("atlas/ready", False):
        pass

    rospy.set_param("atlas/ready", False)
    with AtlasEc() as atlas_ec:
        while not rospy.is_shutdown():
            atlas_ec.poll()
            if atlas_ec.ec is not None:
                rospy.set_param("atlas/ready", True)
                ec_pub.publish(atlas_ec.ec)

            r.sleep()
