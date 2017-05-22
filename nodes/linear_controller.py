#!/usr/bin/env python
"""
A controller that passes on the measured reading of a sensor as an
actuation command.

1 sense = 1 actuate.
"""
import rospy
from std_msgs.msg import Float64
from openag_brain.constants import NULL_SETPOINT_SENTINEL

if __name__ == '__main__':
    rospy.init_node('linear_controller')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )

    variable = rospy.get_param("~variable", None)
    if variable is not None:
        command_pub_name = "{}/commanded".format(variable)
        state_pub_name = "{}/measured".format(variable)
    else:
        command_pub_name = "cmd"
        state_pub_name = "state"


    command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)

    def state_callback(item):
        if item.data is not NULL_SETPOINT_SENTINEL:
            command_pub.publish(item)

    state_sub = rospy.Subscriber(state_pub_name, Float64, state_callback)

    rospy.spin()
