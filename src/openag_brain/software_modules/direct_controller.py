#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('direct_controller')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )

    command_pub_name = "cmd"
    state_pub_name = "state"
    desired_sub_name = "desired"

    variable = rospy.get_param("~variable", None)
    if variable is not None:
        command_pub_name = "{}/commanded".format(variable)
        state_pub_name = "{}/measured".format(variable)
        desired_sub_name = "{}/desired".format(variable)

    command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)
    state_pub = rospy.Publisher(state_pub_name, Float64, queue_size=10)

    def desired_callback(item):
        command_pub.publish(item)
        state_pub.publish(item)

    desired_sub = rospy.Subscriber(desired_sub_name, Float64, desired_callback)

    rospy.spin()
