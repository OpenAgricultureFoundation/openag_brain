#!/usr/bin/env python
""" Direct Controller
This ROS node controls the variables which do not have any feedback and the
output setting is just set once and maintained.

The command and measured topics are simply echo's of the desired topic.

This is called from the launch file, for example:

<node pkg="openag_brain" type="direct_controller.py" name="light_controller_red_1">
  <param name="variable" value="light_intensity_red" type="str"/>
</node>


"""
import rospy
from std_msgs.msg import Float64
from openag_brain.constants import NULL_SETPOINT_SENTINEL

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
        state_pub_name = "{}/raw".format(variable)
        desired_sub_name = "{}/desired".format(variable)

    command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)
    state_pub = rospy.Publisher(state_pub_name, Float64, queue_size=10)

    def desired_callback(item):
        if item.data is not NULL_SETPOINT_SENTINEL:
            command_pub.publish(item)
            state_pub.publish(item)

    desired_sub = rospy.Subscriber(desired_sub_name, Float64, desired_callback)

    rospy.spin()
