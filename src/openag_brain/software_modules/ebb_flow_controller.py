#!/usr/bin/env python
import rospy
import time, sched
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('ebb_flow_controller')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )

    command_pub_name = "cmd"
    state_pub_name = "state"

    on_time = rospy.get_param("~on_time")
    off_time = rospy.get_param("~off_time")

    if on_time and off_time is None:
        raise RuntimeError(
            "There is no on_time or off_time set for this module."
        )

    variable = rospy.get_param("~variable", None)
    if variable is not None:
        command_pub_name = "{}/commanded".format(variable)
        state_pub_name = "{}/measured".format(variable)

    command_pub = rospy.Publisher(command_pub_name, Float64, queue_size=10)
    state_pub = rospy.Publisher(state_pub_name, Float64, queue_size=10)

    s = sched.scheduler(time.time, time.sleep)

    def publish_both(item, is_off=True):
        command_pub.publish(item)
        state_pub.publish(item)
        time_delay = on_time if is_off else off_time
        s.enter(time_delay, 1, publish_both, argument=(1.0 - item, not is_off))
        s.run()

    # Keeps this node running, although it might not be necessary given that the code runs indefinitely
    rospy.spin()
