#!/usr/bin/env python
import rospy
import time, sched
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('ebb_flow_controller')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
#    if namespace == '/':
#        raise RuntimeError(
#            "Cannot be run in the global namespace. Please "
#            "designate an environment for this module."
#        )

    command_pub_name = "cmd"
    state_pub_name = "state"

    on_time = rospy.get_param("~on_time", None)
    off_time = rospy.get_param("~off_time", None)

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

    def publish_both(item):
        print("publishing...{}".format(command_pub_name))
        command_pub.publish(item)
        state_pub.publish(item)
        

    def loop(is_off):
        freq = 1
        out = 0.0 if is_off else 1.0
        time_delay = off_time if is_off else on_time
        for i in range(int(time_delay / freq)):
            s.enter(freq+i*freq, 1, publish_both, argument=[out])

        s.enter(time_delay, 1, loop, argument=[not is_off])
        s.run()

    loop(True)

    # Keeps this node running, although it might not be necessary given that the code runs indefinitely
    rospy.spin()
