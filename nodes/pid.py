#!/usr/bin/env python
"""
The `pid.py` module is a Python implementation of a
Proportional-Integral-Derivative controller for ROS. By default, it listens on
a topic "desired" for the current set point and a topic "state" for the current
state of the plant being controller. It then writes to a topic "cmd" with the
output of the PID controller. If the parameter `variable` is defined, these
topics will be renamed as follows. This makes it easy to integrate this PID
controller with ROS topics from firmware modules without remapping each of the
topics individually.

    desired -> <variable>/desired
    state -> <variable>/measured
    cmd -> <variable>/commanded

It also reads configuration from a number of other ROS parameters as well. The
controller gains are passed in as parameters `Kp`, `Ki`, and `Kd`. It also
accepts an `upper_limit` and `lower_limit` to bound the control effort output.
`windup_limit` defines a limit for the integrator of the control loop.
`deadband_width` can be used to apply a deadband to the control effors.
Specifically, commands with absolute value less than `deadband_width` will be
changed to 0.
"""
import rospy
from std_msgs.msg import Float64, String

class PID:
    """ Discrete PID control """
    def __init__(self, Kp=0, Ki=0, Kd=0, upper_limit=1, lower_limit=-1,
            windup_limit=1000, deadband_width=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.windup_limit = windup_limit
        self.deadband_width = deadband_width

        self.set_point = None
        self.last_error = 0
        self.integrator = 0

    def update(self, state):
        # If setpoint was made null, or was already null, do nothing.
        if self.set_point is None:
            return

        error = self.set_point - state

        if abs(error) < self.deadband_width:
            return 0

        p_value = self.Kp * error
        d_value = self.Kd * (error - self.last_error)
        self.last_error = error
        self.integrator = self.integrator + error
        self.integrator = max(-self.windup_limit, min(self.windup_limit, self.integrator))
        i_value = self.Ki * self.integrator

        res = p_value + i_value + d_value
        res = min(self.upper_limit, max(self.lower_limit, res))

        return res

if __name__ == '__main__':
    rospy.init_node('pid')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )

    param_names = [
        "Kp", "Ki", "Kd", "lower_limit", "upper_limit", "windup_limit",
        "deadband_width"
    ]
    param_values = {}
    for param_name in param_names:
        private_param_name = "~" + param_name
        if rospy.has_param(private_param_name):
            param_values[param_name] = rospy.get_param(private_param_name)

    pid = PID(**param_values)

    pub_name = "cmd"
    state_sub_name = "state"
    desired_sub_name = "desired"

    variable = rospy.get_param("~variable", None)
    if variable is not None:
        pub_name = "{}/commanded".format(variable)
        state_sub_name = "{}/measured".format(variable)
        desired_sub_name = "{}/desired".format(variable)

    pub = rospy.Publisher(pub_name, Float64, queue_size=10)

    def state_callback(item):
        cmd = pid.update(item.data)
        if cmd is None:
            return
        pub.publish(cmd)

    def set_point_callback(item):
        pid.set_point = item.data

    # When we receive the recipe end message, reset this PID controller to its default values.
    # This disables the set point so the controller will just idle until it is set by a new recipe.
    def recipe_end_callback(item):
        pid = PID(**param_values)
        pid.set_point = None

    recipe_end_topic = "{ns}recipe_end/desired".format(ns=rospy.get_namespace())
    recipe_end_sub = rospy.Subscriber(recipe_end_topic, String, recipe_end_callback)
    state_sub = rospy.Subscriber(state_sub_name, Float64, state_callback)
    set_point_sub = rospy.Subscriber(
        desired_sub_name, Float64, set_point_callback
    )

    rospy.spin()
