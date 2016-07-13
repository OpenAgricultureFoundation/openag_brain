#!/usr/bin/env python
"""
The `pid.py` module is a Python implementation of a
Proportional-Integral-Derivative controller for ROS. By default, it listens on
a topic "desired" for the current set point and a topic "state" for the current
state of the plant being controller. It then writes to a topic "cmd" with the
output of the PID controller as well as 2 commands "up_cmd" and "down_cmd"
which are the positive and negative components of the command. This is useful
when there are separate physical actuators for applying positive and negative
control effort to the plant (e.g. a separate heater and cooler for air
temperature). If the parameter `variable` is defined, these topics will be
renamed as follows. This makes it easy to integrate this PID controller with
ROS topics from firmware modules without remapping each of the topics
individually.

    desired -> <variable>_desired
    state -> <variable>
    cmd -> <variable>_cmd
    up_cmd -> <variable>_up_cmd
    down_cmd -> <variable>_down_cmd


It also reads configuration from a number of other ROS parameters as well. The
controller gains are passed in as parameters `Kp`, `Ki`, and `Kd`. It also
accepts an `upper_limit` and `lower_limit` to bound the control effort output.
`windup_limit` defines a limit for the integrator of the control loop.
`deadband_width` can be used to apply a deadband to the control effors.
Specifically, commands with absolute value less than `deadband_width` will be
changed to 0.
"""
import rospy
from std_msgs.msg import Float64

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
        if self.set_point is None:
            return

        error = self.set_point - state

        p_value = self.Kp * error
        d_value = self.Kd * (error - self.last_error)
        self.last_error = error
        self.integrator = self.integrator + error
        self.integrator = max(-self.windup_limit, min(self.windup_limit, self.integrator))
        i_value = self.Ki * self.integrator

        res = p_value + i_value + d_value
        res = min(self.upper_limit, max(self.lower_limit, res))

        if abs(res) < self.deadband_width:
            return 0

        return res

if __name__ == '__main__':
    rospy.init_node('pid')

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
    up_pub_name = "up_cmd"
    down_pub_name = "down_cmd"
    state_sub_name = "state"
    desired_sub_name = "desired"

    variable = rospy.get_param("~variable", None)
    if variable is not None:
        pub_name = "{}_cmd".format(variable)
        up_pub_name = "{}_up_cmd".format(variable)
        down_pub_name = "{}_down_cmd".format(variable)
        state_sub_name = variable
        desired_sub_name = "{}_desired".format(variable)

    pub = rospy.Publisher(pub_name, Float64, queue_size=10)
    up_pub = rospy.Publisher(up_pub_name, Float64, queue_size=10)
    down_pub = rospy.Publisher(down_pub_name, Float64, queue_size=10)

    def state_callback(item):
        cmd = pid.update(item.data)
        if cmd is None:
            return
        up_cmd = 0
        down_cmd = 0
        if cmd > 0:
            up_cmd = cmd
        else:
            down_cmd = -cmd
        pub.publish(cmd)
        up_pub.publish(up_cmd)
        down_pub.publish(down_cmd)

    def set_point_callback(item):
        pid.set_point = item.data

    state_sub = rospy.Subscriber(state_sub_name, Float64, state_callback)
    set_point_sub = rospy.Subscriber(
        desired_sub_name, Float64, set_point_callback
    )

    rospy.spin()
