#!/usr/bin/env python
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

    variable = rospy.get_param("~variable")
    pub_name = "{}_cmd".format(variable)
    up_pub_name = "{}_up_cmd".format(variable)
    down_pub_name = "{}_down_cmd".format(variable)

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

    state_sub = rospy.Subscriber(variable, Float64, state_callback)
    sub_name = "{}_desired".format(variable)
    set_point_sub = rospy.Subscriber(sub_name, Float64, set_point_callback)

    rospy.spin()
