#!/usr/bin/python

import rospy
from std_msgs.msg import Float64, Bool

class OnOffControl(object):
    def __init__(self, default):
        self.current_set_point = default
        self.current_measured = self.current_set_point
        rospy.Subscriber('set_point', Float, self.on_set_point)
        rospy.Subscriber('measured', Float, self.on_measured)
        self.state = rospy.Publisher('state', Bool, queue_size=10)

    def on_set_point(self, data):
        self.current_set_point = data.data
        self.update_state()

    def on_measured(self, data):
        self.current_measured = data.data
        self.update_state()

    def update_state(self):
        if self.current_measured < self.current_set_point:
            self.state.publish(True)
        else:
            self.state.publish(False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        "Performs on/off control given a set point and a stream of "
        "measurements by outputing a stream of commands to an actuator that "
        "is assumed to raise the value of the parameter being measured"
    )
    paresr.add_argument("default")
    args, _ = parser.parse_known_args()
    c = OnOffControl(args.default)
    rospy.spin()
