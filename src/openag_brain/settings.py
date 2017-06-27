import os
import rospy

# Turn on logic tracing by creating a '~/TRACE' file.
# Output is written to the node's log file. e.g:
# tail -f ~/.ros/log/latest/environments-environment_1-recipe_handler_1-6.log
TRACE = os.path.isfile(os.path.expanduser('~/TRACE'))
def trace(msg, *args):
    if TRACE:
        msg = '\nTRACE> ' + msg
        rospy.logdebug(msg, *args)

