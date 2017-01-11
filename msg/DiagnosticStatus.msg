# The message holds the status of an individual component of the robot.
# Based on diagnostic_msgs/DiagnosticStatus

# Possible levels of operation
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerate above
string name # a description of the test/component reporting
uint8 code # a status code for the test/component
