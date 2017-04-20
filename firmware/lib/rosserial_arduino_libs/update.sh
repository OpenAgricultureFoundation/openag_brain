set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROSVERSION="indigo"
ROSSERIAL_ARDUINO_VERSION='0.6.4'

if [ `rosversion -d` != $ROSVERSION ]; then
    >&2 echo "This library is typically built on ROS $ROSVERSION"
    >&2 echo "You are running ROS `rosversion -d`"
fi
if [ `rosversion rosserial_arduino` != "$ROSSERIAL_ARDUINO_VERSION" ]; then
    >&2 echo "This library is typically built using rosserial_arduino $ROSSERIAL_ARDUINO_VERSION"
    >&2 echo "You have rosserial_arduino `rosversion rosserial_arduino`"
fi

if [ -d "$DIR/ros_lib" ]; then
    rm -rf $DIR/ros_lib
fi
rosrun rosserial_arduino make_libraries.py $DIR
cp -rfl $DIR/ros_lib/* $DIR
rm -rf $DIR/ros_lib
