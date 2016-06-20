OpenAg Brain
============

This repository hold code that runs on the main computing board of an OpenAg
food computer (usually a Raspberry Pi). It runs on top on [ROS](www.ros.org)
and uses [CouchDB](http://couchdb.apache.org/) for data storage.

Installation (Raspberry Pi)
---------------------------

There is a pre-built Docker image for running this codebase on ARM machines.
Most people should use this image. To set it up, follow the instructions in
[this repo](https://github.com/OpenAgInitiative/openag_brain_docker_rpi).

If you plan to modify the code in this repository, it might make more sense to
install the software directly on your machine instead of running everything in
a Docker container. For Raspberry Pi, this can be done by following hte
instructions in [this
repo](https://github.com/OpenAgInitiative/openag_brain_install_rpi.git).

Installation (Other Machines)
-----------------------------

If neither the Docker image nor the install script work for you, for whatever
reason, you will have to install each component of the system yourself. These
instructions describe how to do so.

First, install ROS Indigo on your machine. There are installation instructions
[here](http://wiki.ros.org/indigo/Installation/) for doing so. For Raspberry
Pi, there are instructions
[here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)
instead.

Next, install an instance of CouchDB on your machine. There are installation
instructions [here](http://docs.couchdb.org/en/1.6.1/install/index.html) for
doing so. In newer version of Ubuntu (13.10 and up), this can be done via `sudo
apt-get install couchdb`.

Create a catkin workspace as described
[here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace/) and
install the code from this repository in the workspace.

    cd ~/catkin_ws/src
    git clone https://github.com/OpenAgInitiative/openag_brain.git -b ros
    cd ..
    catkin_make
    catkin_make install
    rosdep install -i openag_brain

To run any of the openag commands, you must first activate the catkin workspace
as follows:

    source ~/catkin_ws/devel/setup.bash

Finally, you must install `platformio`. `platformio` and `rosserial` use
different versions of `pyseial`, so installing them both on the same machine
will break things. Instead, `platformio` should be installed in a python
virtual environment. There is a script in `openag_brain` that will do this for
you:

    rosrun openag_brain install_pio

Running
-------

With everything installed (not through Docker) and with the catkin workspace
active, the project can be run as follows:

    rosrun openag_brain main
