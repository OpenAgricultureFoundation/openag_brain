OpenAg Brain
============

This repository hold code that runs on the main computing board of an OpenAg
food computer. It runs on top on [ROS](www.ros.org) and uses
[CouchDB](http://couchdb.apache.org/) for data storage.

Installation
------------

First, install ROS Indigo on your machine. There are installation instructions
[here](http://wiki.ros.org/indigo/Installation/) for doing so.

Next, install an instance of CouchDB on your machine. There are installation
instructions [here](http://docs.couchdb.org/en/1.6.1/install/index.html) for
doing so. In newer version of Ubuntu (13.10 and up), this can be done via `sudo
apt-get install couchdb`.

Create a catkin workspace as described
[here](http://wiki.ros.org/indigo/catkin/Tutorials/create_a_workspace/) and
install the code from this repository in the workspace.

    cd ~/catkin_ws/src
    git clone https://github.com/OpenAgInitiative/openag_brain.git -b ros
    cd ..
    catkin_make
    catkin_make install
    rosdep install openag_brain

Next, initialize the CouchDB database as follows:

    rosrun openag_brain init_db

CouchDB will have to be restarted for all of the changes to take effect. On
Ubuntu, this can be done via `sudo service couchdb restart`.

There is a default fixture that will create a single environment as well a
couple basic modules for working with the environment. The fixture can be
installed as follows:

    rosrun openag_brain load_fixture default

Running
-------

The following command will generate a roslaunch file describing all of the
modules defined in the database:

    rosrun openag_brain update_launch

That command creates a file named `modules.launch`. The modules can be run by
calling:

    roslaunch openag_brain modules.launch

There is also a Flask API that allows external programs (e.g.
[openag_ui](http://github.com/OpenAgInitiative/openag_ui)) to interact with the
local ROS installation. The API can be launched as follows:

    rosrun openag_brain api
