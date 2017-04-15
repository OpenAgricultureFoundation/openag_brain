Getting Started
===============

The project is meant to run on a Raspberry Pi, but should be able to run on any
operating system that supports both `ROS <http://wiki.ros.org>`_ and `CouchDB
<http://couchdb.apache.org>`_.

Installation
------------

First, install a compatible operating system on your machine. For Raspberry Pi,
we recommend Raspbian Jessie. The image can be downloaded `here
<https://www.raspberrypi.org/downloads/raspbian>`_ and flashed to the SD card
using the installation instructions `here
<https://www.raspberrypi.org/documentation/installation/installing-images/README.md>`_.

Raspberry Pi Docker Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is a pre-built Docker image for running this codebase on a Raspberry Pi.
Most people should use this image. The image contains only ROS and the
openag_brain code, so it must be run alongside a CouchDB installation. There
exists a docker-compose file in `this repo
<https://github.com/OpenAgInitiative/openag_brain_docker_rpi>`_ for
automatically configuring the 2 Docker containers. The setup process for it is
fairly straightforward. First, you clone the repository and install Docker
using a script from the repository::

    git clone https://github.com/OpenAgInitiative/openag_brain_docker_rpi
    cd openag_brain_docker_rpi
    sh install_docker.sh

After this, you will have to start a new terminal session in order for the
installation to complete. Then, the project can be started by running the
following::

    docker-compose up -d

Raspberry Pi Installation From Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you plan to work on the code for the project, it might make more sense to
install the software directly on your machine instead of running everything in
Docker containers. openag_brain ships with an install script.

To run it, first, clone openag_brain into ``~/catkin_ws/src``
(``~/catkin_ws`` is the ROS build space)::

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/OpenAgInitiative/openag_brain

Now that openag_brain is in the catkin_build space, you can run the
install script::

    cd ~/catkin_ws/src/openag_brain
    ./scripts/install_dev

This will set up a full development environment for openag_brain and ROS.

Generic Installation From Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If neither the Docker image nor the install script work for you, for whatever
reason, you will have to install each component of the system yourself.

First, install ROS Indigo on your machine. There are installation instructions
`here <http://wiki.ros.org/indigo/Installation/>`_ for doing so. For Raspberry
Pi, there are instructions `here
<http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi>`_
instead.

Next, install an instance of CouchDB on your machine. there are installation
instructions `here <http://docs.couchdb.org/en/1.6.1/install/index.html>`_ for
doing so. In newer versions of Ubuntu (13.10 and up), this can be done via::

    sudo apt-get install couchdb

Create a catkin workspace as described `here
<http://wiki.ros.org/catkin/Tutorials/create_a_workspace/>`_. Then, install the
code from the ``openag_brain`` repository in this workspace as follows::

    cd ~/catkin_ws/src
    git clone http://github.com/OpenAgInitiative/openag_brain.git
    cd ..
    catkin_make
    catkin_make install
    rosdep install -i openag_brain

To run any of the openag commands, you must first activate the catkin workspace
as follows::

    source ~/catkin_ws/devel/setup.bash

Finally, you must install ``PlatformIO``. ``PlatformIO`` and ``rosserial`` use
different versions of ``pyserial``, so installing them both on the same machine
will break things. Instead, ``platformio`` should be installed in a python
virtual environment. There is a script in ``openag_brain`` that will do this
for you::

    rosrun openag_brain install_pio

Running
-------

If you are running the Docker image, the project should run automatically and
persist through reboots. Otherwise, with the catkin workspace active, the
project can be run as follows::

    rosrun openag_brain main -f default

The :code:`-f default` flag (which is also used by the Docker image) loads a
fixture named ``default`` which populates the database with a basic module
configuration. You should be able to view the CouchDB server by going to
``http://localhost:5984/_utils`` in your browser. This is a good way to test
that the project is actually running. See :ref:`ROSNodes` for
instructions on how to configure the software to interface with your specific
hardware.
