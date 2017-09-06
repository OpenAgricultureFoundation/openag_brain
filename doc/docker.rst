Docker
======

We use `Docker <http://www.docker.com/>`_ to containerize the openag_brain
environment, making it easy and fast to install.

**Note**: to install the openag_brain Docker image, don't use this repository directly.
Instead, see
`openag_brain_docker_rpi <https://github.com/OpenAgInitiative/openag_brain_docker_rpi>`_.

Working with Docker
-------------------

Starting the openag_brain Docker containers::

    docker-compose up -d

**Note**: if you install openag_brain with
`the Docker container script <https://github.com/OpenAgInitiative/openag_brain_docker_rpi>`_ you don't have to do this. It is taken care of for you.

2 Docker containers will be started in the background and will persist across
reboots. (If you don't have your Arduino connected, you will see an error.
If this happens, connect the Arduino and run the command again).

See which containers are running::

    docker ps

The install script creates and runs 2 docker containers. You should see them
listed when running `docker ps`::

    openagbraindockerrpi_brain_1
    openagbraindockerrpi_db_1

Stopping and starting the containers is done in the usual way::

    docker-compose start
    docker-compose stop
    docker-compose restart

You can get logs for the containers via docker's logging command::

    docker logs -f openagbraindockerrpi_brain_1

Working with the ROS container
-------------------------------------

openag_brain is powered by `ROS <http://www.ros.org/>`_. Sometimes, you might
want to interact with ROS in the Docker container directly. To do so, first
shell into the Docker container::

    docker exec -it openagbraindockerrpi_brain_1 bash

Then, activate the `catkin workspace <http://wiki.ros.org/catkin/Tutorials/using_a_workspace>`_
within the Docker container::

    source catkin_ws/devel/setup.bash

Now, you can interact with ROS.

To list available ROS topics::

    rostopic list

To log output from a ROS topic::

    rostopic echo <topic name>

To learn more about ROS, check out the `ROS wiki <http://wiki.ros.org/>`_.

Compiling and Publishing a new Docker Image
-------------------------------------------

Updated docker images will be published by the core team from the openag_brain
repository. "Official" OpenAg Docker images are published by the core team
from the openag_brain repository. If you want to contribute to a release,
`issue a pull request <https://github.com/OpenAgInitiative/openag_brain/compare>`_.
If you want to publish Docker images under your own account, you can follow
these steps, substituting your own Docker info:

Note: this section assumes you have a development environment set up with the
``developer_setup`` script that can be found in ``openag_brain/scripts``.

First, build ROS with ``catkin_make``::

    cd ~/catkin_ws
    catkin_make install

This make take a while... Once it's done, you should see a
``~/catkin_ws/install`` directory.

Note: the Dockerfile in ``openag_brain`` will ``ADD`` the contents
of the ``install`` directory to the Docker image, so it's necessary that
you run ``catkin_make install`` first, so Docker has something to copy.

Next, build the Docker image::

  cd ~/catkin_ws
  docker build -t openag/rpi_brain -f ./src/openag_brain/Dockerfile .

This will send the entire contents of the ``catkin_ws`` directory as the
Docker build context.

To publish the image, first log into Docker Hub::

    docker login

Finally, push the image to `Docker Hub <https://hub.docker.com/>`_::

    docker push openag/rpi_brain

More resources:

- http://wiki.openag.media.mit.edu/openag_brain/installing/installing_globally
- http://wiki.openag.media.mit.edu/docker
