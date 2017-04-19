.. _ROSNodes:

ROS Nodes
=========

openag_brain is made of of lots of small scripts called ``nodes``. Each ``node`` solves a single problem, like posting to the database, or reading from a sensor.

Nodes send messages to each other using `ROS Topics <http://wiki.ros.org/Topics>`_. 

Launch Files
------------

Nodes can be configured into a working system using `.launch <http://wiki.ros.org/roslaunch/XML>`_ files. These are simple XML files that describe which nodes should run and what their parameters should be.

Launch files are kept in the ``launch/`` directory. openag_brain currently has one launch file ``launch/personal_food_computer_v2.launch``. In future, there may be more for other hardware configurations.

Params
~~~~~~

Each node can be configured with a set of parameters. You can also load parameters into a launch file in bulk `using .yaml files <http://wiki.ros.org/roslaunch/XML/rosparam>`_.

We use ``launch/personal_food_computer_v2.yaml`` to load in a set of parameters for the Arduino :ref:`Firmware`.

Nodes
-----

The following are all nodes that come bundled with the system. To write your own node, simply define an executable file (usually a Python file) with the code for the node, install it somewhere on the ROS path and create a launch file that configures it.

API
~~~

.. automodule:: openag_brain.software_modules.api

For information on the API endpoints, see :ref:`APIEndpoints`

Handle Arduino
~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.handle_arduino

Image Persistence
~~~~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.image_persistence

PID Controller
~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.pid

Recipe Handler
~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.recipe_handler

Sensor Persistence
~~~~~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.sensor_persistence

Topic Connector
~~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.topic_connector

Topic Filter
~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.topic_filter

Video Writer
~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.video_writer
