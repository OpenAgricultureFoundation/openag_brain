OpenAg Brain
============

This project contains the code that runs on the Raspberry Pi of an OpenAg food
computer. It consists of a single `ROS <http://wiki.ros.org>`_ package that is
responsible for generating code for and flashing the Arduino responsible for
controlling the hardware, running control loops to maintain some given
environmental conditions, running recipes that define the desired environmental
conditions over time, and writing data to a `CouchDB
<http://couchdb.apache.org>`_ instance.

Contents
--------

.. toctree::
    :maxdepth: 2

    getting_started
    ros_nodes
    firmware
    commands
    api
    docker

Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

