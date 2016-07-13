.. _ModuleConfiguration:

Module Configuration
====================

For flexibility and hackability, the project is structured to be as modular as
possible. The core of the project is simply a framework that runs software
modules on the Raspberry and firmware modules on the Arduino and allows for
dynamic reconfiguration of both at runtime. Communication between modules
happens through ROS topics and services.

Software Modules
----------------

Software modules are arbitrary executables that run on the Raspberry Pi. They
are allowed to publish and subscribe to ROS topics and provide ROS services for
communication. The `software_module_type` database contains a repository of all
of the types of software modules that can be run in the system. The
`software_module` database contains the current configuration of software
modules in the system. When the :ref:`OpenagCmdMain` script is run, it
generates a roslaunch file based on the configuration of software module using
the :ref:`OpenagCmdUpdateLaunch` command and runs the generated roslaunch file.
It also updates the roslaunch file and restarts the modules whenever the
configuration of software modules in the database is changed, allowing for the
modules to be reconfigured dynamically. For information on the structure of the
`software_module_type` and `software_module` databases, see
:py:class:`~openag_brain.models.SoftwareModuleTypeModel` and
:py:class:`~openag_brain.models.SoftwareModuleModel`, respectively.

The following are all software modules that come bundled with the system. To
write your own software module type, simply define an executable file with the
code for the software module type, install it somewhere on the ROS path and
create an entry in the `software_module_type` database for it. Then, you should
be able to create instances of the module type by creating entries in the
`software_module` database.

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

Camera

Firmware Modules
----------------

Firmware modules are C++ classes that are generally drivers for individual
pieces of hardware. They must follow a specific format so that this project can
programmatically generate Arduino code that leverages them. The classes must
define the following properties and functions:

.. cpp:member:: bool has_error

  This attribute should be set to True to indicate that the firmware module
  encountered an error in operation.

.. cpp:member:: char* error_msg

  This attribute should hold a string describing the error that occurred
  whenever `has_error` is true.

.. cpp:function:: void begin()

  This function can be used to initialize the module and its attributes.

.. cpp:function:: void update()

  This function is called once per loop and should do the bulk of the work for
  the module. For example, if the module is a driver for a sensor, this
  function should read from the sensor and store the read data somewhere
  internally.

.. cpp:function:: bool get_<variable>(<msg_type> &msg)

  The module must define a getter in this format for every variable it outputs.
  For example, if a module outputs air_temperature as a 32-bit floating point
  number, it should define a function `get_air_temperature(std_msgs::Float32
  &msg)`. The function should populate the message object that is passed in
  with the value read from the sensor and return a boolean value indicating
  whether or not the message should be sent out to the Raspberry Pi.


The `firmware_module_type` database contains a repository of all of the types
of firmware modules that can be run in the system. The `firmware_module`
database contains the current conifiguration of firmware modules in the system.
When the :ref:`OpenagCmdGenerateFirmware` script is run, it generates Arduino
code based on the configuration of firmware modules that can be flashed to the
Arduino to interface with the hardware. For information on the structure of the
`firmware_module_type` and `firmware_module` databases, see
:py:class:`~openag_brain.models.FirmwareModuleTypeModel` and
:py:class:`~openag_brain.models.FirmwareModelModel`, respectively.

The following is a list of peripherals for which firmware modules already
exist:

* AM2315 Temperature/Humidity Sensor
* Atlas Scientific EC Sensor
* Atlas Scientific pH Sensor
* Atlas Scientific RGB Sensor
* Generic binary actuator
* Generic analog (pwm) actuator
