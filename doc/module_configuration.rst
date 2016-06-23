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

Persistence
~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.persistence

Recipe Handler
~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.recipe_handler

Topic Connector
~~~~~~~~~~~~~~~

.. automodule:: openag_brain.software_modules.topic_connector

Firmware Modules
----------------

Firmware modules are C++ classes that are generally drivers for individual
pieces of hardware. They must inherit from the :cpp:class:`Peripheral` class in
`openag_peripheral <http://github.com/OpenAgInitiative/openag_peripheral>`_.
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
* GC0012 Ambient CO2 Sensor
* Atlas Scientific EC Sensor
* Atlas Scientific pH Sensor
* Atlas Scientific RGB Sensor
