.. _DatabaseFixtures:

Database Fixtures
=================

Default configurations for the database can be created with *fixtures*. Fixtures
are JSON documents that describe records that should be added to the database.

You can find the default JSON fixtures in `/fixtures <https://github.com/OpenAgInitiative/openag_brain/tree/master/fixtures>`_.

Fixtures follow the format::

    {
      database_name: [
        {
          _id: 'couchdb_doc_1',
          ...
        }
      ]
    }

Where each key in the top level object is the name of a Couch database and each
object in the array is a Couch document.

.. _FirmwareModuleFixtures:

Firmware Module Fixtures
------------------------

One place where fixtures come in handy is configuring a firmware module setup
for a particular Food Computer or Food Server model. Let's look at an example
``firmware_module`` database fixture::

  {
    "firmware_module": [
      {
        "_id": "dht22_1",
        "type": "dht22",
        "environment": "environment_1",
        "arguments": [5],
        "outputs": {
          "air_temperature": {"variable": "air_temperature"},
          "air_humidity": {"variable": "air_humidity"}
        }
      }
    ]
  }

``firmware_module`` is the database we're creating a fixture for. Within it is a list of JSON documents.

* ``_id``: an id for the record
* ``type``: the firmware module ID, as defined in the ``firmware_module_type``
  database. This tells the system what firmware module code to flash the
  Arduino with for this piece of firmware.
* ``environment``: the ID of the environment this sensor or actuator has
  been installed in. For most setups, this is ``environment_1``.
* ``arguments``: configuration arguments to pass to the firmware module. These
  are typically pin numbers. See the firmware module's documentation or class 
  definition for specifics.
* ``outputs``: a dictionary of keyed output ports. In the case of the example,
  the ``dht22_1`` outputs two sensing points (``air_temperature`` and
  ``air_humidity``). We map these outputs to environmental variable types.
* ``inputs``: some firmware may receive inputs for configuration or
  actuation.

Default values for ``arguments``, ``outputs`` and ``inputs`` can be configured
for each firmware type in the ``firmware_module_type`` database, so you don't
always have to specify all of these to configure a module.

.. _FirmwareTypeFixtures:

Firmware Type Fixtures
----------------------

The ``firmware_module_type`` database describes module types installed and
supported by the system. It acts as a package manager and glue layer for the
C++ firmware code that is flashed on to the Arduino.

Let's look at an example ``firmware_module_type`` database fixture::

    {
      "firmware_module_type": [
        {
          "_id": "atlas_do",
          "repository": {
            "type": "git",
            "url": "https://github.com/OpenAgInitiative/openag_atlas_do.git"
          },
          "description": "",
          "header_file": "openag_atlas_do.h",
          "class_name": "AtlasDo",
          "arguments": [
            {
              "name": "i2c_address",
              "type": "int",
              "default": 97
            }
          ],
          "inputs": {
            "atmospheric_calibration": {
              "type": "std_msgs/Empty",
              "categories": ["calibration"]
            },
            "zero_calibration": {
              "type": "std_msgs/Empty",
              "categories": ["calibration"]
            }
          },
          "outputs": {
            "water_dissolved_oxygen": {
              "type": "std_msgs/Float32"
            }
          },
          "dependencies": [
            {"type": "git", "url": "https://github.com/OpenAgInitiative/openag_firmware_module.git"}
          ]
        }
      ]
    }

Let's go over these fields:

* ``_id``: an id for the record. This is the same ID used in the ``type`` field
  of ``firmware_module`` records.
* ``repository``
  * ``type``: this should nearly always be ``git``
  * ``url``: the url of the Git repository
* ``description``: a friendly description of the module
* ``header_file``: the path to the C++ header file for the module.
* ``class_name``: the name of the class defined by the module file.
* ``arguments``: an array of named arguments to the module.
  * ``name``: the name key of the argument
  * ``type``: datatype (int, bool, ...)
  * ``default``: (optional) a default value for this argument.
* ``inputs``: a object of named inputs for the module (if any). The key
  is the name of the argument. The value should be an object with two fields:
  * ``type``: the datatype (usually a ROS topic type)
  * ``categories``: an array of category keywords.
* ``outputs``: an object of named outputs for the module. The key should be the
  name of the output. The value should be an object with one field:
  * ``type``: the datatype (usually a ROS topic type)
* ``dependencies``: an array of other OpenAg modules this module depends on.
  Each dependency is an object of:
  * ``type``: usually ``git``.
  * ``url``: the url of the repository.