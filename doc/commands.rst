Commands
========

The project contains several execuables for interacting with the system. In
most cases, just running the `init_db` and `main` commands is sufficient, but the
others may prove useful for developers.

.. _OpenagCmdInitDb:

Main
----

.. code-block:: bash

    rosrun openag_brain main

.. program-output:: rosrun openag_brain main -h

.. _OpenagCmdInstallPio:

Install PlatformIO
------------------

Rosserial and PlatformIO depend on different versions of the `pyserial`
library, so installing them normally through a package manager will break
things. Because of this, we recommend installing PlatformIO in a virtual
environment. This script automates that process.

.. code-block:: bash

    rosrun openag_brain install_pio

.. _OpenagCmdFirmware:

Build and Flash Firmware
------------------------

For more information on the firmware system, see :ref:`Firmware`.

.. code-block:: bash

    rosrun openag_brain firmware

.. program-output:: rosrun openag_brain firmware -h

Init PlatformIO
---------------

PlatformIO projects need to be configured once. A default configuration is already checked in, so you shoudn't have to do this yourself. However, if you want to change the default configuration, you can use `platformio init <http://docs.platformio.org/en/latest/userguide/cmd_init.html>`_. For example:

.. code-block:: bash

    platformio init --board chiwawa