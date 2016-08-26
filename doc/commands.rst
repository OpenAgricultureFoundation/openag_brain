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

.. _OpenagCmdLoadFixture:

Load Fixture
------------

Fixtures are JSON documents containing a list of objects to create on the
CouchDB server. They are mostly used for bootstrapping an installation of the
system.

.. code-block:: bash

    rosrun openag_brain load_fixture

.. program-output:: rosrun openag_brain load_fixture -h

.. _OpenagCmdUpdateLaunch:

Update Roslaunch File
---------------------

.. code-block:: bash

    rosrun openag_brain update_launch

.. program-output:: rosrun openag_brain update_launch -h
