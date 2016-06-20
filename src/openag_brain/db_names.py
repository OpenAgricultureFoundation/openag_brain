"""
This module defines the names of the databases to be accessed on the CouchDB
server storing data for the application.
"""
__all__ = ['DbName']

class DbName:
    """ Stores a set of constants representing the databases names """
    ENVIRONMENT = 'environment'
    """
    Stores information on each separate controlled environment contained in the
    system
    """

    ENVIRONMENTAL_DATA_POINT = 'environmental_data_point'
    """
    Stores data points recorded about the environmental conditions of the
    environment(s)
    """

    FIRMWARE_MODULE = 'firmware_module'
    """
    Stores information on Arduino modules currently in use in the system
    """

    FIRMWARE_MODULE_TYPE = 'firmware_module_type'
    """
    Stores information on Arduino modules that can be used with the system
    """

    RECIPE = 'recipes'
    """ Stores climate recipes for the system """

    SOFTWARE_MODULE = 'software_module'
    """
    Stores information on ROS modules currently in use in the system
    """

    SOFTWARE_MODULE_TYPE = 'software_module_type'
    """
    Stores information on ROS modules that can be used with the system
    """

