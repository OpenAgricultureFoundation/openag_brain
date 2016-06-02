"""
This module defines the names of the databases to be accessed on the CouchDB
server storing data for the application.
"""
__all__ = ['DbName']

class DbName:
    RECIPE = 'recipes'
    SOFTWARE_MODULE_TYPE = 'software_module_type'
    SOFTWARE_MODULE = 'software_module'
    FIRMWARE_MODULE_TYPE = 'firmware_module_type'
    FIRMWARE_MODULE = 'firmware_module'
    ENVIRONMENT = 'environment'
    ENVIRONMENTAL_DATA_POINT = 'environmental_data_point'
