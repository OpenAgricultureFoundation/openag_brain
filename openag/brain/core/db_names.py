"""
This module defines the names of the databases to be accessed on the CouchDB
server storing data for the application.
"""
from enum import Enum

__all__ = ['DbName']

class DbName(Enum):
    RECIPE = 'recipes'
    MODULE = 'module'
    MODULE_TYPE = 'module_type'
    MODULE_GROUP = 'module_group'
    MODULE_CONNECTION = 'module_connection'
    ENVIRONMENT = 'environment'
    ENVIRONMENTAL_DATA_POINT = 'environmental_data_point'
