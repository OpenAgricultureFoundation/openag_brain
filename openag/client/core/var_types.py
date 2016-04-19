"""
This module defines all of the types of data that can be passed between
modules. There should be one type in this file for each variable type in the
database.
"""
from enum import Enum

class InternalVariable(Enum):
    TEST = 'test'
    REQUEST = 'request'
    RESPONSE = 'response'

class EnvironmentalVariable(Enum):
    AIR_TEMPERATURE = 'air_temperature'
    HUMIDITY = 'humidity'
