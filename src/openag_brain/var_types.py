"""
This module defines all of the types of data that can be passed between
modules. There should be one type in this file for each variable type in the
database.
"""
__all__ = ['InternalVariable', 'EnvironmentalVariable']

class InternalVariable:
    TEST = 'test'
    BOOLEAN = 'boolean'
    STRING = 'string'
    REQUEST = 'request'
    RESPONSE = 'response'

class EnvironmentalVariable:
    AIR_TEMPERATURE = 'air_temperature'
    AIR_HUMIDITY = 'air_humidity'
    WATER_TEMPERATURE = 'water_temperature'
    RECIPE_START = 'recipe_start'
    RECIPE_END = 'recipe_end'
