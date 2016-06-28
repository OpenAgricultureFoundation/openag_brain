"""
This `openag_brain.var_types` module defines some standard variable types that
can be used for the `variable` field in an `EnvironmentalDataPoint`.
"""
__all__ = ["EnvironmentalVariable"]

class EnvironmentalVariable(object):
    """ A set of variable type constants """
    AIR_TEMPERATURE = "air_temperature"
    AIR_HUMIDITY = "air_humidity"
    AIR_CARBON_DIOXIDE = "air_carbon_dioxide"
    WATER_TEMPERATURE = "water_temperature"
    WATER_EC = "water_electrical_conductivity"
    WATER_PH = "water_potential_hydrogen"
    LIGHT_ILLUMINANCE = "light_illuminance"
    LIGHT_SPECTRUM = "light_spectrum"
    RECIPE_START = "recipe_start"
    """
    The value field for data points of this type should be the ID of the recipe
    that was run.
    """
    RECIPE_END = "recipe_end"
    """
    The value field for data points of this type should be the ID of the recipe
    that was run.
    """
