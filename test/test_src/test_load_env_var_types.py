#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_load_env_var_types'

from openag_brain.load_env_var_types import create_variables, VariableInfo
import unittest
import rospy
from roslib.message import get_message_class

var_dict = {'air_temp': {'units': 'C', 'name': 'air_temp', 'description': 'Measurement of the surrounding air temperature in Celcius'},
             'air_press': {'units': 'mmhg', 'name': 'air_pres', 'description': 'Measurement of the air pressure in mmhg.'}}

class TestLoadEnvironmentalVariables(unittest.TestCase):

    def test_create_variables(self):
        variables = create_variables(var_dict)
        print(variables["air_temp"].name)
        assert isinstance(variables["air_temp"], VariableInfo)
        assert isinstance(variables["air_press"], VariableInfo)
        assert variables["air_temp"].name == "air_temp"
        assert variables["air_temp"].__doc__ == var_dict["air_temp"]["description"]
        assert variables["air_temp"].units == var_dict["air_temp"]["units"]


    def test_valid_variables(self):
        ENVIRONMENTAL_VARIABLES = frozenset(
            VariableInfo.from_dict(d)
            for d in rospy.get_param("/var_types/environment_variables").itervalues())
 
        RECIPE_VARIABLES = frozenset(
            VariableInfo.from_dict(d)
            for d in rospy.get_param("/var_types/recipe_variables").itervalues())
 
        VALID_VARIABLES = ENVIRONMENTAL_VARIABLES.union(RECIPE_VARIABLES)
        print(VALID_VARIABLES)
        assert len(VALID_VARIABLES) == 18


        # This builds a dictionary of publisher instances using a
        # "dictionary comprehension" (syntactic sugar for building dictionaries).
        # The constant has to be declared here because get_message_class
        # needs to be called after the node is initialized.
        PUBLISHERS = {
            variable.name: rospy.Publisher(
                "{}/desired".format(variable.name),
                get_message_class(variable.type),
                queue_size=10)
            for variable in VALID_VARIABLES
        }

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestLoadEnvironmentalVariables)
