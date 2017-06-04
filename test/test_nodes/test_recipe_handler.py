#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_recipe_handler'

import sys, os
import unittest
from time import time
import rospy
from std_msgs.msg import Float64
from openag_brain import services
from openag_brain.srv import StartRecipe

# Hack to allow importing of functions from ros nodes. 
#  Ideas to come up with a permanent solution: (Need to research the best known methods)
#     1. Figure out how nodes are loaded into the default path or if they are just run as subprocesses by ros
#     2. Split out functions as a separate services or move to a standard library to be imported into the nodes. 
#          This would mean the nodes do nothing but run services, publish and script to topics.
DIR_NAME = os.path.dirname(__file__)
sys.path.append(os.path.join(DIR_NAME, '../..'))
from nodes.recipe_handler import interpret_simple_recipe


MOCK_RECIPE_A = {
            "_id": "air_water_temp_test",
            "format": "simple",
            "version": "0.0.1",
            "operations": [
                            [0, "air_temperature", 24],
                            [0, "water_temperature", 22],
                            [30, "air_temperature", 24],  # Lesson Learned interpret_simple_recipe calculates the end time by the last time in the list
                            [30, "water_temperature", 22]
                          ]
            }

class TestRecipeHandler(unittest.TestCase):
    """
    Test the recipe_handler.py node to verify it is returning the topics
    correctly.
    """

    def test_recipe_interpreter(self):
        now_time = time()
        # Test for recipe in process
        start_time = now_time - 10
        setpoints = interpret_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
        assert len(setpoints) == 2
        # Test for completed recipe
        start_time = now_time - 300
        setpoints = interpret_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
        assert setpoints[0][0] == 'recipe_end'

#    def test_start_recipe(self):
#       rospy.wait_for_service(services.START_RECIPE)
#       s = rospy.ServiceProxy(services.START_RECIPE, StartRecipe)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestRecipeHandler)

