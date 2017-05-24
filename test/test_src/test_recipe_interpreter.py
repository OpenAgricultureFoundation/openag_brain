#!/usr/bin/env python
PKG="openag_brain"

import sys
import unittest

from time import time

MOCK_RECIPE_A = {
    "format": "simple",
    "version": "0.0.1",
    "operations": [
        [0, "air_temperature", 24],
        [0, "water_temperature", 22]
    ]
}


class TestRecipeInterpreter(unittest.TestCase):

    def test_interpret_simple_recipe():
       now_time = time()
       start_time = now_time - 100
       setpoints = recipe_handler.interpet_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
       print("Recipe Handler returns expected number of setpoints")
       assert len(setpoints) == 2


if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_recipe_interpreter', TestRecipeInterpreter)

