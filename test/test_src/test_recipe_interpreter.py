#!/usr/bin/env python
PKG="openag_brain"

import sys, os
import unittest
from time import time
from datetime import datetime
import json

DIR_NAME = os.path.dirname(__file__)
sys.path.append(os.path.join(DIR_NAME, '../..'))
from openag_brain.recipe_interpreters import interpret_simple_recipe, interpret_phased_dense_recipe
from test.data.mock_recipes import phased_dense, MOCK_RECIPE_A


def load_recipe_from_file(file_name):
    with open(file_name, 'r') as f:
        recipe = json.load(f)
    return recipe


def _test_load_recipe_from_file():
    recipe = load_recipe_from_file(os.path.join(DIR_NAME, '../../test/data/mock_recipe_v3.json'))
    print(recipe)
    assert type(recipe) == dict
    assert False


class TestRecipeInterpreter(unittest.TestCase):

    def test_interpret_simple_recipe(self):
        now_time = time()
        start_time = now_time - 10
        setpoints = interpret_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
        assert len(setpoints) == 2
  

    def test_interpret_phased_dense_recipe(self):
        #now_time = time()
        #start_time = now_time - 10
        start_time = datetime.strptime("2017-04-17 14:00", "%Y-%m-%d %H:%S")
        now_time = datetime.strptime("2017-04-18 20:00", "%Y-%m-%d %H:%S")
        setpoints = interpret_phased_dense_recipe(phased_dense, start_time, now_time)
        assert len(setpoints) == 4


if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_recipe_interpreter', TestRecipeInterpreter)
