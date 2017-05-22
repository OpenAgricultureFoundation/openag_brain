"""
@FIXME this doesn't work yet because of Python import problems.
Figure out how to do unit testing in ROS environment.
"""
from ...src.openag_brain import recipe_handler
from time import time

MOCK_RECIPE_A = {
    "format": "simple",
    "version": "0.0.1",
    "operations": [
        [0, "air_temperature", 24],
        [0, "water_temperature", 22]
    ]
}

def test_interpet_simple_recipe():
    now_time = time()
    start_time = now_time - 100
    setpoints = recipe_handler.interpet_simple_recipe(MOCK_RECIPE_A, start_time, now_time)
    print("Recipe Handler returns expected number of setpoints")
    assert len(setpoints) == 2