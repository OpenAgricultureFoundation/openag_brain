import sys
import time
import random
import gevent
from couchdb import Server
from openag.client.core import *

class Recipe:
    def __init__(self, _id, operations, start_time):
        self._id = _id
        self.operations = operations
        self.start_time = start_time

    def set_points(self):
        raise NotImplementedError()

class SimpleRecipe(Recipe):
    def __init__(self, _id, operations, start_time):
        super().__init__(_id, operations, start_time)
        self.current_index = 0

    def next_operation(self):
        if self.current_index == len(self.operations):
            return None
        else:
            return self.operations[self.current_index]

    def set_points(self):
        initial_values = {}
        while True:
            next_operation = self.next_operation()
            if next_operation is None:
                break
            offset, variable, value = next_operation
            next_time = self.start_time + offset
            if next_time < time.time():
                initial_values[variable] = (next_time, value)
                self.current_index += 1
            else:
                break
        for variable, (timestamp, value) in initial_values.items():
            yield (timestamp, variable, value)
        while True:
            next_operation = self.next_operation()
            if next_operation is None:
                return
            offset, variable, value = next_operation
            next_time = self.start_time + offset
            while next_time > time.time():
                gevent.sleep(1)
            yield (timestamp, variable, value)
            self.current_index += 1

class RecipeHandler(Module):
    set_points = Output()

    # Dictionary that maps recipe formats to python classes
    recipe_class_map = {
        "simple": SimpleRecipe
    }

    def init(self, env_id: "The id of the environment for which to record data"
            = None):
        server = Server()
        self.env_data_db = server[DbName.ENVIRONMENTAL_DATA_POINT.value]
        self.recipe_db = server[DbName.RECIPE.value]
        self.env_id = env_id
        self.current_recipe = None
        # Check if there is a recipe currently running
        view = self.env_data_db.view("openag/latest", key=[
            env_id, EnvironmentalVariable.RECIPE_START.value, "desired"
        ])
        if len(view):
            doc = view.rows[0].value
            self.start_recipe(doc["value"], doc["timestamp"])

    @endpoint
    def start_recipe(self, recipe_id: "The id of the recipe to run" = None,
            start_time: "Time to start the recipe" = None):
        recipe = self.recipe_db[recipe_id]
        start_time = start_time or time.time()
        point = EnvironmentalDataPointModel(
            _id=gen_doc_id(),
            environment=self.env_id,
            variable=EnvironmentalVariable.RECIPE_START.value,
            is_desired=True,
            value=recipe_id,
            timestamp=start_time
        )
        point.store(self.env_data_db)
        self.current_recipe = self.recipe_class_map[
                getattr(recipe, "format", "simple")
        ](
            recipe["_id"], recipe["operations"], start_time
        )
        return "Success"

    def run(self):
        while True:
            while self.current_recipe is None:
                gevent.sleep(1)
            for timestamp, variable, value in self.current_recipe.set_points():
                self.set_points.emit(value, variable, timestamp)
            point = EnvironmentalDataPointModel(
                _id=gen_doc_id(),
                environment=self.env_id,
                variable=EnvironmentalVariable.RECIPE_END.value,
                is_desired=True,
                value=self.current_recipe._id,
                timestamp=time.time()
            )
            self.current_recipe = None
