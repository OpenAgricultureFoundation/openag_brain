import sys
import time
import random
import gevent
from gevent.event import Event
from couchdb import Server
from openag.brain.core import *

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
        if self.current_index >= len(self.operations):
            return None
        else:
            return self.operations[self.current_index]

    def cancel(self):
        self.current_index = len(self.operations)

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

    def init(self, env_id: ReferenceParameter(DbName.ENVIRONMENT, "The id of "
            "the environment for which to record data")):
        global server
        self.env_data_db = server[DbName.ENVIRONMENTAL_DATA_POINT.value]
        self.recipe_db = server[DbName.RECIPE.value]
        self.env_id = env_id
        self.current_recipe = None

        # Indicates whether or not a recipe is running
        self.recipe_flag = Event()

        # Get the recipe that has been started most recently
        start_view = self.env_data_db.view("openag/latest", key=[
            env_id, EnvironmentalVariable.RECIPE_START.value, "desired"
        ])
        if len(start_view) == 0:
            return
        start_doc = view.rows[0].value

        # If a recipe has been ended more recently than the most recent time a
        # recipe was started, don't run the recipe
        end_view = self.env_data_db.view("openag/latest", key=[
            env_id, EnvironmentalVariable.RECIPE_END.value, "desired"
        ])
        if len(end_view):
            end_doc = view.rows[0].value
            if (end_doc["timestamp"] > start_doc["timestamp"]):
                return

        # Run the recipe
        self.start_recipe(doc["value"], doc["timestamp"])

    @endpoint
    def start_recipe(self, recipe_id: "The id of the recipe to run",
            start_time: "Time to start the recipe" = None):
        recipe = self.recipe_db[recipe_id]
        start_time = start_time or time.time()
        point = EnvironmentalDataPointModel(
            environment=self.env_id,
            variable=EnvironmentalVariable.RECIPE_START.value,
            is_desired=True,
            value=recipe_id,
            timestamp=start_time
        )
        point["_id"] = gen_doc_id()
        point.store(self.env_data_db)
        self.current_recipe = self.recipe_class_map[
            getattr(recipe, "format", "simple")
        ](
            recipe["_id"], recipe["operations"], start_time
        )
        self.recipe_flag.set()
        return "Success"

    @endpoint
    def stop_recipe(self):
        self.current_recipe.cancel()

    def run(self):
        while True:
            self.recipe_flag.wait()
            for timestamp, variable, value in self.current_recipe.set_points():
                self.set_points.emit(value, variable, timestamp)
            point = EnvironmentalDataPointModel(
                environment=self.env_id,
                variable=EnvironmentalVariable.RECIPE_END.value,
                is_desired=True,
                value=self.current_recipe._id,
                timestamp=time.time()
            )
            point["_id"] = gen_doc_id()
            self.recipe_flag.clear()
