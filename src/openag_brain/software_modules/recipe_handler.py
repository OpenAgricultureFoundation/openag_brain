#!/usr/bin/python
"""
The `recipe_handler.py` module is in charge of running recipes. It provides a
service `/<environment_id>/start_recipe` which takes as input a recipe ID and
starts the recipe. It also provides a service `/<environment_id>/stop_recipe`
which takes no inputs and stops the currently running recipe. It defines a
parameter `current_recipe` which stores the ID of the currently running recipe.
It also defines a parameter `current_recipe_start` which stores the UNIX
timestamp at which the currently running recipe was started. If no recipe is
running, `current_recipe` will be set to an empty string and
`current_recipe_start` will be set to 0. There should always be exactly one
instance of this module per environment in the system.
"""

import sys
import time
import random
from threading import Event, Timer

import rospy
from couchdb import Server
from std_msgs.msg import Float64

from openag_brain import params, services
from openag_brain.srv import StartRecipe, Empty
from openag_brain.db_names import DbName
from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.var_types import EnvironmentalVariable

class Recipe(object):
    def __init__(self, _id, operations, start_time):
        self._id = _id
        self.operations = operations
        self.start_time = start_time

    def set_points(self):
        raise NotImplementedError()

class SimpleRecipe(Recipe):
    def __init__(self, *args):
        super(SimpleRecipe, self).__init__(*args)
        self.current_index = 0
        self.is_running = True

    def next_operation(self):
        if self.current_index >= len(self.operations):
            return None
        else:
            return self.operations[self.current_index]

    def cancel(self):
        self.current_index = len(self.operations)
        self.is_running = False

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
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                if not self.is_running:
                    return
                rospy.sleep(1)
            yield (timestamp, variable, value)
            self.current_index += 1

class PublisherDict:
    def __init__(self):
        self.publishers = {}

    def __getitem__(self, topic):
        if not topic in self.publishers:
            self.publishers[topic] = rospy.Publisher(
                topic, Float64, queue_size=10
            )
        return self.publishers[topic]

class RecipeHandler(object):
    # Dictionary that maps recipe formats to python classes
    recipe_class_map = {
        "simple": SimpleRecipe
    }

    def __init__(self, server):
        self.env_data_db = server[DbName.ENVIRONMENTAL_DATA_POINT]
        self.recipe_db = server[DbName.RECIPE]

        # Indicates whether or not a recipe is running
        self.recipe_flag = Event()

        self.namespace = rospy.get_namespace()
        self.environment = self.namespace.split('/')[-2]

        rospy.init_node('recipe_handler')
        self.publishers = PublisherDict()
        rospy.Service(services.START_RECIPE, StartRecipe, self.start_recipe)
        rospy.Service(services.STOP_RECIPE, Empty, self.stop_recipe)
        rospy.set_param(
            params.SUPPORTED_RECIPE_FORMATS,
            ','.join(self.recipe_class_map.keys())
        )

        self.current_recipe = None
        self.current_set_points = {}

        rospy.set_param(params.CURRENT_RECIPE, "")
        rospy.set_param(params.CURRENT_RECIPE_START, 0)

        # Start publishing set points
        self.publish_set_points()

        # Get the recipe that has been started most recently
        start_view = self.env_data_db.view("openag/by_variable", startkey=[
            self.environment, EnvironmentalVariable.RECIPE_START, "desired"
        ], endkey=[
            self.environment, EnvironmentalVariable.RECIPE_START, "desired", {}
        ], group_level=3)
        if len(start_view) == 0:
            return
        start_doc = start_view.rows[0].value

        # If a recipe has been ended more recently than the most recent time a
        # recipe was started, don't run the recipe
        end_view = self.env_data_db.view("openag/by_variable", startkey=[
            self.environment, EnvironmentalVariable.RECIPE_END, "desired"
        ], endkey=[
            self.environment, EnvironmentalVariable.RECIPE_END, "desired", {}
        ], group_level=3)
        if len(end_view):
            end_doc = end_view.rows[0].value
            if (end_doc["timestamp"] > start_doc["timestamp"]):
                return

        # Run the recipe
        self.start_recipe(
            StartRecipe._request_class(start_doc["value"]),
            start_doc["timestamp"]
        )

    def publish_set_points(self):
        if rospy.is_shutdown():
            return
        for variable, value in self.current_set_points.items():
            self.publishers["desired_" + variable].publish(value)
        Timer(5, self.publish_set_points).start()

    def start_recipe(self, data, start_time=None):
        recipe_id = data.recipe_id
        if not recipe_id:
            return False, "No recipe id was specified"
        if self.recipe_flag.is_set():
            return False, "There is already a recipe running. Please stop it before "\
            "attempting to start a new one"
        try:
            recipe = self.recipe_db[recipe_id]
        except Exception as e:
            return False, "\"{}\" does not reference a valid "\
            "recipe".format(recipe_id)
        start_time = start_time or time.time()
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=EnvironmentalVariable.RECIPE_START,
            is_desired=True,
            value=recipe_id,
            timestamp=start_time
        )
        point.id = self.gen_doc_id(start_time)
        point.store(self.env_data_db)
        rospy.set_param(params.CURRENT_RECIPE, recipe_id)
        rospy.set_param(params.CURRENT_RECIPE_START, start_time)
        self.current_recipe = self.recipe_class_map[
            getattr(recipe, "format", "simple")
        ](
            recipe.id, recipe["operations"], start_time
        )
        self.recipe_flag.set()
        return True, "Success"

    def stop_recipe(self, data):
        self.current_recipe.cancel()
        return True, "Success"

    def run(self):
        while True:
            while not self.recipe_flag.is_set():
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.sleep(1)
            self.current_set_points = {}
            for timestamp, variable, value in self.current_recipe.set_points():
                self.current_set_points[variable] = value
                self.publishers["desired_"+ variable].publish(value)
            curr_time = time.time()
            point = EnvironmentalDataPointModel(
                environment=self.environment,
                variable=EnvironmentalVariable.RECIPE_END,
                is_desired=True,
                value=self.current_recipe._id,
                timestamp=curr_time
            )
            point.id = self.gen_doc_id(curr_time)
            point.store(self.env_data_db)
            rospy.set_param(params.CURRENT_RECIPE, "")
            rospy.set_param(params.CURRENT_RECIPE_START, 0)
            self.recipe_flag.clear()

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))

if __name__ == '__main__':
    server = Server(rospy.get_param('/' + params.DB_SERVER))
    mod = RecipeHandler(server)
    try:
        mod.run()
    except rospy.ROSInterruptException:
        pass
