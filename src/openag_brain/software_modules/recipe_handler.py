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
import rospy
import random
from openag.db_names import ENVIRONMENTAL_DATA_POINT, RECIPE
from openag.cli.config import config as cli_config
from openag.models import EnvironmentalDataPoint
from openag.var_types import RECIPE_START, RECIPE_END, EnvVar
from couchdb import Server
from std_msgs.msg import Float64
from threading import Event, Timer

from openag_brain import params, services
from openag_brain.srv import StartRecipe, Empty

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
        if not isinstance(self.operations, list):
            raise ValueError(
                "Invalid recipe. Operations should be an array of set points."
            )
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
                "desired/"+topic, Float64, queue_size=10
            )
        return self.publishers[topic]

class RecipeHandler(object):
    # Dictionary that maps recipe formats to python classes
    recipe_class_map = {
        "simple": SimpleRecipe
    }

    def __init__(self, server):
        self.env_data_db = server[ENVIRONMENTAL_DATA_POINT]
        self.recipe_db = server[RECIPE]

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

        self.valid_variables = list(EnvVar.items.keys())

        rospy.set_param(params.CURRENT_RECIPE, "")
        rospy.set_param(params.CURRENT_RECIPE_START, 0)

        # Start publishing set points
        self.publish_set_points()

        # Get the recipe that has been started most recently
        start_view = self.env_data_db.view("openag/by_variable", startkey=[
            self.environment, "desired", RECIPE_START.name
        ], endkey=[
            self.environment, "desired", RECIPE_START.name, {}
        ], group_level=3)
        if len(start_view) == 0:
            return
        start_doc = start_view.rows[0].value

        # If a recipe has been ended more recently than the most recent time a
        # recipe was started, don't run the recipe
        end_view = self.env_data_db.view("openag/by_variable", startkey=[
            self.environment, "desired", RECIPE_END.name,
        ], endkey=[
            self.environment, "desired", RECIPE_END.name, {}
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
            if variable in self.valid_variables:
                self.publishers[variable].publish(value)
            else:
                rospy.logwarn('Recipe references invalid variable "{}"'.format(
                    variable
                ))
        Timer(5, self.publish_set_points).start()

    def start_recipe(self, data, start_time=None):
        recipe_id = data.recipe_id
        if not recipe_id:
            return False, "No recipe id was specified"
        if self.recipe_flag.is_set():
            return False, "There is already a recipe running. Please stop it "\
                "before attempting to start a new one"
        try:
            recipe = self.recipe_db[recipe_id]
        except Exception as e:
            return False, "\"{}\" does not reference a valid "\
            "recipe".format(recipe_id)
        start_time = start_time or time.time()
        self.current_recipe = self.recipe_class_map[
            recipe.get("format", "simple")
        ](
            recipe_id, recipe["operations"], start_time
        )
        point = EnvironmentalDataPoint({
            "environment": self.environment,
            "variable": RECIPE_START.name,
            "is_desired": True,
            "value": recipe_id,
            "timestamp": start_time
        })
        point_id = self.gen_doc_id(start_time)
        self.env_data_db[point_id] = point
        rospy.set_param(params.CURRENT_RECIPE, recipe_id)
        rospy.set_param(params.CURRENT_RECIPE_START, start_time)
        self.recipe_flag.set()
        rospy.loginfo('Starting recipe "{}"'.format(recipe_id))
        return True, "Success"

    def stop_recipe(self, data):
        if self.current_recipe and self.current_recipe.is_running:
            self.current_recipe.cancel()
            while self.recipe_flag.is_set():
                rospy.sleep(1)
            return True, "Success"
        else:
            return False, "There is no recipe running"

    def run(self):
        while True:
            while not self.recipe_flag.is_set():
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.sleep(1)
            self.current_set_points = {}
            for timestamp, variable, value in self.current_recipe.set_points():
                if variable in self.valid_variables:
                    self.current_set_points[variable] = value
                    self.publishers[variable].publish(value)
                else:
                    rospy.logwarn('Recipe references invalid variable "{}"'.format(
                        variable
                    ))
            curr_time = time.time()
            point = EnvironmentalDataPoint({
                "environment": self.environment,
                "variable": RECIPE_END.name,
                "is_desired": True,
                "value": self.current_recipe._id,
                "timestamp": curr_time
            })
            point_id = self.gen_doc_id(curr_time)
            self.env_data_db[point_id] = point
            rospy.set_param(params.CURRENT_RECIPE, "")
            rospy.set_param(params.CURRENT_RECIPE_START, 0)
            self.recipe_flag.clear()

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))

if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    mod = RecipeHandler(server)
    try:
        mod.run()
    except rospy.ROSInterruptException:
        pass
