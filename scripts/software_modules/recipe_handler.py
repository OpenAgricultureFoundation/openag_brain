#!/usr/bin/python

import sys
import time
import random
from threading import Event, Timer

import rospy
from couchdb import Server
from std_msgs.msg import Float64

from openag_brain.srv import StartRecipe, Empty
from openag_brain.db_names import DbName
from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.var_types import EnvironmentalVariable

CURRENT_RECIPE = 'current_recipe'
CURRENT_RECIPE_START = 'current_recipe_start'

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
        rospy.Service('start_recipe', StartRecipe, self.start_recipe)
        rospy.Service('stop_recipe', Empty, self.stop_recipe)

        self.current_recipe = None
        self.current_set_points = {}

        # Start publishing set points
        self.publish_set_points()

        # Get the recipe that has been started most recently
        start_view = self.env_data_db.view("openag/latest", key=[
            self.environment, EnvironmentalVariable.RECIPE_START, "desired"
        ])
        if len(start_view) == 0:
            return
        start_doc = start_view.rows[0].value

        # If a recipe has been ended more recently than the most recent time a
        # recipe was started, don't run the recipe
        end_view = self.env_data_db.view("openag/latest", key=[
            self.environment, EnvironmentalVariable.RECIPE_END, "desired"
        ])
        if len(end_view):
            end_doc = end_view.rows[0].value
            if (end_doc["timestamp"] > start_doc["timestamp"]):
                return

        # Run the recipe
        self.start_recipe(
            ChangeString._request_class(start_doc["value"]),
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
            return "No recipe id was specified", 400
        if self.recipe_flag.is_set():
            return "There is already a recipe running. Please stop it before "\
            "attempting to start a new one", 400
        try:
            recipe = self.recipe_db[recipe_id]
        except Exception as e:
            return "\"{}\" does not reference a valid "\
            "recipe".format(recipe_id), 400
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
        rospy.set_param(CURRENT_RECIPE, recipe_id)
        rospy.set_param(CURRENT_RECIPE_START, start_time)
        self.current_recipe = self.recipe_class_map[
            getattr(recipe, "format", "simple")
        ](
            recipe.id, recipe["operations"], start_time
        )
        self.recipe_flag.set()
        return "Success", 200

    def stop_recipe(self, data):
        self.current_recipe.cancel()
        return "Success", 200

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
            rospy.delete_param(CURRENT_RECIPE)
            rospy.delete_param(CURRENT_RECIPE_START)
            self.recipe_flag.clear()

    def gen_doc_id(self, curr_time):
        return "{}-{}".format(curr_time, random.randint(0, sys.maxsize))

if __name__ == '__main__':
    server = Server(rospy.get_param("/database"))
    mod = RecipeHandler(server)
    try:
        mod.run()
    except rospy.ROSInterruptException:
        pass
