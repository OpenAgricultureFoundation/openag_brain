#!/usr/bin/python

import time
import rospy
from roslib.message import get_message_class
from couchdb import Server

from openag.db_names import ENVIRONMENTAL_DATA_POINT
from openag.cli.config import config as cli_config
from openag.models import EnvironmentalDataPoint

from openag_brain.utils import gen_doc_id, read_environment_from_ns
from openag_brain.load_env_var_types import VariableInfo

# Create a tuple constant of valid environmental variables
# Should these be only environment_variables?
ENVIRONMENTAL_VARIABLES = frozenset(
    VariableInfo.from_dict(d)
    for d in rospy.get_param("/var_types/environment_variables").itervalues()
    )

RECIPE_VARIABLES = frozenset(
    VariableInfo.from_dict(d)
    for d in rospy.get_param("/var_types/recipe_variables").itervalues()
    )

VALID_VARIABLES = ENVIRONMENTAL_VARIABLES.union(RECIPE_VARIABLES)


if __name__ == "__main__":

    rospy.init_node('recipe_persistence')
    namespace = rospy.get_namespace()
    environment = read_environment_from_ns(namespace)

    # Initialize the database server object
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    environmental_data_db = Server(db_server)[ENVIRONMENTAL_DATA_POINT]

    def generate_callback(variable):
        """
        ROS Subscribers need a callback function to call when a topic gets published to.
        This function is a high order function that takes the variable name which is determined at
        Subscriber generation time, to create a callback function which takes a topic and persists it to the database.
        """
        def desired_callback(desired_data):
            timestamp = time.time()

            doc = EnvironmentalDataPoint({
                "environment": environment,
                "variable": variable,
                "is_desired": True,
                "value": desired_data.data,
                "timestamp": timestamp
            })

            doc_id = gen_doc_id(timestamp)
            environmental_data_db[doc_id] = doc

        return desired_callback

    for variable in VALID_VARIABLES:
        rospy.Subscriber(
            "~{}/desired".format(variable.name),
            get_message_class(variable.type),
            generate_callback(variable.name)
        )

    # Keep running until ROS stops
    rospy.spin()
