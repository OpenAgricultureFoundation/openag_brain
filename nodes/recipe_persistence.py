#!/usr/bin/python

import time
import rospy
from roslib.message import get_message_class
from couchdb import Server
from std_msgs.msg import Float64

from openag_lib.db_bootstrap.db_names import ENVIRONMENTAL_DATA_POINT
from openag_lib.config import config as cli_config
from openag_brain.models import EnvironmentalDataPoint
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


#-----------------------------------------------------------------------------
class TopicPersistence:
    def __init__(
        self, db, topic, topic_type, environment, variable, is_desired
    ):
        self.db = db
        self.topic_type = topic_type
        self.environment = environment
        self.variable = variable
        self.is_desired = is_desired
        self.last_value = None
        self.sub = rospy.Subscriber(topic, topic_type, self.on_data)

    def on_data(self, item):
        curr_time = time.time()
        value = item.data
        if value is None or value == self.last_value:
            return
        # This is kind of a hack to correctly interpret UInt8MultiArray
        # messages. There should be a better way to do this
        if item._slot_types[item.__slots__.index('data')] == "uint8[]":
            value = [ord(x) for x in value]
        # Throttle updates by value only (not time)
        if self.topic_type == Float64 and \
           self.last_value is not None and \
           self.last_value != 0.0:
            delta_val = value - self.last_value
            if abs(delta_val / self.last_value) <= 0.01:
                return
        # Save the data point
        point = EnvironmentalDataPoint({
            "environment": self.environment,
            "variable": self.variable,
            "is_desired": self.is_desired,
            "value": value,
            "timestamp": curr_time
        })
        point_id = gen_doc_id(curr_time)
        self.db[point_id] = point
        self.last_value = value


#-----------------------------------------------------------------------------
def create_persistence_objects( server, environment_id, ):
    env_var_db = server[ENVIRONMENTAL_DATA_POINT]
    for variable in VALID_VARIABLES:
        topic = "{}/desired".format(variable.name)
        TopicPersistence(
            db=env_var_db, 
            topic=topic, 
            topic_type=get_message_class(variable.type),
            environment=environment_id,
            variable=variable.name, is_desired=True
        )


#-----------------------------------------------------------------------------
if __name__ == "__main__":

    rospy.init_node('recipe_persistence')

    # Initialize the database server object
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    environment_id = read_environment_from_ns(rospy.get_namespace())
    create_persistence_objects( server, environment_id )

    # Keep running until ROS stops
    rospy.spin()
