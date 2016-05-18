import sys
import time
import random
from couchdb import Server
from openag.brain.core import *

class Persistence(Module):
    desired_data = Input()
    measured_data = Input()

    def init(self, env_id: ReferenceParameter(DbName.ENVIRONMENT, "The id of "
            "the environment for which to record data")):
        server = Server()
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT.value]
        self.env_id = env_id

    def on_desired_data(self, item):
        point = EnvironmentalDataPointModel(
            environment=self.env_id,
            variable=item.data_type.value,
            is_desired=True,
            value=item.value,
            timestamp=item.timestamp
        )
        point["_id"] = gen_doc_id()
        point.store(self.db)

    def on_measured_data(self, item):
        point = EnvironmentalDataPointModel(
            environment=self.env_id,
            variable=item.data_type.value,
            is_desired=False,
            value=item.value,
            timestamp=item.timestamp
        )
        point["_id"] = gen_doc_id()
        point.store(self.db)
