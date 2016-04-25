import sys
import time
import random
from couchdb import Server
from openag.client.core import *

class Persistence(Module):
    desired_data = Input()
    measured_data = Input()

    def init(self, env_id: "The id of the environment for which to record " \
            "data" = None):
        server = Server()
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT.value]
        # TODO: Make sure an envinroment with this ID already exists
        self.env_id = env_id

    def on_desired_data(self, item):
        point = EnvironmentalDataPointModel(
            _id=_gen_doc_id(),
            environment=self.env_id,
            variable=item.data_type.value,
            is_desired=True,
            value=item.value,
            timestamp=item.timestamp
        )
        point.store(self.db)

    def on_measured_data(self, item):
        point = EnvironmentalDataPointModel(
            _id=gen_doc_id(),
            environment=self.env_id,
            variable=item.data_type.value,
            is_desired=False,
            value=item.value,
            timestamp=item.timestamp
        )
        point.store(self.db)
