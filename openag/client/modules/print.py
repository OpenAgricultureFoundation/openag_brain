import gevent
from openag.client.core import *

class Print(Module):
    in_data = Input(TEST_TYPE)

    def on_in_data(self, item):
        print(item.value)
