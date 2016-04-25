import gevent
from openag.client.core import *

class Print(Module):
    in_data = Input(InternalVariable.TEST)

    def on_in_data(self, item):
        print(item.data_type, item.value)
