import gevent
from openag.client.core import *

class Echo(Module):
    out_data = Output(TEST_TYPE)

    def run(self):
        while True:
            self.out_data.emit("test")
            gevent.sleep(1)
