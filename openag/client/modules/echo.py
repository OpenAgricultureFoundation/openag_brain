import gevent
from openag.client.core import *

class Echo(Module):
    out_data = Output(TEST_TYPE)

    def init(self, msg: "The message to echo"):
        self.msg = msg

    @endpoint
    def set_msg(self, msg: "The new message to echo"):
        """ Set the message to echo """
        self.msg = msg
        return msg

    def run(self):
        while True:
            self.out_data.emit(self.msg)
            gevent.sleep(1)
