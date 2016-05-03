import gevent
from openag.brain.core import *

class Echo(Module):
    out_data = Output(InternalVariable.STRING)

    def init(self, msg: StringParameter("The message to echo")):
        self.msg = msg

    @endpoint
    def set_msg(self, msg: StringParameter("The new message to echo")):
        """ Set the message to echo """
        self.msg = msg
        return msg

    def run(self):
        while True:
            self.out_data.emit(self.msg)
            gevent.sleep(1)
