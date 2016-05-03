from openag.brain.core import Module
from openag_pymata_aio.pymata3 import PyMata3

class Arduino(Module):
    def init(self):
        self.board = PyMata3()

    def process_request(self, fn_name, fn_args, fn_kwargs):
        return getattr(self.board, fn_name)(*fn_args, **fn_kwargs)
