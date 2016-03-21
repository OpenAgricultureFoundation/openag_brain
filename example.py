#!/usr/bin/env python3
import gevent
from openag.modules import *

class Src(Module):
    output = SimpleOutput(data_type='test')
    def init(self, val: "The value to output"):
        self.val = val

    def run(self):
        while True:
            self.output.emit(self.val)
            gevent.sleep(1)

class Dest(Module):
    input = SimpleInput(data_type='test')
    def init(self):
        self.input.add_callback(self.on_input)

    def on_input(self, value):
        print(value)

src = Src(1, val='test')
dest = Dest(2)
src.output.output_to(dest.input)
threads = []
threads.append(src.start())
threads.append(dest.start())
gevent.joinall(threads)
