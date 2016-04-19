from openag.client.core import *

class OnOffControl(Module):
    set_point = Input()
    measured = Input()
    state = Output(InternalVariable.BOOLEAN)

    def init(self, default: "The default value for the set point"):
        self.current_set_point = float(default)
        self.current_measured = float(default)

    def on_set_point(self, item):
        self.current_set_point = float(item.value)
        self.compute_state()

    def on_measured(self, item):
        self.current_measured = float(item.value)
        self.compute_state()

    def compute_state(self):
        if self.current_measured < self.current_set_point:
            self.state.emit(1)
        else:
            self.state.emit(0)
