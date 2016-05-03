from openag.brain.core import *

class Lasko30(Module):
    state = Input(InternalVariable.BOOLEAN)

    def init(self, board_id: ReferenceParameter(DbName.MODULE, "ID of the "
            "module for the board to which this heater is connected"), pin:
            IntegerParameter("The number of the pin connected to the relay "
            "for this heater")):
        self.board = self.ask(board_id)
        self.pin = pin

    def on_state(self, item):
        if item.value:
            self.board.digital_write(self.pin, 0)
        else:
            self.board.digital_write(self.pin, 1)
