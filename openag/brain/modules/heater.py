from openag.brain.core import *

class heater(Module):
    def init(self, board_id: ReferenceParameter(DbName.MODULE, "ID of the "
            "module for the board to which this heater is connected"), pin:
            IntegerParameter("The number of the pin connected to the relay "
            "for this heater")):
        self.board = self.ask(board_id)
        self.pin = pin

    @endpoint
    def set_state(self, state):
        if state == 'true':
            print("heater on")
            self.board.digital_write(self.pin, 0)
        else:
            print("heater off")
            self.board.digital_write(self.pin, 1)
