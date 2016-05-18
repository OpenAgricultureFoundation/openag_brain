import gevent
from openag.brain.core import *

class Ds18b20(Module):
    water_temperature = Output(EnvironmentalVariable.WATER_TEMPERATURE)

    def init(self, board_id: ReferenceParameter(DbName.MODULE, "ID of the "
            "module for the board to which this sensor is connected"), temp_id:
            StringParameter("CBAN id of the temperature point"), update_interval:
            "Frequency with which to poll this sensor" = 1):
        self.board = self.ask(board_id)
        self.temp_id = temp_id
        self.update_interval = update_interval

    def run(self):
        while True:
            self.water_temperature.emit(self.board.cban_get(self.temp_id))
            gevent.sleep(self.update_interval)
