import gevent
from openag.brain.core import *

class Am2315(Module):
    air_temperature = Output(EnvironmentalVariable.AIR_TEMPERATURE)
    air_humidity = Output(EnvironmentalVariable.AIR_HUMIDITY)

    def init(self, board_id: ReferenceParameter(DbName.MODULE, "ID of the "
            "module for the board to which this sensor is connected"),
            sensor_id: StringParameter("CBAN id of the sensor"),
            update_interval: IntegerParameter("Frequency with which to poll "
            "this sensor") = 1):
        self.board = self.ask(board_id)
        self.sensor_id = sensor_id
        self.update_interval = update_interval

    def run(self):
        while True:
            self.air_temperature.emit(self.board.cban_get(
                self.sensor_id, EnvironmentalVariable.AIR_TEMPERATURE.value
            ))
            self.air_humidity.emit(self.board.cban_get(
                self.sensor_id, EnvironmentalVariable.AIR_HUMIDITY.value
            ))
            gevent.sleep(self.update_interval)
