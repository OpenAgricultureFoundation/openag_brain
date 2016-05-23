from ..core import *
from ..module_groups.sensor import SensorModule

class Am2315(SensorModule):
    air_temperature = Output(EnvironmentalVariable.AIR_TEMPERATURE)
    air_humidity = Output(EnvironmentalVariable.AIR_HUMIDITY)

    def read(self):
        self.air_temperature.emit(self.board.cban_get(
            self.sensor_id, EnvironmentalVariable.AIR_TEMPERATURE.value
        ))
        self.air_humidity.emit(self.board.cban_get(
            self.sensor_id, EnvironmentalVariable.AIR_HUMIDITY.value
        ))
