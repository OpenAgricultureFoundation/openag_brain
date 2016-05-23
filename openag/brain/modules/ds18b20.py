from ..core import *
from ..module_groups.sensor import SensorModule

class Ds18b20(SensorModule):
    water_temperature = Output(EnvironmentalVariable.WATER_TEMPERATURE)

    def read(self):
        self.water_temperature.emit(self.board.cban_get(
            self.sensor_id, EnvironmentalVariable.WATER_TEMPERATURE
        ))
