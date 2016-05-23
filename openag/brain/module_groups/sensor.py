import gevent
from ..core import *

class SensorModule(ModuleGroup):
    def init(self, board_id: ReferenceParameter(DbName.MODULE, "ID of the "
            "module for the board to which this sensor is connected"),
            sensor_id: StringParameter("CBAN ID of the sensor"),
            update_interval: IntegerParameter("Frequency with which to poll "
            "this sensor") = 1):
        self.board = self.ask(board_id)
        self.sensor_id = sensor_id
        self.update_interval = update_interval

    def read(self):
        raise NotImplementedError()

    def run(self):
        while True:
            self.read()
            gevent.sleep(self.update_interval)
