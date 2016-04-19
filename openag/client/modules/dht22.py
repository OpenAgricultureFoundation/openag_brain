from openag.client.core import *
import gevent

class Dht22(Module):
    temperature = Output(EnvironmentalVariable.AIR_TEMPERATURE)
    humidity = Output(EnvironmentalVariable.HUMIDITY)

    def init(self, board_id: "ID of the module for the board to which this " \
            "sensor is connected" = None, temp_id: "CBAN id of the" \
            "temperature point" = None, hum_id: "CBAN id of the humitdity " \
            "point" = None, update_interval: "Frequency with which to poll " \
            "this sensor" = 1):
        self.board = self.ask(board_id)
        self.temp_id = temp_id
        self.hum_id = hum_id
        self.update_interval = update_interval

    def run(self):
        while True:
            self.temperature.emit(self.board.cban_get(self.temp_id))
            self.humidity.emit(self.board.cban_get(self.hum_id))
            gevent.sleep(self.update_interval)
