from openag.client.core import *

class RecipeHandler(Module):
   set_points = Output()

   def run(self):
       self.set_points.emit(18, EnvironmentalVariable.AIR_TEMPERATURE)
       gevent.sleep(15)
       self.set_points.emit(30, EnvironmentalVariable.AIR_TEMPERATURE)
