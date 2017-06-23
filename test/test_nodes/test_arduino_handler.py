#!/usr/bin/env python

import sys, os
import unittest
import rospy

PKG = 'openag_brain'
NAME = 'test_arduino_handler'

# HACK to allow importing of functions from ros nodes.
#  Ideas to come up with a permanent solution: (Need to research the best known
#  methods)
#     1. Figure out how nodes are loaded into the default path or if they
#        are just run as subprocesses by ros
#     2. Split out functions as a separate services or move to a standard
#        library to be imported into the nodes.  This would mean the nodes do
#        nothing but run services, publish and script to topics.
DIR_NAME = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(DIR_NAME, '../../')))
from nodes.arduino_handler import process_message

class TestArduinoHandler(unittest.TestCase):
    def setUp(self):
        self.namespace = "mock_controller"
        self._received = ""

    def test_process_message(self):
        # test normal message
        message = "{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format(
        0, # status: OK
        50, # air_humidity
        25, # air_temperature
        400, # air_carbon_dioxide
        25, # water_temperature
        1, # water_level_low
        1, # water_level_high
        7, # water_potential_hydrogen
        0 # water_electrical_conductivity
        )
        self.assertEqual(process_message(message), (
            ("air_humidity", 50),
            ("air_temperature", 25),
            ("air_carbon_dioxide", 400),
            ("water_temperature", 25),
            ("water_level_low", True),
            ("water_level_high", True),
            ("water_potential_hydrogen", 7),
            ("water_electrical_conductivity", 0)
        ))

        # test bad input
        message = ""
        output = process_message(message).lower()
        self.assertTrue(("partial message") in output, output)

        message = "0,1,2,3"
        output = process_message(message).lower()
        self.assertTrue(("type conversion error") in output, output)

        message = "1,MHZ16 #1,"
        output = process_message(message).lower()
        self.assertTrue(("partial message") in output, output)

        # test weird bit
        message = 'F\xc3\xb8\xc3\xb6\xbbB\xc3\xa5r'
        output = process_message(message).lower()
        self.assertTrue(("type conversion error") in output, output)

        # test fake OK status
        message = "0,{0},{1},{2},{3},{4},{5},{6}\n".format(
        25, # air_temperature
        400, # air_carbon_dioxide
        25, # water_temperature
        1, # water_level_low
        1, # water_level_high
        7, # water_potential_hydrogen
        0 # water_electrical_conductivity
        )
        output = process_message(message)
        if not isinstance(output, tuple):
            self.assertTrue(False, "failed to process valid message")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestArduinoHandler)
