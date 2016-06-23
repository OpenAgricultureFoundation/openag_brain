#!/usr/bin/env python
"""
The `handle_arduino.py` modules is in charge of managing the Arduino. It
generates firmware with which to flash the Arduino using the
:ref:`OpenagCmdGenerateFirmware` command based on the configuration of firmware
modules in the database. It then flashes the arduino and spawns an instance of
`rosserial_python.serial_node.py` to read in the data from the Arduino.
Whenever the configuration of firmware modules changes, it regenerates firmware
code and reflashes the Arduino. There should always be exactly one instance of
this module in the system.
"""
import sys
import time
import rospy
import requests
import argparse
import subprocess
from couchdb import Server

from openag_brain import commands, params
from openag_brain.util import get_database_changes
from openag_brain.db_names import DbName

serial_node = None

def kill_children():
    if serial_node is not None and serial_node.poll():
        serial_node.terminate()
        serial_node.wait()

def update(server):
    try:
        rospy.loginfo("Generating firmware")
        commands.generate_firmware(server)
        rospy.loginfo("Flashing Arduino")
        subprocess.call(["rosrun", "openag_brain", "flash_arduino"])
    except Exception as e:
        rospy.logerr("Failed to update Arduino: %s", e)

def handle_arduino(db_server, development=False):
    server = Server(db_server)

    # Flash the arduino
    if not development:
        update(server)

    # Start reading from the arduino
    print "Starting to read from Arduino"
    global serial_node
    serial_node = subprocess.Popen([
        "rosrun", "rosserial_python", "serial_node.py", "/dev/ttyACM0"
    ])

    if development:
        while True:
            if rospy.is_shutdown():
                kill_children()
                sys.exit(0)
            time.sleep(5)

    # Whenever the firmware module configuration changes, reflash the arduino
    last_seq = get_database_changes(
        db_server, DbName.FIRMWARE_MODULE
    )['last_seq']
    while True:
        if rospy.is_shutdown():
            break
        time.sleep(5)
        changes = get_database_changes(
            db_server, DbName.FIRMWARE_MODULE, last_seq
        )
        last_seq = changes['last_seq']
        if len(changes['results']):
            serial_node.terminate()
            serial_node.wait()
            update(server)
            serial_node = subprocess.Popen([
                "rosrun", "rosserial_python", "serial_node.py", "/dev/ttyACM0"
            ])
    kill_children()

if __name__ == '__main__':
    rospy.init_node("handle_arduino")
    if rospy.has_param(params.DEVELOPMENT):
        development = rospy.get_param(params.DEVELOPMENT)
        development = development == "True"
    else:
        development = False
    handle_arduino(rospy.get_param(params.DB_SERVER), development)
