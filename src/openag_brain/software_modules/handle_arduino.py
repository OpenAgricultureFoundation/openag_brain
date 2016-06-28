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

def update(server, serial_port):
    try:
        rospy.loginfo("Generating firmware")
        commands.generate_firmware(server)
        rospy.loginfo("Flashing Arduino")
        res = subprocess.call([
            "rosrun", "openag_brain", "flash_arduino", serial_port
        ])
        if res:
            raise Exception("Flashing failed")
    except Exception as e:
        rospy.logerr("Failed to update Arduino: %s", e)
        kill_children()
        sys.exit(1)

def handle_arduino(db_server, serial_port, development=False):
    server = Server(db_server)

    # Flash the arduino
    if not development:
        update(server, serial_port)

    # Start reading from the arduino
    print "Starting to read from Arduino"
    global serial_node
    serial_node = subprocess.Popen([
        "rosrun", "rosserial_python", "serial_node.py", serial_port
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
            update(server, serial_port)
            serial_node = subprocess.Popen([
                "rosrun", "rosserial_python", "serial_node.py", serial_port
            ])
    kill_children()

if __name__ == '__main__':
    rospy.init_node("handle_arduino")
    if rospy.has_param(params.DEVELOPMENT):
        development = rospy.get_param(params.DEVELOPMENT)
        development = development == "True"
    else:
        development = False
    parser = argparse.ArgumentParser(
        "Handles generating code for, flashing, and reading from the Arduino"
    )
    parser.add_argument("serial_port")
    args, _ = parser.parse_known_args()
    handle_arduino(
        rospy.get_param(params.DB_SERVER), args.serial_port, development
    )
