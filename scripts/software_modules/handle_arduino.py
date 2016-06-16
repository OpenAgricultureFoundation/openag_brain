#!/usr/bin/env python
import time
import rospy
import atexit
import requests
import argparse
import subprocess
from couchdb import Server

from openag_brain import commands
from openag_brain.db_names import DbName

serial_node = None

@atexit.register
def kill_children():
    if serial_node is not None:
        serial_node.terminate()
        serial_node.wait()

def handle_arduino(database):
    server = Server(database)

    # Flash the arduino
    print "Generating firmware"
    commands.generate_firmware(server)
    print "Flashing arduino"
    subprocess.call(["rosrun", "openag_brain", "flash_arduino"])

    # Start reading from the arduino
    print "Starting to read from Arduino"
    global serial_node
    serial_node = subprocess.Popen([
        "rosrun", "rosserial_python", "serial_node.py", "/dev/ttyACM0"
    ])

    # Whenever the firmware module configuration changes, reflash the arduino
    last_firmware_seq = requests.get(
        database + "/{db}/_changes".format(db=DbName.FIRMWARE_MODULE)
    ).json()['last_seq']
    while True:
        time.sleep(2)
        firmware_changes = requests.get(
            database + "/{db}/_changes?last-event-id={last_seq}".format(
                db=DbName.FIRMWARE_MODULE, last_seq=last_firmware_seq
            )
        ).json()
        last_firmware_seq = firmware_changes['last_seq']
        if len(firmware_changes['results']):
            serial_node.terminate()
            print "Generating firmware"
            commands.generate_firmware(server)
            serial_node.wait()
            print "Flashing arduino"
            subprocess.call(["rosrun", "openag_brain", "flash_arduino"])
            serial_node = subprocess.Popen([
                "rosrun", "rosserial_python", "serial_node.py", "/dev/ttyACM0"
            ])

if __name__ == '__main__':
     handle_arduino(rospy.get_param("/database"))
