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

def update(server):
    try:
        rospy.loginfo("Generating firmware")
        commands.generate_firmware(server)
        rospy.loginfo("Flashing Arduino")
        subprocess.call(["rosrun", "openag_brain", "flash_arduino"])
    except Exception as e:
        rospy.logerr("Failed to update Arduino: %s", e)

def handle_arduino(database, development=False):
    server = Server(database)

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
            time.sleep(10)

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
            serial_node.wait()
            update(server)
            serial_node = subprocess.Popen([
                "rosrun", "rosserial_python", "serial_node.py", "/dev/ttyACM0"
            ])

if __name__ == '__main__':
    rospy.init_node("handle_arduino")
    if rospy.has_param("/development"):
        development = rospy.get_param("/development")
        development = development == "True"
    else:
        development = False
    handle_arduino(rospy.get_param("/database"), development)
