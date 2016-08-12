#!/usr/bin/env python
"""
The `handle_arduino.py` module is in charge of managing the Arduino. It
generates firmware with which to flash the Arduino using the
:ref:`OpenagCmdGenerateFirmware` command based on the configuration of firmware
modules in the database. It then flashes the arduino and spawns an instance of
`rosserial_python.serial_node.py` to read in the data from the Arduino.
Whenever the configuration of firmware modules changes, it regenerates firmware
code and reflashes the Arduino. There should always be exactly one instance of
this module in the system.

The module reads from a command line argument `serial_port` which is the UNIX
path to the serial port to which the Arduino is connected (e.g. "/dev/ttyACM0")
"""
import sys
import time
import rospy
import shutil
import atexit
import select
import tempfile
import argparse
import traceback
import subprocess
from openag.cli.config import config as cli_config
from openag.db_names import FIRMWARE_MODULE
from couchdb import Server

from openag_brain import commands, params

global serial_node
serial_node = None
global build_dir
build_dir = None

@atexit.register
def kill_children():
    if serial_node is not None and serial_node.poll():
        serial_node.terminate()
        serial_node.wait()
    global build_dir
    if build_dir:
        shutil.rmtree(build_dir)
        build_dir = None

def handle_process(proc, err):
    """
    Takes a running subprocess.Popen object `proc`, rosdebugs everything it
    prints to stdout, roswarns everything it prints to stderr, and raises `err`
    if it fails
    """
    poll = select.poll()
    poll.register(proc.stdout)
    poll.register(proc.stderr)
    while proc.poll() is None:
        res = poll.poll(1)
        for fd, evt in res:
            if not (evt & select.POLLIN):
                continue
            if fd == proc.stdout.fileno():
                line = proc.stdout.readline().strip()
                if line:
                    rospy.logdebug(line)
            elif fd == proc.stderr.fileno():
                line = proc.stderr.readline().strip()
                if line:
                    rospy.logwarn(line)
    if proc.returncode:
        raise err

def update(serial_port):
    rospy.loginfo("Updating arduino at %s", serial_port)
    try:
        global build_dir
        proc = subprocess.Popen([
            "openag", "firmware", "run", "-p", "ros", "-t", "upload"
        ], cwd=build_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        handle_process(proc, Exception())
    except Exception:
        rospy.logerr("Failed to update Arduino")

def start_reading(serial_port):
    rospy.loginfo("Starting to read from Arduino")
    global serial_node
    serial_node = subprocess.Popen([
        "rosrun", "rosserial_python", "serial_node.py", serial_port
    ])

def handle_arduino(db_server, serial_port, development=False):
    if not development:
        update(serial_port)
    start_reading(serial_port)

    if development:
        while True:
            if rospy.is_shutdown():
                kill_children()
                sys.exit(0)
            time.sleep(5)

    # Whenever the firmware module configuration changes, reflash the arduino
    server = Server(db_server)
    db = server[FIRMWARE_MODULE]
    last_seq = db.changes()['last_seq']
    while True:
        if rospy.is_shutdown():
            break
        time.sleep(5)
        changes = db.changes(since=last_seq)
        last_seq = changes['last_seq']
        if len(changes['results']):
            rospy.loginfo("Firmware module configuration changed; Restarting")
            serial_node.terminate()
            serial_node.wait()
            update(serial_port)
            start_reading(serial_port)
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
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError(
            "No local DB server specified. Run `openag db init` to select one"
        )
    build_dir = tempfile.mkdtemp()
    rospy.loginfo("Initializing firmware project")
    proc = subprocess.Popen(
        ["openag", "firmware", "init"], cwd=build_dir, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    handle_process(proc, RuntimeError(
        "Failed to iniailize OpenAg firmware project"
    ))
    handle_arduino(db_server, args.serial_port, development)
