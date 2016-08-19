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


class ArduinoHandler(object):
    def __init__(self, serial_port=None, development=False):
        self.serial_port = serial_port
        self.development = development
        self.serial_node = None
        self.build_dir = tempfile.mkdtemp()
        rospy.loginfo(
            "Initializing firmware project for arduino at %s", self.serial_port
        )
        proc = subprocess.Popen(
            ["openag", "firmware", "init"], cwd=self.build_dir,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        self.handle_process(proc, RuntimeError(
            "Failed to iniailize OpenAg firmware project"
        ))

    def __del__(self):
        if self.serial_node is not None and self.serial_node.poll():
            self.serial_node.terminate()
            self.serial_node.wait()
        if self.build_dir:
            import shutil
            shutil.rmtree(self.build_dir)

    def start(self):
        if not self.development:
            rospy.loginfo("Updating arduino at %s", self.serial_port)
            try:
                proc = subprocess.Popen(
                    [
                        "openag", "firmware", "run", "-p", "ros", "-t",
                        "upload"
                    ], cwd=self.build_dir, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.handle_process(proc, Exception())
            except Exception:
                rospy.logerr("Failed to update Arduino")
        rospy.loginfo("Starting to read from Arduino at %s", self.serial_port)
        self.serial_node = subprocess.Popen([
            "rosrun", "rosserial_python", "serial_node.py", self.serial_port
        ])

    def stop(self):
        self.serial_node.terminate()
        self.serial_node.wait()

    def restart(self):
        self.stop()
        self.start()

    def handle_process(self, proc, err):
        """
        Takes a running subprocess.Popen object `proc`, rosdebugs everything it
        prints to stdout, roswarns everything it prints to stderr, and raises
        `err` if it fails
        """
        poll = select.poll()
        poll.register(proc.stdout)
        poll.register(proc.stderr)
        while proc.poll() is None and not rospy.is_shutdown():
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
        if proc.poll():
            proc.terminate()
            proc.wait()
            raise RuntimeError("Process interrupted by ROS shutdown")
        if proc.returncode:
            raise err

if __name__ == '__main__':
    rospy.init_node("handle_arduino", anonymous=True)
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
    handler = ArduinoHandler(args.serial_port, development)
    handler.start()

    if development:
        while True:
            if rospy.is_shutdown():
                sys.exit(0)
            time.sleep(5)

    # Whenever the firmware module configuration changes, reflash the arduino
    server = Server(db_server)
    db = server[FIRMWARE_MODULE]
    last_seq = db.changes(limit=1, descending=True)['last_seq']
    while True:
        time.sleep(5)
        if rospy.is_shutdown():
            break
        changes = db.changes(since=last_seq)
        last_seq = changes['last_seq']
        if len(changes['results']):
            rospy.loginfo("Firmware module configuration changed; Restarting")
            handler.restart()
