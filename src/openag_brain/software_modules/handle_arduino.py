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

This module has
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
from openag.categories import default_categories
from couchdb import Server

from openag_brain import commands, params


class ArduinoHandler(object):
    def __init__(self, serial_port, should_flash=True, categories=default_categories):
        self.serial_node = None
        self.serial_port = serial_port
        self.categories = categories
        # If we need to flash the Arduino, create a build_dir for the source
        # and initialize a firmware project within it. We'll use this
        # directory later at self.start().
        if should_flash:
            self.build_dir = tempfile.mkdtemp()
            rospy.loginfo("Initializing firmware project for Arduino")
            proc = subprocess.Popen(
                ["openag", "firmware", "init"], cwd=self.build_dir,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.handle_process(proc, RuntimeError(
                "Failed to initialize OpenAg firmware project"
            ))
        else:
            self.build_dir = None

    def __del__(self):
        if self.serial_node is not None and self.serial_node.poll():
            self.serial_node.terminate()
            self.serial_node.wait()
        # If we have a build_dir laying around, trash it.
        if self.build_dir is not None:
            import shutil
            shutil.rmtree(self.build_dir)

    def start(self):
        # If we have a build_dir, it means we need to flash the Arduino
        if self.build_dir is not None:
            rospy.loginfo("Updating Arduino")
            command = [
                "openag", "firmware", "run", "-p", "ros", "-t", "upload"
            ]
            for category in self.categories:
                command.extend(["-c", category])
            try:
                proc = subprocess.Popen(
                    command, cwd=self.build_dir, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.handle_process(proc, Exception())
            except Exception:
                rospy.logerr("Failed to update Arduino")
        else:
            rospy.loginfo("Skipping Arduino flash (should_flash is False)")
        rospy.loginfo("Starting to read from Arduino")
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
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError(
            "No local DB server specified. Run `openag db init` to select one"
        )

    try:
        should_flash = rospy.get_param("~should_flash")
    except KeyError:
        rospy.logwarn(
            "Not specified whether Arduino should be flashed on startup. Defaulting to True."
        )
        should_flash = True
    try:
        serial_port = rospy.get_param("~serial_port")
    except KeyError:
        rospy.logwarn(
            "Serial port for arduino_handler not specified. Defaulting to /dev/ttyACM0"
        )
        serial_port = "/dev/ttyACM0"
    try:
        categories = rospy.get_param(params.CATEGORIES)
    except KeyError:
        rospy.logwarn(
            "Not specified what categories are currently enabled. Arduino will"
            "be flashed with default categories"
        )
        categories = default_categories

    handler = ArduinoHandler(
        serial_port, should_flash=should_flash, categories=categories
    )
    handler.start()

    rospy.spin()
