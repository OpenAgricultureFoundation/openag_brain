"""
This module consists of code for interacting with the MHZ16 co2 sensor.
"""

import rospy
import time
from periphery import I2C, I2CError

class MHZ16:
    """
    Class that represents a MHZ16 co2 sensor instance
    and provides functions for interfacing with the sensor.

    One instance = 1 successful i2c connection.
    """

    def __init__(self, i2c_addr = 0x4d, i2c_bus = "/dev/i2c-1"):
        self.__i2c_master = None
        self.i2c_addr = i2c_addr
        self.i2c_bus = i2c_bus
        self.co2 = None
        self.cmd_measure = [0xFF,0x01,0x9C,0x00,0x00,0x00,0x00,0x00,0x63]
        self.ppm = 0
        self.IOCONTROL = 0X0E << 3
        self.FCR = 0X02 << 3
        self.LCR = 0X03 << 3
        self.DLL = 0x00 << 3
        self.DLH = 0X01 << 3
        self.THR = 0X00 << 3
        self.RHR = 0x00 << 3
        self.TXLVL = 0X08 << 3
        self.RXLVL = 0X09 << 3

        # Attempt I2C connect.
        self.connect()

    def __del__(self):
        """
        Destructor will close the I2C manager if instance is garbage collected.
        """
        if self.__i2c_master:
            self.__i2c_master.close()

    def __enter__(self):
        """
        We use the __enter__ and exit methods to manage the lifecycle of
        i2c managers. This class can be used with the ``with`` keyword.
        See https://www.python.org/dev/peps/pep-0343/ for more on `with`.

        Example::

            with MHZ16() as mhz16:
              mhz16.poll()
        """
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        # Close the I2C manager on exit.
        if self.__i2c_master:
            self.__i2c_master.close()

    def connect(self):
        # Instantiate the periphery I2C manager. If it fails it will raise
        # an I2CError, which we then catch.
        # If we have an ``__i2c_master`` it means we have a successful
        # connection.
        # See http://python-periphery.readthedocs.io/en/latest/i2c.html
        try:
            self.__i2c_master = I2C(self.i2c_bus)
            rospy.loginfo("Connected to MHZ16")
            self.configure()
        except I2CError:
            rospy.logwarn("Failed to connect to MHZ16")
            pass

    def poll(self):
        if self.__i2c_master:
            try:
                self.co2 = self.get_co2()
            except:
                self.co2 = None
        else:
            self.connect()


    def configure(self):
        try:
            self.write_register(self.IOCONTROL, 0x08)
        except IOError:
            pass
        self.write_register(self.FCR, 0x07)
        self.write_register(self.LCR, 0x83)
        self.write_register(self.DLL, 0x60)
        self.write_register(self.DLH, 0x00)
        self.write_register(self.LCR, 0x03)

    def get_co2(self):
        try:
            # Set the FCR register
            self.write_register(self.FCR, 0x07)
            # cmd_data = bytearray((self.FCR, 0x07))
            # cmd_msgs = [I2C.Message(cmd_data)]
            # self.__i2c_master.transfer(self.i2c_addr, cmd_msgs)

            # Read TXLVL register
            msgs = [I2C.Message([self.TXLVL]), I2C.Message([0x00], read=True)]
            self.__i2c_master.transfer(self.i2c_addr, msgs)
            txlvl = msgs[1].data[0]

            # Verify TXLVL
            if txlvl >= len(self.cmd_measure):
                # Send Command Bytes to THR Register
                cmd_data = bytearray([self.THR] + self.cmd_measure)
                cmd_msgs = [I2C.Message(cmd_data)]
                self.__i2c_master.transfer(self.i2c_addr, cmd_msgs)

                # Wait for the sensor to supply data to read.
                time.sleep(0.1)

                # Now read 9 bytes from the MHZ16.
                read_data = bytearray((0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))
                read_msgs = [I2C.Message(read_data, read=True)]
                self.__i2c_master.transfer(self.i2c_addr, read_msgs)

                # Break the string we want out of the array the transaction returns.
                response = bytearray(read_msgs[0].data)

                ## Compute checksum
                checksum = 0
                for i in range (0, 9):
                    checksum += response[i]

                # Confirm the command worked by checking the response for the command we executed
                # and validating checksum
                if (response[0] == 0xFF) and (response[1] == 0x9C) and (checksum % 256 == 0xFF):
                    return (response[2]<<24) + (response[3]<<16) + (response[4]<<8) + response[5]

        except IOError:
            raise IOError("mhz16 IO Error: failed to read from sensor.")

        return None

    def write_register(self, reg_addr, val):
        time.sleep(0.001)
        cmd_data = bytearray((reg_addr, val))
        cmd_msgs = [I2C.Message(cmd_data)]
        self.__i2c_master.transfer(self.i2c_addr, cmd_msgs)
