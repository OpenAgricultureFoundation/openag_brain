"""
This module consists of code for interacting with the AM2315 temperature and
humidity sensor.
"""

import rospy
import time
import quick2wire.i2c as qI2c

class AM2315:
    """
    Class that represents a AM2315 temperature and humidity sensor instance
    and provides functions for interfacing with the sensor.
    """

    def __init__(self, i2c_addr = 0x5c, i2c_bus = 1, pseudo=False):
        self.i2c_addr = i2c_addr
        self.i2c_bus = i2c_bus
        self.pseudo = pseudo
        self.temperature = None
        self.humidity = None
        self.sensor_is_connected = True
        self.regRhMSB = 0x00
        self.regRhLSB = 0x01
        self.regTmpMSB = 0x02
        self.regTmpLSB = 0x03
        self.regModelHi = 0x08
        self.regModelLo = 0x09
        self.regVersion = 0x0a
        self.regIDA = 0x0b
        self.regIDB = 0x0c
        self.regIDD = 0x0d
        self.regIDE = 0x0e
        self.regStat = 0x0f
        self.regUsrAMSB = 0x10
        self.regUsrALSB = 0x11
        self.regUsrBMSB = 0x12
        self.regUsrBLSB = 0x13
        self.cmdReadReg = 0x03

        self.connect()

    def connect(self):
        if self.pseudo:
            rospy.loginfo('Connected to pseudo AM2315 temperature & humidity sensor')
            return
        try:
            self.__i2c = qI2c
            self.__i2cMaster = qI2c.I2CMaster(self.i2c_bus)
            if not self.sensor_is_connected:
                self.sensor_is_connected = True
                rospy.loginfo('Connected to AM2315 temperature & humidity sensor')
        except:
            if self.sensor_is_connected:
                self.sensor_is_connected = False
                rospy.logwarn('Unable to connect to AM2315 temp/humidity sensor')

    def poll(self):
        if self.pseudo:
            self.temperature = 23.1
            self.humidity = 46.6
            return
        if self.sensor_is_connected:
            try:
                self.temperature, self.humidity = self.__am2315.get_temp_humid()
            except:
                self.temperature = None
                self.humidity = None
                self.sensor_is_connected = False
        else:
            self.connect()

    def get_signed(self, unsigned):
        """
        Converts the temp reading from the AM2315 to a signed int.
        """

        signednum = 0

        # If we have the negative temp bit set
        if (unsigned & 0x8000) == 0x8000:
            # Clear the negative sign bit, and make the number negative.
           signednum = 0 - (unsigned & 0x7fff)
        else:
            signednum = unsigned

        # Return the unsigned int.
        return signednum

    def get_temp_humid(self):
        """
        get_temp_humid()

        Get the temperature and humidity from the sensor. Returns an array with two integers - temp. [0] and humidity [1]
        """

        # Track temperature data.
        tempRaw = 0
        humidRaw = 0

        # Raw temperature and humidity data.
        rawTH = 0

        # Return value
        retVal = []

        # Loop sentinel values
        failCount = 0
        notDone = True

        # Commands to get data temp and humidity data from AM2315
        thCmd = bytearray([0x00,0x04])

        # If we have failed more than twice to read the data, or have finished getting data break the loop.
        while ((failCount < 2) and notDone):

            try:
                # Request data from the sensor, using a reference to the command bytes.
                self.__i2cMaster.transaction(self.__i2c.writing_bytes(self.i2c_addr, self.cmdReadReg, *thCmd))

                # Wait for the sensor to supply data to read.
                time.sleep(0.1)

                # Now read 8 bytes from the AM2315.
                rawTH = self.__i2cMaster.transaction(self.__i2c.reading(self.i2c_addr, 8))

                # Break the string we want out of the array the transaction returns.
                rawTH = bytearray(rawTH[0])

                # Confirm the command worked by checking the response for the command we executed
                # and the number of bytes we asked for.
                if (rawTH[0] == self.cmdReadReg) and (rawTH[1] == 0x04):

                    # We're done. flag the loop to exit so we can interpret the data sent back to us.
                    notDone = False

            # We usually fail to read data 50% of the time because the sensor goes to sleep, so try twice.
            except IOError:
                if failCount > 1:
                    raise IOError("am2315 IO Error: failed to read from sensor.")
                else:
                    failCount = failCount + 1

        # And the MSB and LSB for each value together to yield our raw values.
        humidRaw = (rawTH[2] << 8) | rawTH[3]

        # Get signed int from AND'd temperature bytes.
        tempRaw = self.get_signed((rawTH[4] << 8) | rawTH[5])

        # The return data is sacled up by 10x, so compensate.
        retVal.append(tempRaw / 10.0)
        retVal.append(humidRaw / 10.0)

        return retVal
