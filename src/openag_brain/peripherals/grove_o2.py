import serial
import rospy

class GroveO2:
    """
    Class that represents a Grove O2 sensor instance and provides functions
    for interfacing with the sensor.
    """

    def __init__(self, analog_port=0, serial_port='/dev/serial/by-id/usb-Numato_Systems_Pvt._Ltd._Numato_Lab_8_Channel_USB_GPIO_Module-if00', pseudo=False):
        self.analog_port = analog_port
        self.serial_port = serial_port
        self.pseudo = pseudo
        self.o2 = None
        self.sensor_is_connected = True
        self.connect()

    def __del__(self):
        if self.serial:
            self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        if self.serial:
            self.serial.close()
        return self

    def connect(self):
        if self.pseudo:
            rospy.loginfo('Connected to pseudo sensor')
            return
        try:
            self.serial = serial.Serial(self.serial_port, 19200, timeout=1)
            rospy.logdebug("self.serial.isOpen() = {}".format(self.serial.isOpen()))
            if not self.sensor_is_connected:
                self.sensor_is_connected = True
                rospy.loginfo('Connected to sensor')
        except:
            if self.sensor_is_connected:
                self.sensor_is_connected = False
                rospy.logwarn('Unable to connect to sensor')

    def poll(self):
        if self.pseudo:
            self.o2 = 19.3
            return
        if self.sensor_is_connected:
            try:
                self.serial.write(('adc read {}\r'.format(self.analog_port)).encode())
                response = self.serial.read(25)
                voltage = float(response[10:-3]) * 5 / 1024
                if voltage == 0:
                    return
                self.o2 = voltage * 0.21 / 2.0 * 100 # percent
                rospy.logdebug('o2 = {}'.format(self.o2))
            except:
                rospy.logwarn("O2 SENSOR> Failed to read value during poll")
                self.o2 = None
                self.sensor_is_connected = False
        else:
            self.connect()
