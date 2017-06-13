"""
This module consists of code for interacting with the USB HID 16-Channel
relay module.
"""

import rospy
import time
import os
import sys
import math
import numpy
import usb.core
import usb.util

# TODO: Sometimes this class misses...need to add support to handle better...

class HidRelay16:
    """HID USB SainSmart 16 Relay Module using NUC122ZC1 - AN413AD22 - 31B033 - ARM"""

    print_cfg = False

    vid = 0x0416
    pid = 0x5020

    hid = None
    ep_in = None  # From Device to Host Computer.
    ep_in_address = 0x84 # bEndpointAddress -> 0x84  EP 4 IN
    ep_out = None  # From Host Computer to Device.
    ep_out_address = 0x05 # bEndpointAddress -> 0x05  EP 5 OUT

    relayBitmap = None
    relayBitmap16 = [128, 256, 64, 512, 32, 1024, 16, 2048, 8, 4096, 4, 8192, 2, 16384, 1, 32768]
    relayBitmap8 = [128, 256, 64, 512, 32, 1024, 16, 2048, 8]

    max_relay_id = None
    max_relay_id16 = 15
    max_relay_id8 = 7

    packet_len = 64

    """Initiates de device. Ends the process if no device was found."""
    def __init__(self, version=16):
        rospy.loginfo("Initializing HidRelay16")

        # if (version is 8) or (version is "8"):
        #     self.relayBitmap = self.relayBitmap8
        #     self.max_relay_id = self.max_relay_id8
        # else:
        self.relayBitmap = self.relayBitmap16
        self.max_relay_id = self.max_relay_id16

        init_ok = self.get_device()

        return None

    """Small function to pack array of bytes"""
    def pack_bytes(self, bytesArray):
        packet = [0x0] * self.packet_len
        i = 0
        for byte in bytesArray:
            packet[i] = byte
            i += 1

        bytesString = ''.join([chr(c) for c in packet])
        return bytesString

    """Small function to unpack array of bytes"""
    def unpack_bytes(self, bytesString):
        bytesArray = bytearray()
        bytesArray.extend(bytesString)

        return bytesArray

    """Gets the devices and sets the two endpoints."""
    def get_device(self):
        rospy.loginfo("Getting device")

        # initialising debuging - don't have a clue if this will work
        self.hid = usb.core.find(idVendor=self.vid, idProduct=self.pid)

        # was it found?
        if self.hid is None:
            rospy.loginfo("Device not found")
            return False

        try:
            self.hid.detach_kernel_driver(0)
        except:  # this usually mean that kernel driver has already been dettached
            pass

        if self.print_cfg is True:
            for cfg in self.hid:
                sys.stdout.write('cfg.bConfigurationValue: ' + str(cfg.bConfigurationValue) + '\n')
                for intf in cfg:
                    sys.stdout.write('\t' + \
                                     'intf.bInterfaceNumber: ' + str(intf.bInterfaceNumber) + \
                                     ',' + \
                                     'intf.bAlternateSetting: ' + str(intf.bAlternateSetting) + \
                                     '\n')
                    for ep in intf:
                        sys.stdout.write('\t\t' + \
                                         'ep.bEndpointAddress: ' + str(ep.bEndpointAddress) + \
                                         '\n')

        # access the second configuration
        cfg = self.hid[0]
        # access the first interface
        intf = cfg[(0,0)]
        # IN endpoint
        self.ep_in = intf[0]
        # OUT endpoint
        self.ep_out = intf[1]

        # Set the first configuration
        self.hid.set_configuration()

        return True

    def read(self):

        readCmd = [0xD2, 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x48, 0x49, 0x44,
                   0x43, 0x80, 0x02, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                   0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                   0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                   0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]

        readCmd_pack = self.pack_bytes(readCmd)
        # assert len(self.hid.write(self.ep_out_address, readCmd_pack, 100)) == len(readCmd_pack)
        self.hid.write(self.ep_out_address, readCmd_pack, 100)

        data_pack = self.hid.read(self.ep_in_address, self.packet_len, 100)

        np_arr8 = numpy.uint8([data_pack[2], data_pack[3]])

        arr16 = np_arr8.view('uint16')

        mask = arr16[0]

        return mask

    def reset(self):

        resetCmd = [0x71, 0x0E, 0x71, 0x00, 0x00, 0x00, 0x11, 0x11, 0x00, 0x00, 0x48, 0x49, 0x44,
                    0x43, 0x2A, 0x02, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]

        resetCmd_pack = self.pack_bytes(resetCmd)

        self.hid.write(self.ep_out_address, resetCmd_pack, 100)

    def write(self, mask):
        if (type(mask) is not int) or (mask < 0) or (mask > 0xFFFF):
            raise ValueError('Invalid write mask: ', mask)

        writeCmd = [0xC3, 0x0E, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x49, 0x44,
                    0x43, 0xEE, 0x01, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
                    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];

        np_arr16 = numpy.uint16([mask])
        np_arr8 = np_arr16.view('uint8')

        writeCmd[2] = np_arr8[0]
        writeCmd[3] = np_arr8[1]

        size = writeCmd[1]

        writeCmd_sliced = writeCmd[0:size]
        checksum = reduce((lambda a, b: a + b), writeCmd_sliced)

        np_arr32 = numpy.uint32([checksum])
        arr8 = np_arr32.view('uint8')

        for x in range(0, 3):
            writeCmd[size + x] = arr8[x]

        writeCmd_pack = self.pack_bytes(writeCmd)
        self.hid.write(self.ep_out_address, writeCmd_pack, 100)

        return True

    def set(self, relay_id, state):
        try:
            if state:
                rospy.loginfo("Turning ON Relay {}".format(relay_id))
            else:
                rospy.loginfo("Turning OFF Relay {}".format(relay_id))

            if type(relay_id) is not int:
                raise ValueError('Invalid relay ID type: ', relay_id)

            if (relay_id < 0) or (relay_id > self.max_relay_id):
                raise ValueError('Invalid relay ID: ', relay_id)

            if type(state) is not bool:
                 raise ValueError('Invalid state: ', state)

            readmask = self.read()
            rospy.loginfo("Readmask: {}".format(readmask))

            bit = int(math.pow(2, relay_id))

            mask = 0

            isOn = False

            for x in range(0, self.max_relay_id):
                if (readmask & self.relayBitmap[x]) != 0:
                    currentRelayMask = int(math.pow(2, x))
                    mask = mask | currentRelayMask
                    if currentRelayMask == int(bit):
                        isOn = True
                    rospy.loginfo("Led {} is ON".format(x))

            rospy.loginfo("Current Mask: {}".format(mask))

            if state is True:
                mask = mask | bit
            elif state is False:
                if isOn is True:
                    mask = mask ^ bit


            rospy.loginfo("Writing Mask: {}".format(mask))
            writemask = self.write(mask)
            rospy.loginfo("Writemask: {}".format(writemask))

            new_readmask = self.read()

            rospy.loginfo("New Readmask: {}".format(new_readmask))

            return writemask

        except:
            return None


# def testRelay():
#     relay = HIDRelay(8)
#
#     relay.write(0)
#     relay.reset()
#     time.sleep(1)
#
#     relay.read()
#     time.sleep(0.25)
#     relay.write(0xFFFF)
#     time.sleep(0.25)
#     relay.write(0)
#     time.sleep(0.25)
#     relay.write(0xFFFF)
#     time.sleep(0.25)
#     relay.write(0)
#     p = relay.read()
#     time.sleep(0.5)
#
#     for x in range(0, relay.max_relay_id):
#         p = relay.set(x, True)
#         time.sleep(0.7)
#
#     for x in range(relay.max_relay_id, 0, -1):
#         p = relay.set(x, False)
#         time.sleep(0.7)
#
#     time.sleep(0.5)
#     relay.write(0xFFFF)
#     time.sleep(0.25)
#     relay.write(0)

# def mapRelay():
#     relay = HIDRelay(8)
#
#     relayBitmap = []
#
#     # Turn off all relays
#     relay.write(0)
#     relay.read()
#     time.sleep(2)
#
#     for x in range(0, relay.max_relay_id):
#         relay.write(int(math.pow(2, x)))
#         readmask = relay.read()
#         print "Readmask", readmask
#         relayBitmap.insert(x, readmask)
#         time.sleep(2)
#
#     # Clear all relays
#     relay.write(0)
#
#     print "Relay BitMap:", str(relayBitmap)
