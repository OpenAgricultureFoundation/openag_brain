# Using atlas sensors over USB
### Install Raspbian Jessie on the Raspberry Pi 3

It is highly recommended to install Raspbian Jessie releases on 18th March 2016.
http://downloads.raspberrypi.org/raspbian/images/raspbian-2016-03-18/2016-03-18-raspbian-jessie.zip

### Expand file system

Expand file system by following this:
https://www.raspberrypi.org/documentation/configuration/raspi-config.md

### Update and Upgrade Packages

    sudo apt-get update
    sudo apt-get upgrade

# Download sample code.

    cd ~
    git clone https://github.com/OpenAgInitiative/openag_atlas_ph_python.git


# FTDI MODE #

FTDI mode works with Atlas Scientific's FTDI based USB to Serial devices such as the
[Electrically Isolated USB EZO™ Carrier Board](https://www.atlas-scientific.com/product_pages/components/usb-iso.html) and the [Basic USB to Serial Converter](https://www.atlas-scientific.com/product_pages/components/basic_usb.html)

### Installing dependencies for FTDI adaptors ###

- Install libftdi package.

        sudo apt-get install libftdi-dev


- Install pylibftdi python package.

        sudo pip3 install pylibftdi


- Create SYMLINK of the FTDI adaptors.
    **NOTE:** If you are using device with root permission, just skip this step.

    The following will allow ordinary users (e.g. ‘pi’ on the RPi) to access to the FTDI device without needing root permissions:

    Create udev rule file by typing `sudo nano /etc/udev/rules.d/99-libftdi.rules` and insert below:

        SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", GROUP="dialout", MODE="0660", SYMLINK+="FTDISerial_Converter_$attr{serial}"

    Press CTRL+X, Y and hit Enter to save & exit.

    Restart `udev` service to apply changes above.

        sudo service udev restart


- Modify FTDI python driver

    Since our FTDI devices use other USB PID(0x6015), we need to tweak the original FTDI Driver.

        sudo nano /usr/local/lib/python3.4/dist-packages/pylibftdi/driver.py

    Move down to the line 70 and add `0x6015` at the end of line.

    Original line:

        USB_PID_LIST = [0x6001, 0x6010, 0x6011, 0x6014]

    Added line:

        USB_PID_LIST = [0x6001, 0x6010, 0x6011, 0x6014, 0x6015]


- Testing Installation.

    Connect your device, and run the following (as a regular user):

        python3 -m pylibftdi.examples.list_devices

    If all goes well, the program should report information about each connected device.

    If no information is printed, but it is when run with sudo,
    a possibility is permissions problems - see the section under Linux above regarding udev rules.

    You may get result like this:

        FTDI:FT230X Basic UART:DA00TN73

    FTDI adaptors has its own unique serial number.

    We need this to work with our sensors.

    In the result above, serial number is `DA00TN73`.

### Using pylibftdi module for Atlas Sensors. ###

Run the sample node.

    rosrun openag_brain sensor_atlas_[ec,ph].py
