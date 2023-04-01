# MightyHat gateway unit.
Uses RFM69W/RFM69HW/RFM69CW/RFM69HCW

This will relay all RF data over serial to the host computer (RaspberryPi) and vice versa.
It will buffer the serial data to ensure host serial requests are not missed.

Mighty Hat board monitors the input power and switches to battery backup. If battery is
low a shutdown signal is sent to the RPi. A push button switch is montored to support
reboot and manual shutdown.

The power on/off state is stored in the ATMega328 EEPROM and is used to power on the RPi
when power is restored.

Repo contains files for project build under VSCode with PlatformIO extensions.

Original code at:
http://LowPowerLab.com/MightyHat
PiGateway project: http://LowPowerLab.com/gateway




## Programming MightHat from Raspberry Pi
Note: that a MightyHat compatible image is published for the “Moteino Framework Gateway” already has this patch, so you don’t need to do this again. Otherwise if you are installing MightyHat on your own run this command to install the patched avrdude:

`git clone https://github.com/LowPowerLab/avrdude-rpi ~/avrdude-rpi && ~/avrdude-rpi/install`

After avrdude is patched you can upload a new sketch to the MightyHat using a command such as this:
`avrdude -c arduino -p atmega328p -P /dev/ttyAMA0 -b 115200 -U flash:w:firmware.hex`
