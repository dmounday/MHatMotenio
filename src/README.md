MightyHat gateway unit.
Uses RFM69W/RFM69HW/RFM69CW/RFM69HCW

This will relay all RF data over serial to the host computer (RaspberryPi) and vice versa.
It will buffer the serial data to ensure host serial requests are not missed.

Mighty Hat board monitors the input power and switches to battery backup. If battery is
low a shutdown signal is sent to the RPi. A push button switch is montored to support
reboot and manual shutdown.

The power on/off state is stored in the ATMega328 EEPROM and is used to power on the RPi
when power is restored.

http://LowPowerLab.com/MightyHat
PiGateway project: http://LowPowerLab.com/gateway