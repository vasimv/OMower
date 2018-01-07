OMower project is open-source open-hardware wheeled robots platform with 32-bit ARM ATSAM3X8E
microcontroller, same as in Arduino Due. The project compiles under Arduino IDE (requires patching standard
ATSAM platform library files to support all GPIO lines of the MCU).

The project was inspired by ardumower project (https://github.com/Ardumower/ardumower) and uses few bits
of its code.


FEATURES

Control, measurements and navigation routines for wheeled robots (primarily lawn mowers but not limited to)
with wire perimeters, sonars and other sensors, GPS or RTK GPS (centimeters-level precision).

Support for 2 (with third support wheel) or 4 motors platforms. Brushed motors supported with Polulu Dual
MC33926 (onboard) or IHM12A1 controllers (external), brushless motors will require external ESCs (controlled
by PPM/servo output).

Includes PPM/servo output for the brushless mowing motor (or you may use onboard for brushed mowing motor)
and step-motor drivers for cutter's height adjusting.  

Supports many communication ports of ATSAM3X8E with onboard connectors to connect almost everything. 


HARDWARE

The board is based on ATSAM3X8E-AU MCU in 144-pins LQFP case (for arduino IDE it looks like Arduino Due
without "native" port).

Power subsystem includes step-down charging regulator (up to 4A charging current) and step-up for solar
battery, bi-stable relay for emergency shutdown. Has 3 switching regulators to provide 3.3V and 5V to
multiple internal and external devices, precise linear regulator for analog parts.

Supports battery voltages up to 28 volts, has onboard switching regulators for external charger and solar
panel.

Three ACS725-30 current hall sensors to measure consumption and charging currents up to 30A real-time.

Multiple connectors for everything mower or any other robot needed with 5V->3.3V logic conversion (6x HC-SR04
sonars, 5 odometers, 3 serial ports, 4 wire perimeter sensors, two bumper sensors, lawns and drops sensors,
two temperature NTC termistors connectors, two SPI ports, LEDs, button and more).

Has I2C buffers (both 3.3V and 5V versions) to connect multiple I2C devices, USB port (firmware upload),
external 12-channel ADC MAX11617 (connected to I2C).

Has port for Orange PI Zero (for RTK GPS and Wifi connectivity, firmware upload), can be replaced by any
Wifi/Bluetooth adapter connected to serial port of MCU.

Multiple PPM/servo outputs for external stuff with standard 3-pins connectors.

Note, board requires external components: external IMU GY-80 board, external motors controllers and
steppig motor driver boards. External charger voltage must be 6 volts higher than maximum battery voltage
(i.e. for 14.6V LiFePo-battery it must provide 20..34V).

Latest version is 3, in v4 version the board will include FRAM chip to store settings/waypoints/etc (as
ATSAM3X8E's flash is getting erased when uploading new firmware) and onboard IMU MPU9250.

Schematics and KiCad development files for the 4-layers board is in hardware folder. Version 3 board
variant can be ordered from oshpark: https://oshpark.com/shared_projects/ILEdXK2T


SOFTWARE

The SDK's main goal is isolate hardware stuff from higher level software and easy development of wheeled
robot platforms. It configures timers, interrupts, ADCs and other stuff to provide API to develop software
without underlying hardware knowledge. It provides basic control and navigation stuff (like moving between
points with RTK GPS), external ports (like sonars or PPM/servo stuff), measurements for ADCs (internal and
onboard MAX11617), etc.

Look omower-*.h header files for API classes, check OMower_Simple firmware to see how to use them.

OMower SDK includes patches (see *.patch files) for standard ATSAM* libraries to support all GPIO's of
ATSAM3X8E (compatible GPIO mapping with Taijuino Due Pro). Without them you won't able to use all ports of
the OMower board!

Some of OMower stuff is not compatible with other arduino libraries, check restrictions.txt file.

Decawave DWM1000 radiotags navigation support is not tested yet, requires DWM1000 tags connected to MCU with
MyRangeTag software, see https://github.com/vasimv/MyRangeTag


USAGE

Place OMower folder into ~/Arduino/libraries, choose your board in omower-defs.h (currently supported boards
are OMower v3 and Arduino Due + IHM12A1 motor controller. Remember to patch ATSAM hardware library files to
use all GPIOs of ATSAM3X8E MCU.

Clone https://github.com/vasimv/OMower_Simple and  put its folder into ~/Arduino, compile and upload. Use
pfodApp to control the robot.
