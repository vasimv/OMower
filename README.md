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

It may operate with pure pfodApp (or Modbus) user interface but has limited support for Robot OS (ROS).


HARDWARE

The board is based on ATSAM3X8E-AU MCU in 144-pins LQFP case (for arduino IDE it looks like Arduino Due
without "programming" port).

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

Note, board requires external components: external IMU GY-80 board or MPU-9250 board, external motors
controllers and steppig motor driver boards. External charger voltage must be 6 volts higher than
maximum battery voltage (i.e. for 14.6V LiFePo-battery it must provide 20..34V).

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


ROS support

At this moment, ROS support is limited and includes sending some of sensors data to ROS (with *MultiArray
messages but i planning to convert them to standartized message types) and routing debug and user
interface to ROS topics. See OMower_Simple firmware example on how to enable ROS in your firmware.


HOW IT WORKS

OMower software must define all needed objects, set pointers to other objects where it needed (like
gps object needs to set pointer to imu object), call all begin() functions of objects (hardware init, called
just once), then sets values for objects parameters and call init() functions (software init, can be
called many times to reset the object's hardware), define poll10/20/50 hooks functions to call objects
poll*() functions from there (see OMower_Simple.ino for example).

One of most important classes is motors object (see omower-motors.h), it controls motor drivers from its
poll10() function. User just calls functions roll/move (to simple turning or driving in a specific
direction) or rollCourse/MoveCourse to drive with navigation by something (compass, gps, etc). 

IMU stuff (compass, gyro, accelerometer) are read in imu object (omower-imu.h) by its poll50() function.
It does filtering/calculation and provide angle values for compass and pitch/yaw/roll in degrees
(see omower-imu.h). The imu object is daughter class of navThing class (see omower-root.h) that means
it has readCourseError() function, so it can be used for navigation by motors object. Currently it supports
navigation by compass angle.

GPS object is daughter class of navThing too, so it supports navigation. All coordinates are stored in
int32_t values (so, 18.3495,-68.5960 will be 183495000,-685960000 actually) to improve precision (otherwise
it would need large float class which does not exists in arduino framework). The user software must
interface with a GPS receiver and feed NMEA lines from it to gps::parseString() function or raw
coordinates to gps::settCoords() (see omower-gps.h). In precision mode (when centimeters-precise
coordinates are used) - the gps object calculates robot's true coordinates using offset of GPS's antenna
(distance from robot's center and angle).

Power object (omower-power.h) handles power stuff (battery, solar panel and charger regulators) and
provides human-readable values of voltages and currents.

Mow object (omower-mow.h) manages the mowing motor(s) and its actuator (if have) to adjust cutting height.
Note, currently it supports only PPM regulation on PWM-E output (brushless motors ESC connected to it), but
will support brushed motors too (in 2 wheel motors + mow motor configuration).

Perimeter object (omower-perimeter.h) supports up to 4 wire perimeter sensors (same as for ardumower, just
inductor+analog amplifier).

Serial object (omower-serial.h) handles communication with serial ports (as you can't use arduino's
objects for that because possible interrupts conflicts). It has also better printf() functions which
supports %f/%g for floats. UART (Serial.*) interface works with DMA (through built-in PDC) with huge
output buffer (2048 bytes, can be changed in omower-debug.h), so debug output function won't cause
slow-downs.

Sonars object (omower-sonars.h) handles HC-SR04 sonars (up to 6). It returns sonar's reported distance
(in centimeters) through its readSonar() function. Since motors routines do not check for any obstacles
during movement (except overcurrent on motors drivers), the user software must check sonars and bumpers
to avoid obstacles and move around them.

At start of initialization chassis object (omower-chassis.h) initializes interrupts. This is most important
task as almost all of OMower's stuff is done through interrupts hooks. It configures TIM7 to generate
interrupts 100 times per second.

At every TIM7 interrupt, it reads 12 channels of MAX11617 ADC to "uint16_t adc11617Arr[]"
(defined in max11617-adc-scan.h), this array is accessed from other OMower's objects (like power object, but
of course you can read its values directly).

Every second cycle of TIM7 interrupt it calls user's poll50() hook, at every fifth - poll20() hook, at
every tenth - poll10(). User's software must define these functions, set it through
chassis::setHooksPoll() function and call all other objects poll10/20/50 functions from there. By default,
to call hook functions it creates software interrupts for each kind of them (defined symbol
USE_SOFTWARE_INTERRUPTS in omower-defs-omowerv3.h), so they are running asynchronous. Without
USE_SOFTWARE_INTERRUPTS - poll50/20/10 hooks called after each other and you are limited to 20ms total time
for all of them (OMower SDK functions were optimized to comply with that time and software interrupts
can be turned off but if you place some other CPU hungry stuff in it - you will need software interrupts).

It was possible to create automatic binding for poll functions (so user wouldn't need to call all
objects poll*() functions from firmware code), but it was decided not to do so to simplify the SDK
internals.

All internal ADC channels of ATSAM3X8E are read by separate TIM5 interrupts 38461 times per second to the
"adcArr[]" array (see due-adc-scan.h for its structure). This array is accessed by other omower objects
from their poll10/20/50 functions, so no need to read it directly (but you still can do). Note, it is
possible to add circle buffer to every channel read, so you won't miss any of value (this method is used
in wire perimeter class for FFT filtering of the perimeter signal) through adcAddChannel()
function (due-adc-scan.h).

PWM and TCC (in PWM mode) outputs are configured through pwmServo object (omower-pwmservo.h), don't use
other methods to modify their output as it may conflict with OMower code. Since the code uses some
MCU's timers, you won't able to use these for other things (Timer0, Timer6, Timer8 in pwmServo, Timer5 in
due-adc-scan.cpp, Timer7 in chassis). Usually, nothing uses them but some of arduino libraries do (like
Servo library, which you wouldn't need as there is pwmServo object).

Debug function debug() is defined in omower-debug.h. By default, it sends its output to Serial device, has
ability to set priority for debug messages (so low-priority stuff won't be send at all).

There are some other objects which are easy to understand (omower-rtc.h for RTC clock, omower-current*.h for
current sensors, omower-odometry.h for odometry sensors), please, look their header files and OMower_Simple
firmware code if you want to know how they are used.
