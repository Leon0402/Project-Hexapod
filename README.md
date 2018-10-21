# Project-Hexapod

Provides an API for programming a hexapod.

### Current Status

* alpha, see Feature List

### Latest Changes

* test script for moving forward added
* Expanded MathLibrary (LinearFunction)
* Added method (not tested) to calculate longest distance one feet can go following a linear function
  -> will be used to calculate linearMovement
* new API for different gaits (not finished)

### Features

* Control each leg through Inverse Kinematics
* Legs follow a quadratic function when moving (up to 254 intermediate steps)
* test script for moving forwars
* Performant i2c/twi library and uart library, no need for Arduino dependencies
* X86_64 modus: Compiles for x86_64 (for example to write a simulation with the calculated points)

### Next Steps

- [ ] LinearMovement can be calcuated
- [ ] Finish Gait API
- [ ] Add some test scripts for translation and rotation of the hexapod
- [ ] make "#defines" in Leg.h static const members instead?
- [ ] Add enhancements for controlling the Hexapod (Web, joystick ...)

### Dependencies

* avr-g++
* avrdude
* avr-libc
* g++ compiler (if you want to debug it on your computer)

### Requirements (Debian / Ubuntu)

* Install all Dependencies
  * $ sudo apt install gcc-avr
  * $ sudo apt install avrdude
  * $ sudo apt install avr-libc

* give yourself access to the ttyUSB0 port (if you don't have them already)
  * $ sudo adduser <username> dialout
  * restart session or reboot

### Configuration

  --- Will be added ---

### Usage

  --- Will be added ---

### Team

[@Leon0402](https://github.com/Leon0402) (Project manager)<br>
[@Grifting](https://github.com/Grifting)<br>
[@mirdoga](https://github.com/mirdoga)<br>
[@GhostJumper](https://github.com/GhostJumper)<br>
