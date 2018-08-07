# Project-Hexapod

Provides an API for programming a hexapod.

Current Status

  - Not usable, prealpha, see Feature List

Last Changed

  - We wrote our own I2c/Twi Library
  - New Makefile
  - Arduino dependencies removed
  - Give Hexapod Class control over communication with servo controller
    -> possible to control, which servos/legs move paralell or delayed
    
Features

  - Control each leg through Inverse Kinematics
  - Legs move from one point to the other in a more realistic way (following a quadratic function)

Next Steps:

  - paralell movement of servos!!!
  - Add some test scripts for translation and rotation of the hexapod

Dependencies

  - avr-g++
  - avrdude
  - avr-libc
  (- g++ compiler if you want to debug it on your computer)

Requirements (Debian / Ubuntu)

  - Install all Dependencies
  $ sudo apt install gcc-avr
  $ sudo apt install avrdude
  $ (sudo apt install build-essential)

  - give yourself access to the ttyUSB0 port
  $ sudo adduser <username> dialout
  - restart session or reboot


  Configuration

  --- Will be added ---

  Usage

  --- Will be added ---
