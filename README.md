# Project-Hexapod

Provides an API for programming a hexapod.

Current Status

  - Not usable, prealpha, see Feature List
  - Arduino dependencies removed

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
  $ apt install gcc-avr
  $ apt install avrdude
  $ (apt install build-essential)

  - give yourself access to the ttyUSB0 port
  $ adduser <username> dialout


  Configuration

  Usage

  --- Will be added ---
