# Project-Hexapod

> Provides an API for programming a hexapod

## Current Status

  - Not usable, prealpha, see Feature List

## Latest Changes

  - More typesafety by using enumerations instead of "#defines"
  -> One function to acces pin/onTime/angle instead of one function for every servo
  - test script for moving forward added

## Features

  - Control each leg through Inverse Kinematics
  - Legs follow a quadratic function when moving (up to 254 intermediate steps)
  - Performant i2c/twi library, no need for Arduino dependencies
  - Debug modus: Print out angles and points

## Next Steps

  - Add some test scripts for translation and rotation of the hexapod
  - make "#defines" in Leg.h static const members instead?
  - Add enhancements for controlling the Hexapod (Web, joystick ...)

## Dependencies

  - avr-g++
  - avrdude
  - avr-libc
  (- g++ compiler if you want to debug it on your computer)

## Requirements (Debian / Ubuntu)

  - Install all Dependencies
  -> $ sudo apt install gcc-avr
  -> $ sudo apt install avrdude
  -> $ (sudo apt install build-essential)

  - give yourself access to the ttyUSB0 port
  -> $ sudo adduser <username> dialout
  - restart session or reboot


## Configuration

  --- Will be added ---

## Usage

  --- Will be added ---

## Team

* @Leon0402 (Project manager)
* @Grifting
* @Mirkan
