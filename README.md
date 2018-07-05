# Project-Hexapod

Provides an API for programming a hexapod.

Current Status

  - Not usable, prealpha, see Feature List

Features

  - Control each leg through Inverse Kinematics
  - Legs move from one point to the other in a more realistic way (following a quadratic function)

Next Steps:

  - paralell movement of servos!!!
  - Add some test scripts for translation and rotation of the hexapod
  - Remove Arduino dependencies

Dependencies

  - pyserial (https://pypi.python.org/pypi/pyserial)
  - arduino libraries(https://www.arduino.cc/en/Main/Software?)
  - avr-g++
  (- g++ compiler if you want to debug it on your computer)

Requirements (Debian / Ubuntu)

  - Install all Dependencies
  $ apt install python-serial
  $ apt install avr-g++
  $ (apt install build-essential)

  - give yourself access to the ttyUSB0 port
  $ adduser <username> dialout

  - download Arduino IDE  (recommended to download the newest version from their homepage)
  $ apt install arduino

  - download Arduino Makefile (just download and unzip)
  https://github.com/sudar/Arduino-Makefile

  Configuration

  - Decide, whether you want to execute it on the arduino or debug it on your computer
  Debug: Makefile uncomment line 1, Controls.h uncomment l. 4 (you will need to have installed the g++ compiler or change the makefile, so it fits with the other compiler)
    -> DEBUG = 1
    -> #define DEBUG
  Arduino: Makefile comment out line 1, Controls.h comment out l.4
    -> #DEBUG = 1
    -> //#define DEBUG

  - Configurate Makefile (default values for the Arduino Nano connected to a Linux machine)
    Makefile l. 23: BOARD_TAG = Name of your Arduino
    Makefile l. 24: BOARD_SUB = Name of the atmega on the Arduino (Arduino Uno = atmega328p)
    Makefile l. 25: MONITOR_PORT  = USB port connected to the arduino (should be also displayed in the arduino IDE, )
    Makefile l. 26: ARDUINO_DIR = path to the Arduino IDE Folder
    Makefile l. 27: ARDMK_DIR = path to the Makefile Folder
    Makefile l. 28: AVR_TOOLS_DIR = path to the avr folder inside the arduino IDE folder
    Makefile l.29 change nothing
    Makefile l. 30: include = path to the Makefile FolderRDMK_DIR

  Usage

  --- Will be added ---
