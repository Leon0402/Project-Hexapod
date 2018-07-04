# Project-Hexapod

Provides an API for programming a hexapod.

Current Status

- Not usable, prealpha, see Feature List

Features

- Control each leg through Inverse Kinematics
- Legs move from one point to the other in a more realistic way (following a quadratic function)

Next Steps:

- Add some test scripts for translation and rotation of the hexapod
- Remove Arduino dependencies

Dependencies

- pyserial (https://pypi.python.org/pypi/pyserial)
-> apt install python-serial

Requirements (Debian / Ubuntu)

- Install all Dependencies

apt install python-serial

- give yourself access to the ttyUSB0 port

adduser <username> dialout
