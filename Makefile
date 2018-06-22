#DEBUG = 1

ifdef DEBUG

Hexapod: main.o hexapod.o leg.o servo.o servocontroller.o
	g++ -o Hexapod main.o hexapod.o leg.o servo.o servocontroller.o
main.o: main.cpp
	g++ -c main.cpp
hexapod.o: hexapod.cpp
	g++ -c hexapod.cpp
leg.o: leg.cpp
	g++ -c leg.cpp
servo.o: servo.cpp
	g++ -c servo.cpp
servocontroller.o: servocontroller.cpp
	g++ -c servocontroller.cpp
clean:
	rm -f main.o hexapod.o leg.o servo.o servocontroller.o
all: clean Hexapod

else

BOARD_TAG  = nano
BOARD_SUB   = atmega328
MONITOR_PORT  = /dev/ttyUSB0
ARDUINO_DIR =  /home/leon/Programme/arduino-1.8.5
ARDMK_DIR = /home/leon/Programme/arduino-1.8.5
AVR_TOOLS_DIR = /home/leon/Programme/arduino-1.8.5/hardware/tools/avr
ARDUINO_LIBS = Wire
include /home/leon/Programme/arduino-1.8.5/Arduino.mk

endif
