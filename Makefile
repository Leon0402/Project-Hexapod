#DEBUG = 1

ifdef DEBUG
run: clean Hexapod
	./Hexapod

Hexapod: main.o leg.o servocontroller.o
	g++ -o Hexapod main.o leg.o servocontroller.o
main.o: main.cpp
	g++ -c main.cpp
leg.o: leg.cpp
	g++ -c leg.cpp
servocontroller.o: servocontroller.cpp
	g++ -c servocontroller.cpp
clean:
	rm -f main.o leg.o servocontroller.o

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
