#include "Hexapod.h"

#ifndef DEBUG
#include "Arduino.h"
#endif

int main() {

    #ifndef DEBUG
    init();
    #endif

    Servocontroller servocontroller1 {};
    Servocontroller servocontroller2 {0x41};

    Hexapod hexapod {servocontroller1, servocontroller2};
    hexapod.test();

    return 0;
}
