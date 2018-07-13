#include "Hexapod.h"

int main() {
    Servocontroller servocontroller1 {};
    Servocontroller servocontroller2 {0x41};

    Hexapod hexapod {servocontroller1, servocontroller2};
    hexapod.test();

    return 0;
}
