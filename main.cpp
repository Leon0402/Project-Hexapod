#include "Hexapod.h"

int main() {
    #ifndef DEBUG
    init();
    #endif
    Hexapod hexapod;
    hexapod.initServo();
    return 0;
}
