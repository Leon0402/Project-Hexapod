#include "Hexapod.h"

int main() {

    #ifndef DEBUG
    init();
    #endif

    Hexapod hexapod;
    hexapod.test();
    return 0;
}
