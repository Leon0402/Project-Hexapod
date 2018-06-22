#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Controls.h"

#ifdef DEBUG
  #include <cstdint>
#else
  #include <Wire.h>
#endif

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6

class Servocontroller {
public:
    Servocontroller(uint8_t addr = 0x40);

    void begin();
    void reset();
    void setPWMFreq(float freq);
    void setPWM(uint8_t pinNum, uint16_t on, uint16_t off);

private:
    uint8_t read8(uint8_t addr);
    void write8(uint8_t addr, uint8_t d);

    const uint8_t i2caddr;

    #ifndef DEBUG
    TwoWire i2c;
    #endif
};

#endif //SERVOCONTROLLER_H
