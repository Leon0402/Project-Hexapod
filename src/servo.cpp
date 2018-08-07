#include "Servo.h"

#ifdef DEBUG
  #include <cstdint>
#else
  #include <util/delay.h>
  #include <stdint.h>
#endif

/******************************************************************************************************************************************************/
//public
/******************************************************************************************************************************************************/

Servo::Servo(uint8_t pin, uint16_t servoMin, uint16_t servoMax)
: angle {0}, pin {pin}, servoMin {servoMin}, servoMax {servoMax} {}

uint16_t Servo::getOnTime() const {
  return ((this->angle * (this->servoMax - this->servoMin) * 2.0f) / ANGLEMAX + 1.0f) / 2.0f + this->servoMin;
}

uint8_t Servo::getPin() const {
  return this->pin;
}

void Servo::setAngle(uint8_t angle) {
  if(angle >= 0 && angle <= ANGLEMAX) {
    this->angle = angle;
  }
}

uint8_t Servo::getAngle() const {
  return this->angle;
}
