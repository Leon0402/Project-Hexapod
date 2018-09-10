#include "Servo.h"

#ifdef DEBUG
  #include <cstdint>
  #include <cmath>
  #include <iostream>
#else
  #include <stdint.h>
  #include <stdlib.h>
  #include <time.h>
#endif

/******************************************************************************************************************************************************/
// public
/******************************************************************************************************************************************************/

Servo::Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax)
: servocontroller {servocontroller}, currentPulseWidth{(servoMin+servoMax)/2},
destinationPulseWidth{currentPulseWidth}, pin {pin}, servoMin {servoMin}, servoMax {servoMax} {}

void Servo::update(uint32_t currentMillis) {
  if(this->isActive && ((currentMillis - this->lastUpdate) - this->updateInterval)) {
    this->lastUpdate = time(nullptr);

    if(this->currentPulseWidth != this->destinationPulseWidth) {

      if(this->currentPulseWidth < this->destinationPulseWidth) {
        ++this->currentPulseWidth;
      } else {
        --this->currentPulseWidth;
      }
      this->servocontroller.setPWM(this->pin, 0, this->currentPulseWidth);
    } else {
      this->isActive = true;
    }
  }
}

void Servo::move(uint16_t time) {
  float difference = abs(this->destinationPulseWidth - this->currentPulseWidth);
  this->updateInterval = time/difference;
}

void Servo::setAngle(uint8_t angle) {
  this->destinationPulseWidth = mapToPulseWidth(angle);
}

/******************************************************************************************************************************************************/
// private
/******************************************************************************************************************************************************/

uint16_t Servo::mapToPulseWidth(uint8_t angle) {
  return ((angle * (this->servoMax - this->servoMin) * 2.0f) / Servo::servoRange + 1.0f) / 2.0f + this->servoMin;
}
