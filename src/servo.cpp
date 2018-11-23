 #include "Servo.h"

 #ifndef X86_64
  #include <time.h>
  #include <math.h>
#else
  #include <cmath>
#endif

/******************************************************************************************************************************************************/
// public
/******************************************************************************************************************************************************/
Servo::Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax)
: servocontroller {servocontroller}, pin {pin}, servoMin {servoMin}, servoMax {servoMax} {}

void Servo::update(uint32_t currentMillis) {
  if(this->active) {
    int16_t velocityAddition;

    if(this->acceleration > 0) {
      velocityAddition = this->acceleration*(currentMillis - this->lastUpdate) + 0.5;

      if(velocityAddition == 0) {
        velocityAddition = 1;
      }
      if(velocityAddition > (this->targetVelocity - this->velocity)) {
        this->velocity = this->targetVelocity;
      } else {
        this->velocity += velocityAddition;
      }
    } else if (this->acceleration < 0) {
      velocityAddition = this->acceleration*(currentMillis - this->lastUpdate) - 0.5;

      if(velocityAddition == 0) {
        velocityAddition = -1;
      }
      if(velocityAddition < (this->targetVelocity - this->velocity)) {
        this->velocity = this->targetVelocity;
      } else {
        this->velocity += velocityAddition;
      }
    }

    int16_t movementAddition;

    if(this->velocity > 0) {
      movementAddition = this->velocity*(currentMillis - this->lastUpdate) + 0.5;

      if(movementAddition == 0) {
        movementAddition = 1;
        //"Please make ISR slower or movement faster"
      }
      if(movementAddition >= ((int16_t)this->destinationPulseWidth - (int16_t)this->pulseWidth)) {
        this->pulseWidth = this->destinationPulseWidth;
        //Movement is finished at the end of the update method, so set active to false
        this->active = false;
      } else {
        this->pulseWidth += movementAddition;
      }
    } else if(this->velocity < 0){
      movementAddition = this->velocity*(currentMillis - this->lastUpdate) - 0.5;

      if(movementAddition == 0) {
        movementAddition = -1;
        //"Please make ISR slower or movement faster"
      }
      if(movementAddition <= ((int16_t)this->destinationPulseWidth - (int16_t)this->pulseWidth)) {
        this->pulseWidth = this->destinationPulseWidth;
        //Movement is finished at the end of the update method, so set active to false
        this->active = false;
      } else {
        this->pulseWidth += movementAddition;
      }
    }
    //Move servo
    this->servocontroller.setPWM(this->pin, 0, this->pulseWidth);
  }
  #ifndef X86_64
  this->lastUpdate = time(nullptr);
  #endif
}

void Servo::move(float speed, float targetSpeed, float acceleration) {
  while(this->active);
  //If servo has already reached its destinationPulseWidth do not change status to active
  if(this->pulseWidth == this->destinationPulseWidth || speed == 0.0f) {
    return;
  }

  if(this->pulseWidth < this->destinationPulseWidth) {
    this->velocity = speed;
    this->targetVelocity = targetSpeed;
    this->acceleration = acceleration;
  } else {
    this->velocity = -speed;
    this->targetVelocity = -targetSpeed;
    this->acceleration = -acceleration;
  }
  this->active = true;

}

void Servo::move(uint16_t time) {
  float difference = fabs((float)this->destinationPulseWidth - (float)this->pulseWidth);
  uint16_t pulseWidthRange = mapToPulseWidth(Servo::angleRange);
  if(time != 0) {
    move(difference/time, pulseWidthRange, 0.0f);
  } else {
    move(pulseWidthRange, pulseWidthRange, 0.0f);
  }
}

void Servo::setAngle(uint8_t angle) {
  this->destinationPulseWidth = mapToPulseWidth(angle);
}

/******************************************************************************************************************************************************/
// private
/******************************************************************************************************************************************************/
uint16_t Servo::mapToPulseWidth(float angle) {
  return ((angle * (this->servoMax - this->servoMin) * 2.0f) / Servo::angleRange + 1.0f) / 2.0f + this->servoMin;
}
