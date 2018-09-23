#include "Servo.h"

#ifdef DEBUG
  #include <cstdint>
  #include <cmath>
  #include <iostream>
#else
  #include <stdint.h>
  #include <stdlib.h>
  #include <time.h>
  #include "Stream.h"
#endif

/******************************************************************************************************************************************************/
// public
/******************************************************************************************************************************************************/

Servo::Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax)
: servocontroller {servocontroller}, pulseWidth{(servoMin+servoMax)/2}, destinationPulseWidth{pulseWidth},
  pin {pin}, servoMin {servoMin}, servoMax {servoMax}, pulseWidthRange {mapToPulseWidth(Servo::angleRange)} {}

void Servo::update(uint32_t currentMillis) {
  avr::cout << this->pin << '\t' << this->active << '\n';

  if(this->active) {
    avr::cout << "Hallo" << '\n';
    //Apply acceleration
    uint16_t velocityAddition = this->acceleration*(currentMillis - this->lastUpdate) + 0.5;

    if(this->acceleration > 0) {
      if(velocityAddition == 0) {
        velocityAddition = 1;
      }
      if(velocityAddition > (this->targetVelocity - this->velocity)) {
        this->velocity = this->targetVelocity;
      } else {
        this->velocity += velocityAddition;
      }
    } else if (this->acceleration < 0) {
      if(velocityAddition == 0) {
        velocityAddition = -1;
      }
      if(velocityAddition < (this->targetVelocity - this->velocity)) {
        this->velocity = this->targetVelocity;
      } else {
        this->velocity += velocityAddition;
      }
    }

    avr::err.println("**********");
    avr::err << this->pulseWidth << '\t' << this->destinationPulseWidth << '\n';

    //Apply velocity
    uint16_t movementAddition = this->velocity*(currentMillis - this->lastUpdate) + 0.5;

    if(this->velocity > 0) {
      if(movementAddition == 0) {
        movementAddition = 1;
      }
      if(movementAddition >= (this->destinationPulseWidth - this->pulseWidth)) {
        this->pulseWidth = this->destinationPulseWidth;
        //Movement is finished at the end of the update method, so set active to false
        this->active = false;
      } else {
        this->pulseWidth += movementAddition;
      }
    } else if(this->velocity < 0){
      if(movementAddition == 0) {
        movementAddition = -1;
      }
      if(movementAddition <= (this->destinationPulseWidth - this->pulseWidth)) {
        this->pulseWidth = this->destinationPulseWidth;
        //Movement is finished at the end of the update method, so set active to false
        this->active = false;
      } else {
        this->pulseWidth += movementAddition;
      }
    }

    avr::err.println(movementAddition);
    avr::err << this->pulseWidth << '\t' << this->destinationPulseWidth << '\n';
    avr::err.println("**********");

    //Move servo
    this->servocontroller.setPWM(this->pin, 0, this->pulseWidth);
  }
  this->lastUpdate = time(nullptr);
}

void Servo::move(float speed, float targetSpeed, float acceleration) {
  avr::cout << speed << '\t' << targetSpeed << '\t' << acceleration << '\n';
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
  avr::cout << "Pin: " << this->pin << '\t' << this->active << '\n';
}

void Servo::move(uint16_t time) {
  avr::cout << "move(time): " << this->pin << '\t' << time << '\n';

  float difference = abs(this->destinationPulseWidth - this->pulseWidth);
  if(time != 0) {
    move(difference/time, this->pulseWidthRange, 0.0f);
  } else {
    move(this->pulseWidthRange, this->pulseWidthRange, 0.0f);
  }
}

void Servo::setAngle(uint8_t angle) {
  this->destinationPulseWidth = mapToPulseWidth(angle);
}

/******************************************************************************************************************************************************/
// private
/******************************************************************************************************************************************************/

uint16_t Servo::mapToPulseWidth(uint8_t angle) {
  return ((angle * (this->servoMax - this->servoMin) * 2.0f) / Servo::angleRange + 1.0f) / 2.0f + this->servoMin;
}
