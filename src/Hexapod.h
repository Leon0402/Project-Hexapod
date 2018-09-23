#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Servocontroller.h"
#include "Leg.h"
#include "Servo.h"

enum class LegPosition { FrontLeft, MiddleLeft, RearLeft, FrontRight, MiddleRight, RearRight};

class Hexapod {
public:
  /*!
  @brief  Instanstiates the Hexapod with it's legs and servocontrollers
  */
  Hexapod();

  void test();

  void moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time = 0, uint16_t waitTime = 0);
  void moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time);

private:
  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};
#endif //HEXAPOD_H
