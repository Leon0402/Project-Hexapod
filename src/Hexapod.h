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
  @param  servocontroller1 the three left legs are connected to this servocontroller
  @param  conServocontroller the three right legs are connected to this servocontroller
  */
  Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2);

  void test();

  void moveLegDirectlyToPoint(Servocontroller& servocontroller, LegPosition legPosition, const Pointf& destination);
  void moveLegToPoint(Servocontroller& servocontroller, LegPosition legPosition, const Pointf& destination);

private:
  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Servo servos[18];
  Leg legs[6];
};
#endif //HEXAPOD_H
