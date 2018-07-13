#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Servocontroller.h"
#include "Leg.h"
#include "Servo.h"

#define FRONT_LEFT 0
#define MIDDLE_LEFT 1
#define REAR_LEFT 2
#define REAR_RIGHT 3
#define MIDDLE_RIGHT 4
#define FRONT_RIGHT 5

class Hexapod {
public:
  /*!
  @brief  Instanstiates the Hexapod with it's legs and servocontrollers
  @param  servocontroller1 the three left legs are connected to this servocontroller
  @param  conServocontroller the three right legs are connected to this servocontroller
  */
  Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2);

  void test();

private:
  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Servo servos[18];
  Leg legs[6];
};
#endif //HEXAPOD_H
