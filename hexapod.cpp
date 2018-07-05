#include "Hexapod.h"

#include "Point.h"

Hexapod::Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2)
: servocontroller1 {servocontroller1}, servocontroller2 {servocontroller2},
  servos {
    Servo {servocontroller1,  0, 105, 465},
    Servo {servocontroller1,  1,  95, 472},
    Servo {servocontroller1,  2, 135, 510},
    Servo {servocontroller1,  5, 130, 400},
    Servo {servocontroller1,  6, 100, 475},
    Servo {servocontroller1,  7,  80, 445},
    Servo {servocontroller1, 13, 100, 450},
    Servo {servocontroller1, 14, 132, 492},
    Servo {servocontroller1, 15, 110, 475},
    Servo {servocontroller2,  0,  85, 445},
    Servo {servocontroller2,  1, 130, 430},
    Servo {servocontroller2,  2,  85, 389},
    Servo {servocontroller2,  5, 125, 415},
    Servo {servocontroller2,  6,  90, 454},
    Servo {servocontroller2,  7,  89, 445},
    Servo {servocontroller2, 13, 130, 440},
    Servo {servocontroller2, 14,  85, 455},
    Servo {servocontroller2, 15,  85, 380}
  },
  legs {
    Leg { servos[0],  servos[1],  servos[2], 8.5f, 62,  Pointf {15,8,0}},
    Leg { servos[3],  servos[4],  servos[5], 6.5f, 0,   Pointf {0,14,0}},
    Leg { servos[6],  servos[7],  servos[8], 8.5f, 298, Pointf {-15,8,0}},
    Leg { servos[9], servos[10], servos[11], 8.5f, 242, Pointf {-15,-8,0}},
    Leg {servos[12], servos[13], servos[14], 6.5f, 180, Pointf {0,-14,0}},
    Leg {servos[15], servos[16], servos[17], 8.5f, 118, Pointf {15,-8,0}}
  } {}


void Hexapod::test() {
  //rotation
  /*
  this->legs[FRONT_LEFT].moveTo(Pointf(13.68,7.29,0));
  this->legs[MIDDLE_LEFT].moveTo(Pointf(0,13.5,0));
  this->legs[REAR_LEFT].moveTo(Pointf(-13.68,7.29,0));

  this->legs[REAR_RIGHT].moveTo(Pointf(-13.86,-7.29,0));
  this->legs[MIDDLE_RIGHT].moveTo(Pointf(0,-13.5,0));
  this->legs[FRONT_RIGHT].moveTo(Pointf(13.68,-7.29,0));

  this->legs[FRONT_LEFT].moveTo(Pointf(14.5,4,0));
  this->legs[MIDDLE_LEFT].moveTo(Pointf(4.12,12.16,0));
  this->legs[REAR_LEFT].moveTo(Pointf(-7.5,11,0));

  this->legs[REAR_RIGHT].moveTo(Pointf(-14.5,-4,0));
  this->legs[MIDDLE_RIGHT].moveTo(Pointf(-4.12,-12.16,0));
  this->legs[FRONT_RIGHT].moveTo(Pointf(7.5,-11,0));
  */
}
