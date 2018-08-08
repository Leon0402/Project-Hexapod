#include "Hexapod.h"

#include "util/Point.h"

#ifndef DEBUG
  #include <util/delay.h>
#else
  #include <iostream>
#endif

Hexapod::Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2)
: servocontroller1 {servocontroller1}, servocontroller2 {servocontroller2},
  servos {
    Servo { 0, 105, 465}, Servo { 1,  95, 472}, Servo { 2, 135, 510},
    Servo { 5, 130, 400}, Servo { 6, 100, 475}, Servo { 7,  92, 445},
    Servo {13, 100, 450}, Servo {14, 132, 492}, Servo {15, 110, 475},
    Servo { 0,  85, 445}, Servo { 1, 130, 430}, Servo { 2,  85, 389},
    Servo { 5, 125, 415}, Servo { 6,  90, 454}, Servo { 7,  89, 445},
    Servo {13, 130, 440}, Servo {14,  85, 455}, Servo {15,  85, 380}
  },
  legs {
    Leg { servos[0],  servos[1],  servos[2], Pointf { 11.5f,  14.5f, 0.0f}, 8.5f,  62},
    Leg { servos[3],  servos[4],  servos[5], Pointf {  0.0f,  15.0f, 0.0f}, 6.5f,   0},
    Leg { servos[6],  servos[7],  servos[8], Pointf {-11.5f,  14.5f, 0.0f}, 8.5f, 298},
    Leg { servos[9], servos[10], servos[11], Pointf {-11.5f, -14.5f, 0.0f}, 8.5f, 242},
    Leg {servos[12], servos[13], servos[14], Pointf {  0.0f, -15.0f, 0.0f}, 6.5f, 180},
    Leg {servos[15], servos[16], servos[17], Pointf { 11.5f, -14.5f, 0.0f}, 8.5f, 118}
  } {}

void Hexapod::test() {
  moveLegDirectlyToPoint(this->servocontroller1, FRONT_LEFT, Pointf { 8.0f, 12.0f, 0.0f });
  _delay_ms(1000);
  moveLegDirectlyToPoint(this->servocontroller1, MIDDLE_LEFT, Pointf { 0.0f, 12.0f, 0.0f });
  _delay_ms(1000);
  moveLegDirectlyToPoint(this->servocontroller1, REAR_LEFT, Pointf { -10.0f, 12.0f, 0.0f });

  _delay_ms(1000);
  moveLegDirectlyToPoint(this->servocontroller2, FRONT_RIGHT, Pointf { 8.0f, -12.0f, 0.0f });
  _delay_ms(1000);
  moveLegDirectlyToPoint(this->servocontroller2, MIDDLE_RIGHT, Pointf { 0.0f, -12.0f, 0.0f });
  _delay_ms(1000);
  moveLegDirectlyToPoint(this->servocontroller2, REAR_RIGHT, Pointf { -10.0f, -12.0f, 0.0f });

  for(uint8_t i = 0; i < 3; i++) {
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller1, FRONT_LEFT, Pointf { 10.0f, 12.0f, 0.0f });
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller2, FRONT_RIGHT, Pointf { 10.0f, -12.0f, 0.0f });
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller1, MIDDLE_LEFT, Pointf { 2.0f, 12.0f, 0.0f });
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller2, MIDDLE_RIGHT, Pointf { 2.0f, -12.0f, 0.0f });
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller1, REAR_LEFT, Pointf { -7.0f, 12.0f, 0.0f });
    _delay_ms(1000);
    moveLegToPoint(this->servocontroller2, REAR_RIGHT, Pointf { -7.0f, -12.0f, 0.0f });

    _delay_ms(1000);
    this->legs[FRONT_LEFT].subCoxaAngle(20);
    servocontroller1.setPWM(this->legs[FRONT_LEFT].getCoxaPin(), 0,  this->legs[FRONT_LEFT].getCoxaOnTime());
    this->legs[FRONT_RIGHT].addCoxaAngle(20);
    servocontroller2.setPWM(this->legs[FRONT_RIGHT].getCoxaPin(), 0,  this->legs[FRONT_RIGHT].getCoxaOnTime());
    this->legs[MIDDLE_LEFT].subCoxaAngle(20);
    servocontroller1.setPWM(this->legs[MIDDLE_LEFT].getCoxaPin(), 0,  this->legs[MIDDLE_LEFT].getCoxaOnTime());
    this->legs[MIDDLE_RIGHT].addCoxaAngle(20);
    servocontroller2.setPWM(this->legs[MIDDLE_RIGHT].getCoxaPin(), 0,  this->legs[MIDDLE_RIGHT].getCoxaOnTime());
    this->legs[REAR_LEFT].subCoxaAngle(20);
    servocontroller1.setPWM(this->legs[REAR_LEFT].getCoxaPin(), 0,  this->legs[REAR_LEFT].getCoxaOnTime());
    this->legs[REAR_RIGHT].addCoxaAngle(20);
    servocontroller2.setPWM(this->legs[REAR_RIGHT].getCoxaPin(), 0,  this->legs[REAR_RIGHT].getCoxaOnTime());
    _delay_ms(1000);
  }
}

void Hexapod::moveLegDirectlyToPoint(Servocontroller& servocontroller, int legNumber, const Pointf& destination) {
  this->legs[legNumber].setPosition(destination);
  this->legs[legNumber].updateAngles();
  servocontroller.setPWM(this->legs[legNumber].getCoxaPin(), 0,  this->legs[legNumber].getCoxaOnTime());
  servocontroller.setPWM(this->legs[legNumber].getFemurPin(), 0,  this->legs[legNumber].getFemurOnTime());
  servocontroller.setPWM(this->legs[legNumber].getTibiaPin(), 0,  this->legs[legNumber].getTibiaOnTime());
}

void Hexapod::moveLegToPoint(Servocontroller& servocontroller, int legNumber, const Pointf& destination) {

  Pointf movementPath[STEPS+1];
  this->legs[legNumber].calculateMovementTo(movementPath, destination);

  for(uint8_t i = 0; i < STEPS+1; i++) {
    _delay_ms(4);
    moveLegDirectlyToPoint(servocontroller, legNumber, movementPath[i]);

    #ifdef DEBUG
      std::cout <<  movementPath[i] << std::endl;
    #endif
  }
}
