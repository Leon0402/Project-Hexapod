#include "Hexapod.h"

#include "util/Point.h"

#ifndef DEBUG
  #include <util/delay.h>
#else
  #include <iostream>
#endif

Hexapod::Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2)
: servocontroller1 {servocontroller1}, servocontroller2 {servocontroller2},
  legs {
    Leg { Servo { 0, 105, 465}, Servo { 1,  95, 472}, Servo { 2, 135, 510}, Pointf { 11.5f,  14.5f, 0.0f}, 8.5f,  62},
    Leg { Servo { 5, 130, 400}, Servo { 6, 100, 475}, Servo { 7,  92, 445}, Pointf {  0.0f,  15.0f, 0.0f}, 6.5f,   0},
    Leg { Servo {13, 100, 450}, Servo {14, 132, 492}, Servo {15, 110, 475}, Pointf {-11.5f,  14.5f, 0.0f}, 8.5f, 298},
    Leg { Servo { 0,  85, 445}, Servo { 1, 130, 430}, Servo { 2,  85, 389}, Pointf {-11.5f, -14.5f, 0.0f}, 8.5f, 242},
    Leg { Servo { 5, 125, 415}, Servo { 6,  90, 454}, Servo { 7,  89, 445}, Pointf {  0.0f, -15.0f, 0.0f}, 6.5f, 180},
    Leg { Servo {13, 130, 440}, Servo {14,  85, 455}, Servo {15,  85, 380}, Pointf { 11.5f, -14.5f, 0.0f}, 8.5f, 118}
  } {
    //Initial Position
    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::FrontLeft, Pointf { 8.0f, 12.0f, 0.0f });
    _delay_ms(200);
    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::MiddleLeft, Pointf { 0.0f, 14.0f, 0.0f });
    _delay_ms(200);
    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::RearLeft, Pointf { -10.0f, 12.0f, 0.0f });

    _delay_ms(200);
    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::FrontRight, Pointf { 8.0f, -12.0f, 0.0f });
    _delay_ms(200);
    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::MiddleRight, Pointf { 0.0f, -14.0f, 0.0f });
    _delay_ms(200);
    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::RearRight, Pointf { -10.0f, -12.0f, 0.0f });
  }

void Hexapod::test() {
  for(uint8_t i = 0; i < 20; i++) {
    _delay_ms(200);
    moveLegToPoint(this->servocontroller1, LegPosition::FrontLeft, Pointf { 10.0f, 12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(this->servocontroller2, LegPosition::MiddleRight, Pointf { 2.0f, -14.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(this->servocontroller1, LegPosition::RearLeft, Pointf { -7.0f, 12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(this->servocontroller2, LegPosition::FrontRight, Pointf { 10.0f, -14.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(this->servocontroller1, LegPosition::MiddleLeft, Pointf { 2.0f, 12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(this->servocontroller2, LegPosition::RearRight, Pointf { -7.0f, -12.0f, 0.0f });
    _delay_ms(200);

    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::FrontLeft, Pointf { 8.0f, 12.0f, 0.0f });
    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::MiddleLeft, Pointf { 0.0f, 14.0f, 0.0f });
    moveLegDirectlyToPoint(this->servocontroller1, LegPosition::RearLeft, Pointf { -10.0f, 12.0f, 0.0f });

    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::FrontRight, Pointf { 8.0f, -12.0f, 0.0f });
    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::MiddleRight, Pointf { 0.0f, -14.0f, 0.0f });
    moveLegDirectlyToPoint(this->servocontroller2, LegPosition::RearRight, Pointf { -10.0f, -12.0f, 0.0f });
  }
}

void Hexapod::moveLegDirectlyToPoint(Servocontroller& servocontroller, LegPosition legPosition, const Pointf& destination) {
  this->legs[static_cast<int>(legPosition)].setPosition(destination);
  this->legs[static_cast<int>(legPosition)].updateAngles();
  servocontroller.setPWM(this->legs[static_cast<int>(legPosition)].getPin(Joint::Coxa),  0,  this->legs[static_cast<int>(legPosition)].getOnTime(Joint::Coxa));
  servocontroller.setPWM(this->legs[static_cast<int>(legPosition)].getPin(Joint::Femur), 0,  this->legs[static_cast<int>(legPosition)].getOnTime(Joint::Femur));
  servocontroller.setPWM(this->legs[static_cast<int>(legPosition)].getPin(Joint::Tibia), 0,  this->legs[static_cast<int>(legPosition)].getOnTime(Joint::Tibia));
}

void Hexapod::moveLegToPoint(Servocontroller& servocontroller, LegPosition legPosition, const Pointf& destination) {

  Pointf movementPath[STEPS+1];
  this->legs[static_cast<int>(legPosition)].calculateMovementTo(movementPath, destination);

  for(uint8_t i = 0; i < STEPS+1; i++) {
    _delay_ms(10);
    moveLegDirectlyToPoint(servocontroller, legPosition, movementPath[i]);

    #ifdef DEBUG
      std::cout <<  movementPath[i] << std::endl;
    #endif
  }
}
