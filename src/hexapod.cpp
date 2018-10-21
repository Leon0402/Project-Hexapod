#include "Hexapod.h"

#include "Servo.h"
#include "Point.h"
#include "Gait.h"

#ifndef X86_64
  #include <util/delay.h>
#endif

Hexapod::Hexapod()
: servocontroller1 { Servocontroller {0x40}}, servocontroller2 {Servocontroller {0x41}},
  legs {
    // Left (Front - Middle - Rear)
    Leg { Servo {servocontroller1,  0, 105, 465}, Servo {servocontroller1,  1,  95, 472}, Servo {servocontroller1,  2, 135, 510}, Pointf { 11.5f,  14.5f, 0.0f}, 8.5f,  62},
    Leg { Servo {servocontroller1,  5, 130, 400}, Servo {servocontroller1,  6, 100, 475}, Servo {servocontroller1,  7,  92, 445}, Pointf {  0.0f,  15.0f, 0.0f}, 6.5f,   0},
    Leg { Servo {servocontroller1, 13, 100, 450}, Servo {servocontroller1, 14, 132, 492}, Servo {servocontroller1, 15, 110, 475}, Pointf {-11.5f,  14.5f, 0.0f}, 8.5f, 298},
    // Right(Front - Middle - Rear)
    Leg { Servo {servocontroller2, 13, 130, 440}, Servo {servocontroller2, 14,  85, 455}, Servo {servocontroller2, 15,  85, 380}, Pointf { 11.5f, -14.5f, 0.0f}, 8.5f, 118},
    Leg { Servo {servocontroller2,  5, 125, 415}, Servo {servocontroller2,  6,  90, 454}, Servo {servocontroller2,  7,  89, 445}, Pointf {  0.0f, -15.0f, 0.0f}, 6.5f, 180},
    Leg { Servo {servocontroller2,  0,  85, 445}, Servo {servocontroller2,  1, 130, 430}, Servo {servocontroller2, 2,   85, 389}, Pointf {-11.5f, -14.5f, 0.0f}, 8.5f, 242}
  } {}

void Hexapod::update(uint32_t currentMillis) {
  static int counter = 0;

  this->legs[counter].update(currentMillis);

  ++counter;
  if(counter >= 6) {
    counter = 0;
  }
}

void Hexapod::test() {
  for(uint8_t i = 0; i < 20; i++) {
    moveLegToPoint(LegPosition::FrontLeft,   Pointf {  10.0f,  12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(LegPosition::MiddleRight, Pointf {   2.0f, -14.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(LegPosition::RearLeft,    Pointf {  -8.0f,  12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(LegPosition::FrontRight,  Pointf {  10.0f, -12.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(LegPosition::MiddleLeft,  Pointf {   2.0f,  14.0f, 0.0f });
    _delay_ms(200);
    moveLegToPoint(LegPosition::RearRight,   Pointf {  -8.0f, -12.0f, 0.0f });
    _delay_ms(200);

    moveLegDirectlyToPoint(LegPosition::FrontLeft,  Pointf {   8.0f, 12.0f, 0.0f });
    moveLegDirectlyToPoint(LegPosition::MiddleLeft, Pointf {   0.0f, 14.0f, 0.0f });
    moveLegDirectlyToPoint(LegPosition::RearLeft,   Pointf { -10.0f, 12.0f, 0.0f });

    moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {   8.0f, -12.0f, 0.0f });
    moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   0.0f, -14.0f, 0.0f });
    moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf { -10.0f, -12.0f, 0.0f });
    _delay_ms(200);
  }
}

void Hexapod::moveLinear(float slope, bool moveUpwards) {
  for(uint8_t cycle : waveGait.pattern) {
    for(uint8_t i = 0; i < 6; ++i) {
      if((cycle << i) & 0x01) {
        //Swing phase: Move Leg upwards to the destination. If swingPhaseCyles > 1, move only one-half, one-third ... of the route
      } else {
        //Stance phase: Move Leg backwards to the origin. If stancePhaseCyles > 1, move only one-half, one-third ... of the route
      }
      _delay_ms(200);
    }
  }
}

void Hexapod::moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time) {
  this->legs[static_cast<int>(legPosition)].setPosition(destination);
  this->legs[static_cast<int>(legPosition)].updateAngles();
  this->legs[static_cast<int>(legPosition)].moveAll(time);
}

void Hexapod::moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time) {
  Pointf movementPath[STEPS+1];
  this->legs[static_cast<int>(legPosition)].calculateMovementTo(destination, movementPath);

  for(uint8_t i = 0; i < STEPS+1; i++) {
    moveLegDirectlyToPoint(legPosition, movementPath[i]);
    _delay_ms(10);
  }
}
