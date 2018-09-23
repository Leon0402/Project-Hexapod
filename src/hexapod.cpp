 #include "Hexapod.h"

#include "Point.h"

#ifndef DEBUG
  #include <util/delay.h>
  #include <avr/interrupt.h>
  #include <time.h>
#else
  #include <iostream>
#endif

namespace {
  Servocontroller servocontroller1 {0x40};
  Servocontroller servocontroller2 {0x41};

  Servo servos[] = {
    // Left (Front - Middle - Rear)
    Servo {servocontroller1,  0, 105, 465}, Servo {servocontroller1,  1,  95, 472}, Servo {servocontroller1,  2, 135, 510},
    Servo {servocontroller1,  5, 130, 400}, Servo {servocontroller1,  6, 100, 475}, Servo {servocontroller1,  7,  92, 445},
    Servo {servocontroller1, 13, 100, 450}, Servo {servocontroller1, 14, 132, 492}, Servo {servocontroller1, 15, 110, 475},
    // Right(Front - Middle - Rear)
    Servo {servocontroller2, 13, 130, 440}, Servo {servocontroller2, 14,  85, 455}, Servo {servocontroller2, 15,  85, 380},
    Servo {servocontroller2,  5, 125, 415}, Servo {servocontroller2,  6,  90, 454}, Servo {servocontroller2,  7,  89, 445},
    Servo {servocontroller2,  0,  85, 445}, Servo {servocontroller2,  1, 130, 430}, Servo {servocontroller2,  2,  85, 389},
  };
}

static void initializeServoUpdateTimer() {
  #ifndef DEBUG
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
  OCR1A = 30000;
  TIMSK1 |= (1 << OCIE1A);
  #endif
}

#ifndef DEBUG
ISR(TIMER1_COMPA_vect){
  static uint8_t counter = 0;

  //servos[counter].update(time(nullptr));

  ++counter;
  if(counter > 17) {
    counter = 0;
  }
}
#endif

Hexapod::Hexapod()
: servocontroller1 {servocontroller1}, servocontroller2 {servocontroller2},
  legs {
    // Left (Front - Middle - Rear)
    Leg {  servos[0],  servos[1],  servos[2], Pointf { 11.5f,  14.5f, 0.0f}, 8.5f,  62},
    Leg {  servos[3],  servos[4],  servos[5], Pointf {  0.0f,  15.0f, 0.0f}, 6.5f,   0},
    Leg {  servos[6],  servos[7],  servos[8], Pointf {-11.5f,  14.5f, 0.0f}, 8.5f, 298},
    // Right(Front - Middle - Rear)
    Leg {  servos[9], servos[10], servos[11], Pointf { 11.5f, -14.5f, 0.0f}, 8.5f, 118},
    Leg { servos[12], servos[13], servos[14], Pointf {  0.0f, -15.0f, 0.0f}, 6.5f, 180},
    Leg { servos[15], servos[16], servos[18], Pointf {-11.5f, -14.5f, 0.0f}, 8.5f, 242}
  } {
    initializeServoUpdateTimer();
    //Initial Position
    /*
    moveLegDirectlyToPoint(LegPosition::FrontLeft,   Pointf {   8.0f,  12.0f, 0.0f }, 200);
    moveLegDirectlyToPoint(LegPosition::MiddleLeft,  Pointf {   0.0f,  14.0f, 0.0f }, 200);
    moveLegDirectlyToPoint(LegPosition::RearLeft,    Pointf { -10.0f,  12.0f, 0.0f }, 200);
    */

    //moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {   8.0f, -12.0f, 0.0f }, 0, 200);

    /*
    moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   0.0f, -14.0f, 0.0f }, 200);
    moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf { -10.0f, -12.0f, 0.0f }, 200);
    */

    /*
    Servo servo {servocontroller2, 15,  85, 380};
    servo.move();
    _delay_ms(1000);
    servo.setAngle(30);
    servo.move(1000);
    */
}

void Hexapod::test() {
  servos[11].setAngle(90);
  //this->legs[static_cast<int>(LegPosition::FrontRight)].setAngle(Joint::Tibia, 90);
  //this->legs[static_cast<int>(LegPosition::FrontRight)].move(Joint::Tibia, 0);
  servos[11].move(0);
  servos[11].update(100);
  /*
  _delay_ms(1000);
  this->legs[static_cast<int>(LegPosition::FrontRight)].setAngle(Joint::Tibia, 70);
  this->legs[static_cast<int>(LegPosition::FrontRight)].move(Joint::Tibia, 3000);
  _delay_ms(1500);
  this->legs[static_cast<int>(LegPosition::FrontRight)].setAngle(Joint::Tibia, 180);
  this->legs[static_cast<int>(LegPosition::FrontRight)].move(Joint::Tibia, 3000);
  _delay_ms(1500);*/
/*
  for(uint8_t i = 0; i < 20; i++) {
    moveLegToPoint(LegPosition::FrontLeft,   Pointf {  10.0f,  12.0f, 0.0f }, 200);
    moveLegToPoint(LegPosition::MiddleRight, Pointf {   2.0f, -14.0f, 0.0f }, 200);
    moveLegToPoint(LegPosition::RearLeft,    Pointf {  -8.0f,  12.0f, 0.0f }, 200);
    moveLegToPoint(LegPosition::FrontRight,  Pointf {  10.0f, -14.0f, 0.0f }, 200);
    moveLegToPoint(LegPosition::MiddleLeft,  Pointf {   2.0f,  12.0f, 0.0f }, 200);
    moveLegToPoint(LegPosition::RearRight,   Pointf {  -8.0f, -12.0f, 0.0f }, 200);

    moveLegDirectlyToPoint(LegPosition::FrontLeft,  Pointf {   8.0f, 12.0f, 0.0f }, 0);
    moveLegDirectlyToPoint(LegPosition::MiddleLeft, Pointf {   0.0f, 14.0f, 0.0f }, 0);
    moveLegDirectlyToPoint(LegPosition::RearLeft,   Pointf { -10.0f, 12.0f, 0.0f }, 0);

    moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {   8.0f, -12.0f, 0.0f },   0);
    moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   0.0f, -14.0f, 0.0f },   0);
    moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf { -10.0f, -12.0f, 0.0f }, 200);
  }*/
}

void Hexapod::moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time, uint16_t waitTime) {
  this->legs[static_cast<int>(legPosition)].setPosition(destination);
  this->legs[static_cast<int>(legPosition)].updateAngles();
  this->legs[static_cast<int>(legPosition)].moveAll(time);

  #ifndef DEBUG
  for(uint16_t i = 0; i < waitTime; i++) {
    _delay_ms(1);
  }
  #endif
}

void Hexapod::moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time) {
  /*
  Pointf movementPath[STEPS+1];
  this->legs[static_cast<int>(legPosition)].calculateMovementTo(movementPath, destination);

  for(uint8_t i = 0; i < STEPS+1; i++) {
    moveLegDirectlyToPoint(servocontroller, legPosition, movementPath[i], 10);

    #ifdef DEBUG
      std::cout <<  movementPath[i] << std::endl;
    #endif
  }

  #ifdef DEBUG
    for(uint16_t i = 0; i < time; i++) {
      _delay_ms(1);
    }
  #endif
  */
}
