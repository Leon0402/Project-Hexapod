#include "Hexapod.h"

#include "Servo.h"
#include "Point.h"
#include "Gait.h"

#ifndef X86_64
  #include <util/delay.h>
  #include <math.h>
#else
  #include <cmath>
#endif

Hexapod::Hexapod()
: rollAngle {0.0f}, pitchAngle {0.0f}, yawAngle {0.0f},
  servocontroller1 { Servocontroller {0x40}}, servocontroller2 {Servocontroller {0x41}},
  legs {
    // Left (Front - Middle - Rear)
    Leg { Servo {servocontroller1,  0, 105, 465}, Servo {servocontroller1,  1,  95, 472}, Servo {servocontroller1,  2, 135, 510}, Pointf { 20.75f, 11.02f, 0.0f}, 8.5f,  62},
    Leg { Servo {servocontroller1,  5, 130, 400}, Servo {servocontroller1,  6, 100, 475}, Servo {servocontroller1,  7,  92, 445}, Pointf {  0.0f,  14.0f, 0.0f}, 6.5f,   0},
    Leg { Servo {servocontroller1, 13, 100, 450}, Servo {servocontroller1, 14, 132, 492}, Servo {servocontroller1, 15, 110, 475}, Pointf { -8.0f,  12.0f, 0.0f}, 8.5f, 298},
    // Right(Front - Middle - Rear)
    Leg { Servo {servocontroller2, 13, 130,  440}, Servo {servocontroller2, 14,  85, 455}, Servo {servocontroller2, 15,  85, 380}, Pointf {  8.0f, -12.0f, 0.0f}, 8.5f, 118},
    Leg { Servo {servocontroller2,  5, 125, 415}, Servo {servocontroller2,  6,  90, 454}, Servo {servocontroller2,  7,  89, 445}, Pointf {  0.0f, -14.0f, 0.0f}, 6.5f, 180},
    Leg { Servo {servocontroller2,  0,  85, 445}, Servo {servocontroller2,  1, 130, 430}, Servo {servocontroller2, 2,   85, 389}, Pointf {-10.0f, -12.0f, 0.0f}, 8.5f, 242}
  } {}

void Hexapod::update(uint32_t currentMillis) {
  static int counter = 0;

  this->legs[counter].update(currentMillis);

  ++counter;
  if(counter >= 6) {
    counter = 0;
  }
}

void Hexapod::moveLinear(float slope, bool moveUpwards) {
  //for(uint8_t cycle : waveGait.pattern) {
    for(uint8_t i = 1; i < 2; ++i) {
      Pointf nextPosition;

      //if((cycle >> i) & 0x01) {
        nextPosition = this->legs[i].getNextLinearPoint(slope, waveGait.swingPhaseCycles, moveUpwards);
      //} else {
        //nextPosition = this->legs[i].getNextLinearPoint(slope, waveGait.stancePhaseCycles, !moveUpwards);
      //}

      //avr::cout << nextPosition << '\n';

      this->legs[i].setLocalPosition(nextPosition);
      this->legs[i].updateAngles();
      this->legs[i].moveAll();
      _delay_ms(200);
    }
  //}
}

void Hexapod::bodyIk(float yawAngle, float pitchAngle, float rollAngle) {
  if(yawAngle > 45.0f || yawAngle < -45.0f || pitchAngle > 45.0f || pitchAngle < -45.0f || rollAngle > 45.0f || rollAngle < -45.0f) {
    return;
  }

  float yawAngleAddition;
  this->yawAngle < yawAngle ? yawAngleAddition = 0.1f : yawAngleAddition = -0.1f;
  float pitchAngleAddition;
  this->pitchAngle < pitchAngle ? pitchAngleAddition = 0.1f : pitchAngleAddition = -0.1f;
  float rollAngleAddition;
  this->rollAngle < rollAngle ? rollAngleAddition = 0.1f : rollAngleAddition = -0.1f;

  Pointf globalPositions[6];
  for(uint8_t i = 0; i < 6; ++i) {
    globalPositions[i] = this->legs[i].getGlobalPosition();
  }

  do {
    this->yawAngle += yawAngleAddition;
    this->pitchAngle += pitchAngleAddition;
    this->rollAngle += rollAngleAddition;

    for(uint8_t i = 0; i < 6; ++i) {
      Pointf globalPosition = globalPositions[i];
      globalPosition.rotateXYZ(this->yawAngle, this->pitchAngle, this->rollAngle);
      moveLegDirectlyToPoint(static_cast<LegPosition>(i), globalPosition);
    }
  } while(fabs(this->yawAngle - yawAngle) > 0.15f || fabs(this->pitchAngle - pitchAngle) > 0.15f || fabs(this->pitchAngle - pitchAngle) > 0.15f);
}

void Hexapod::roll(float angle) {
  if(angle == this->rollAngle || angle > 45.0f || angle < -45.0f) {
    return;
  }

  float rollAngleAddition;
  this->rollAngle < angle ? rollAngleAddition = 0.1f : rollAngleAddition = -0.1f;

  Pointf globalPositions[6];
  for(uint8_t i = 0; i < 6; ++i) {
    globalPositions[i] = this->legs[i].getGlobalPosition();
  }

  do {
    this->rollAngle += rollAngleAddition;

    for(uint8_t i = 0; i < 6; ++i) {
      Pointf globalPosition = globalPositions[i];
      globalPosition.rotateX(this->rollAngle);
      moveLegDirectlyToPoint(static_cast<LegPosition>(i), globalPosition);
    }
  } while(fabs(this->rollAngle - angle) > 0.1f);
}

void Hexapod::pitch(float angle) {
  if(angle > 45.0f || angle < -45.0f) {
    return;
  }

  float pitchAngleAddition;
  this->pitchAngle < angle ? pitchAngleAddition = 0.1f : pitchAngleAddition = -0.1f;

  Pointf globalPositions[6];
  for(uint8_t i = 0; i < 6; ++i) {
    globalPositions[i] = this->legs[i].getGlobalPosition();
  }

  do {
    this->pitchAngle += pitchAngleAddition;

    for(uint8_t i = 0; i < 6; ++i) {
      Pointf globalPosition = globalPositions[i];
      globalPosition.rotateY(this->rollAngle);
      moveLegDirectlyToPoint(static_cast<LegPosition>(i), globalPosition);
    }
  } while(fabs(this->pitchAngle - angle) > 0.01);
}

void Hexapod::yaw(float angle) {
  if(angle > 45.0f || angle < -45.0f) {
    return;
  }

  float yawAngleAddition;
  this->yawAngle < angle ? yawAngleAddition = 0.1f : yawAngleAddition = -0.1f;

  Pointf globalPositions[6];
  for(uint8_t i = 0; i < 6; ++i) {
    globalPositions[i] = this->legs[i].getGlobalPosition();
  }

  do {
    this->yawAngle += yawAngleAddition;

    for(uint8_t i = 0; i < 6; ++i) {
      Pointf globalPosition = globalPositions[i];
      globalPosition.rotateZ(this->yawAngle);
      moveLegDirectlyToPoint(static_cast<LegPosition>(i), globalPosition);
    }
  } while(fabs(this->yawAngle - angle) > 0.01);
}

void Hexapod::moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time) {
  this->legs[static_cast<int>(legPosition)].setGlobalPosition(destination);
  this->legs[static_cast<int>(legPosition)].updateAngles();
  this->legs[static_cast<int>(legPosition)].moveAll(time);
}

void Hexapod::moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time) {
  Pointf movementPath[50];
  this->legs[static_cast<int>(legPosition)].calculateMovementTo(destination, movementPath, 50);

  for(uint8_t i = 0; i < 50; i++) {
    moveLegDirectlyToPoint(legPosition, movementPath[i]);
    _delay_ms(10);
  }
}

/******************************************************************************************************************************************************/
// Test scripts
/******************************************************************************************************************************************************/
void Hexapod::moveForward_test() {
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
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {   8.0f, -12.0f, 0.0f });
  moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   0.0f, -14.0f, 0.0f });
  moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf { -10.0f, -12.0f, 0.0f });
  _delay_ms(200);
}

void Hexapod::startPosition_test() {
  for(uint8_t i = 0; i < 6; ++i) {
    this->legs[i].setLocalPosition(Pointf {0.0f, 7.5f, -10.0f});
    this->legs[i].updateAngles();
    this->legs[i].moveAll();
    _delay_ms(1000);
  }
}

void Hexapod::calibration_test() {
  for(uint8_t i = 0; i < 6; ++i) {
    this->legs[i].setAllAngles(90, 180, 90);
    this->legs[i].moveAll();
    _delay_ms(1000);
  }
}

void Hexapod::roll_test() {
  for(uint8_t i = 1; i <= 30; ++i) {
    moveLegDirectlyToPoint(LegPosition::FrontLeft,   Pointf {  10.0f,  12.0f, i*0.1f });
    moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   2.0f, -14.0f, i*-0.1f });
    moveLegDirectlyToPoint(LegPosition::RearLeft,    Pointf {  -8.0f,  12.0f, i*0.1f });
    _delay_ms(50);
    moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {  10.0f, -12.0f, i*-0.1f });
    moveLegDirectlyToPoint(LegPosition::MiddleLeft,  Pointf {   2.0f,  14.0f, i*0.1f });
    moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf {  -8.0f, -12.0f, i*-0.1f });
    _delay_ms(50);
  }
}

void Hexapod::pitch_test() {
  moveLegDirectlyToPoint(LegPosition::FrontLeft,  Pointf {   8.0f, 14.0f, 0.0f });
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::MiddleLeft, Pointf {   0.0f, 15.5f, 0.0f });
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::RearLeft,   Pointf { -10.0f, 14.0f, 0.0f });
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::FrontRight,  Pointf {   8.0f, -14.0f, 0.0f });
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::MiddleRight, Pointf {   0.0f, -15.5f, 0.0f });
  _delay_ms(200);
  moveLegDirectlyToPoint(LegPosition::RearRight,   Pointf { -10.0f, -14.0f, 0.0f });
  _delay_ms(200);
}
