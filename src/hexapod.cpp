#include "Hexapod.h"

#include "Servo.h"
#include "Point.h"

#ifndef X86_64
  #include <util/delay.h>
  #include <stdlib.h>
#else
  #include <cmath>
#endif

#include "Stream.h"

Hexapod::Hexapod()
: yawAngle {0}, pitchAngle {0}, rollAngle {0},
  servocontroller1 { Servocontroller {0x40}}, servocontroller2 {Servocontroller {0x41}},
  legs {
    // Left (Front - Middle - Rear)
    Leg { Servo {servocontroller1,  0, 105, 465}, Servo {servocontroller1,  1, 100, 470}, Servo {servocontroller1,  2, 110, 467}, Point<int16_t> {0, 75, -150}, 85,  62},
    Leg { Servo {servocontroller1,  5, 130, 400}, Servo {servocontroller1,  6, 100, 475}, Servo {servocontroller1,  7,  85, 445}, Point<int16_t> {0, 75, -150}, 65,   0},
    Leg { Servo {servocontroller1, 13,  85, 435}, Servo {servocontroller1, 14, 132, 492}, Servo {servocontroller1, 15, 100, 470}, Point<int16_t> {0, 75, -150}, 85, 298},
    // Right(Front - Middle - Rear)
    Leg { Servo {servocontroller2, 13, 120, 450}, Servo {servocontroller2, 14,  85, 455}, Servo {servocontroller2, 15,  95, 380}, Point<int16_t> {0, 75, -150}, 85, 118},
    Leg { Servo {servocontroller2,  5, 130, 415}, Servo {servocontroller2,  6, 100, 445}, Servo {servocontroller2,  7,  89, 460}, Point<int16_t> {0, 75, -150}, 65, 180},
    Leg { Servo {servocontroller2,  0,  85, 445}, Servo {servocontroller2,  1, 105, 458}, Servo {servocontroller2,  2,  80, 370}, Point<int16_t> {0, 75, -150}, 85, 242}
  } {
  for(uint8_t i = 0; i < 6; ++i) {
    this->move(static_cast<LegPosition>(i));
    _delay_ms(200);
  }
}

void Hexapod::update(uint32_t currentMillis) {
  static int counter = 0;

  this->legs[counter].update(currentMillis);

  ++counter;
  if(counter >= 6) {
    counter = 0;
  }
}

<<<<<<< HEAD
=======
template<uint8_t GAIT_PATTERN_SIZE, uint8_t GAIT_START_SEQUENZE_SIZE, uint8_t GAIT_END_SEQUENZE_SIZE>
void Hexapod::moveLinear(const Gait<GAIT_PATTERN_SIZE, GAIT_START_SEQUENZE_SIZE, GAIT_END_SEQUENZE_SIZE>& gait, float slope, bool moveUpwards, bool startSequenze, bool endSequenze) {
  uint8_t size = GAIT_PATTERN_SIZE;
  if(endSequenze) {
    size += GAIT_END_SEQUENZE_SIZE;
  }
  uint8_t pattern[size];

  for(uint8_t i = 0; i < GAIT_PATTERN_SIZE; ++i) {
    pattern[i] = gait.pattern[i];
  }

  if(startSequenze) {
    for(uint8_t i = 0; i < GAIT_START_SEQUENZE_SIZE; ++i) {
      pattern[i] = gait.startSequenze[i];
    }
  }

  if(endSequenze) {
    for(uint8_t i = 0; i < GAIT_END_SEQUENZE_SIZE; ++i) {
      pattern[GAIT_PATTERN_SIZE + i] = gait.endSequenze[i];
    }
  }

  int8_t lastPhase[6] = {-1, -1, -1, -1, -1, -1};
  float nextSteps[6] = {0};
  LinearFunction linearFunctions[6];
  static QuadraticFunction quadraticFunctions[6];

  for(uint8_t cycle : pattern) {
    for(uint8_t i = 0; i < 3; ++i) {
      for(uint8_t j = 0; j < 6; ++j) {

        uint8_t phase;
        ((cycle >> j) & 0x01) ? phase = 1 : phase = 0;

        Pointf localPosition = this->legs[j].getLocalPosition();

        if(phase != lastPhase[j]) {
          lastPhase[j] = phase;
          linearFunctions[j] = this->legs[j].getLinearFunction(slope);

          if(phase == 1) {
            Pointf destination = this->legs[j].getLastLinearPoint(slope, moveUpwards);
            nextSteps[j] = (destination.x - localPosition.x)/(gait.swingPhaseCycles*3);
            //patch, needs to be optimized
            if(localPosition.z <= -10.0f) {
              quadraticFunctions[j] = this->legs[j].getQuadraticFunction(destination, localPosition, -7.5f);
            }
          } else {
            Pointf destination = this->legs[j].getLastLinearPoint(slope, !moveUpwards);
            nextSteps[j] = (destination.x - localPosition.x)/(gait.stancePhaseCycles*3);
          }
        }

        float x = localPosition.x + nextSteps[j];
        Pointf nextPosition {x, linearFunctions[j].getY(x)};
        if(phase == 1) {
          nextPosition.z = quadraticFunctions[j].getY(x);
        } else {
          nextPosition.z = localPosition.z;
        }

        this->legs[j].setLocalPosition(nextPosition);
        this->legs[j].updateAngles();
        this->legs[j].moveAll();
      }
    }
  }
}

/*
void Hexapod::moveLinear(const Gait<uint8_t>& gait, float slope, bool moveUpwards) {
  int8_t lastPhase[6] = {-1, -1, -1, -1, -1, -1};
  float nextSteps[6] = {0};
  LinearFunction linearFunctions[6];
  QuadraticFunction quadraticFunctions[6];

  for(uint8_t cycle : gait.pattern) {
    for(uint8_t i = 0; i < 40; ++i) {
      for(uint8_t j = 0; j < 6; ++j) {

        uint8_t phase;
        ((cycle >> j) & 0x01) ? phase = 1 : phase = 0;

        Pointf localPosition = this->legs[j].getLocalPosition();

        if(phase != lastPhase[j]) {
          lastPhase[j] = phase;
          linearFunctions[j] = this->legs[j].getLinearFunction(slope);

          if(phase == 1) {
            Pointf destination = this->legs[j].getLastLinearPoint(slope, moveUpwards);
            nextSteps[j] = (destination.x - localPosition.x)/(gait.swingPhaseCycles*40);
            quadraticFunctions[j] = this->legs[j].getQuadraticFunction(destination, localPosition, -7.5f);
          } else {
            Pointf destination = this->legs[j].getLastLinearPoint(slope, !moveUpwards);
            nextSteps[j] = (destination.x - localPosition.x)/(gait.stancePhaseCycles*40);
          }
        }

        float x = localPosition.x + nextSteps[j];
        Pointf nextPosition {x, linearFunctions[j].getY(x)};
        if(phase == 1) {
          nextPosition.z = quadraticFunctions[j].getY(x);
        } else {
          nextPosition.z = localPosition.z;
        }

        this->legs[j].setLocalPosition(nextPosition);
        this->legs[j].updateAngles();
        this->legs[j].moveAll();
      }
    }
  }
}*/

>>>>>>> 9060e447d636b443dcfa18bc2e84334e92c8f3ce
void Hexapod::bodyIk(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle) {
  if(yawAngle > 45 || yawAngle < -45 || pitchAngle > 45 || pitchAngle < -45 || rollAngle > 45|| rollAngle < -45) {
    return;
  }

  int8_t yawAngleAddition;
  this->yawAngle < yawAngle ? yawAngleAddition = 1 : yawAngleAddition = -1;
  int8_t pitchAngleAddition;
  this->pitchAngle < pitchAngle ? pitchAngleAddition = 1 : pitchAngleAddition = -1;
  int8_t rollAngleAddition;
  this->rollAngle < rollAngle ? rollAngleAddition = 1 : rollAngleAddition = -1;

  do {
    if(abs(this->yawAngle - yawAngle) != 0) {
      this->yawAngle += yawAngleAddition;
    } else {
      yawAngleAddition = 0;
    }

    if(abs(this->pitchAngle - pitchAngle) != 0) {
      this->pitchAngle += pitchAngleAddition;
    } else {
      pitchAngleAddition = 0;
    }

    if(abs(this->rollAngle - rollAngle) != 0) {
      this->rollAngle += rollAngleAddition;
    } else {
      rollAngleAddition = 0;
    }

    for(uint8_t i = 0; i < 6; ++i) {
      this->legs[i].rotateXYZ(yawAngleAddition, pitchAngleAddition, rollAngleAddition);
      this->move(static_cast<LegPosition>(i));
    }
  } while(yawAngleAddition != 0 || pitchAngleAddition != 0 || rollAngleAddition != 0);
}

void Hexapod::yaw(int8_t angle) {
  this->bodyIk(angle, this->pitchAngle, this->rollAngle);
}

void Hexapod::pitch(int8_t angle) {
  this->bodyIk(this->yawAngle, angle, this->rollAngle);
}

void Hexapod::roll(int8_t angle) {
    this->bodyIk(this->yawAngle, this->pitchAngle, angle);
}

void Hexapod::moveToLocalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].setLocalPosition(destination);
  this->move(legPosition, time);
}

void Hexapod::moveToGlobalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].setGlobalPosition(destination);
  this->move(legPosition, time);
}

/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/
<<<<<<< HEAD
void Hexapod::move(const LegPosition& legPosition, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].updateAngles();
  this->legs[static_cast<uint8_t>(legPosition)].moveAll(time);
=======
void Hexapod::moveForward_test() {
  this->moveLinear(rippleGait, 0, true, true, false);

  for(uint8_t i = 0; i < 1; ++i) {
     this->moveLinear(rippleGait , 0 , true);
  }
  this->moveLinear(rippleGait, 0 , true, false, true);
}

void Hexapod::bodyIk_test() {
  this->bodyIk(15, 0, 0);
  this->bodyIk(7.5, 0, 0);
  this->bodyIk(15, 0, 0);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0, -15, 0);
  this->bodyIk(0, -7.5, 0);
  this->bodyIk(0, -15, 0);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0, 0, 15);
  this->bodyIk(0, 0, 7.5);
  this->bodyIk(0, 0, 15);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0,  15,  15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0, -15,  15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0, -15, -15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0,  15, -15);

  this->bodyIk(0, 0, 0);

  this->bodyIk(-15, 0, 0);
  this->bodyIk(-7.5, 0, 0);
  this->bodyIk(-15, 0, 0);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0, 15, 0);
  this->bodyIk(0, 7.5, 0);
  this->bodyIk(0, 15, 0);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0, 0, -15);
  this->bodyIk(0, 0, -7.5);
  this->bodyIk(0, 0, -15);

  this->bodyIk(0, 0, 0);

  this->bodyIk(0, -15, -15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0,  15, -15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0,  15,  15);
  this->bodyIk(0, 0, 0);
  this->bodyIk(0, -15,  15);
  this->bodyIk(0, 0, 0);

  this->bodyIk(0, 0, 0);
>>>>>>> 9060e447d636b443dcfa18bc2e84334e92c8f3ce
}
