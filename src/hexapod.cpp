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
: yawAngle {0}, pitchAngle {0}, rollAngle {0},
  servocontroller1 { Servocontroller {0x40}}, servocontroller2 {Servocontroller {0x41}},
  legs {
      // Left (Front - Middle - Rear)
      Leg { Servo {servocontroller1,  0, 105, 465}, Servo {servocontroller1,  1, 100, 470}, Servo {servocontroller1,  2, 110, 467}, Pointf {0.0f, 7.5f, -15.0f}, 8.5f,  62},
      Leg { Servo {servocontroller1,  5, 130, 400}, Servo {servocontroller1,  6, 100, 475}, Servo {servocontroller1,  7,  85, 445}, Pointf {0.0f, 7.5f, -15.0f}, 6.5f,   0},
      Leg { Servo {servocontroller1, 13,  85, 435}, Servo {servocontroller1, 14, 132, 492}, Servo {servocontroller1, 15, 100, 470}, Pointf {0.0f, 7.5f, -15.0f}, 8.5f, 298},
      // Right(Front - Middle - Rear)
      Leg { Servo {servocontroller2, 13, 120, 450}, Servo {servocontroller2, 14,  85, 455}, Servo {servocontroller2, 15,  95, 380}, Pointf {0.0f, 7.5f, -15.0f}, 8.5f, 118},
      Leg { Servo {servocontroller2,  5, 130, 415}, Servo {servocontroller2,  6, 100, 445}, Servo {servocontroller2,  7,  89, 460}, Pointf {0.0f, 7.5f, -15.0f}, 6.5f, 180},
      Leg { Servo {servocontroller2,  0,  85, 445}, Servo {servocontroller2,  1, 105, 458}, Servo {servocontroller2,  2,  80, 370}, Pointf {0.0f, 7.5f, -15.0f}, 8.5f, 242}
    } {
  for(uint8_t i = 0; i < 6; ++i) {
    this->legs[i].updateAngles();
    this->legs[i].moveAll();
    _delay_ms(1000);
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

void Hexapod::moveLinear(float slope, bool moveUpwards) {
  int8_t lastPhase[6] = {-1, -1, -1, -1, -1, -1};
  Pointf oldPosition[6];

  for(uint8_t cycle : waveGait.pattern) {
    for(uint8_t i = 0; i < 50; ++i) {
      for(uint8_t j = 0; j < 6; ++j) {

        uint8_t phase;
        ((cycle >> j) & 0x01) ? phase = 1 : phase = 0;

        if(phase != lastPhase[j]) {
          lastPhase[j] = phase;
          oldPosition[j] = this->legs[j].getLocalPosition();
        }

        Pointf destination {};
        Pointf nextPosition {};
        float nextStep;

        if(phase == 1) {
          destination = this->legs[j].getLastLinearPoint(slope, moveUpwards);
          nextStep = (destination.x - oldPosition[j].x)/(waveGait.swingPhaseCycles*50);

          LinearFunction linearFunction = this->legs[j].getLinearFunction(slope);
          destination.z = oldPosition[j].z;
          QuadraticFunction quadraticFunction = this->legs[j].getQuadraticFunction(destination, oldPosition[j], -7.5f);
          nextPosition.x = this->legs[j].getLocalPosition().x + nextStep;
          nextPosition.y = linearFunction.getY(nextPosition.x);
          nextPosition.z = quadraticFunction.getY(nextPosition.x);
        } else {
          destination = this->legs[j].getLastLinearPoint(slope, !moveUpwards);
          nextStep = (destination.x - oldPosition[j].x)/(waveGait.stancePhaseCycles*50);

          LinearFunction linearFunction = this->legs[j].getLinearFunction(slope);
          nextPosition.x = this->legs[j].getLocalPosition().x + nextStep;
          nextPosition.y = linearFunction.getY(nextPosition.x);
          nextPosition.z = oldPosition[j].z;
        }

        this->legs[j].setLocalPosition(nextPosition);
        this->legs[j].updateAngles();
        this->legs[j].moveAll();
      }
    }
  }
}

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
      this->legs[i].updateAngles();
      this->legs[i].moveAll();
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

/*
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
}*/

/******************************************************************************************************************************************************/
// Test scripts
/******************************************************************************************************************************************************/
void Hexapod::moveForward_test() {
  for(uint8_t i = 0; i < 4; ++i) {
     this->moveLinear(0, true);
  }
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
}
