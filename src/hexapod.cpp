#include "Hexapod.h"

#include "Servo.h"
#include "Point.h"

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
        this->move(static_cast<LegPosition>(i));
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

void Hexapod::moveToLocalPoint(const LegPosition& legPosition, const Pointf& destination, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].setLocalPosition(destination);
  this->move(legPosition, time);
}

void Hexapod::moveToGlobalPoint(const LegPosition& legPosition, const Pointf& destination, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].setGlobalPosition(destination);
  this->move(legPosition, time);
}

/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/
void Hexapod::move(const LegPosition& legPosition, uint16_t time) {
  this->legs[static_cast<uint8_t>(legPosition)].updateAngles();
  this->legs[static_cast<uint8_t>(legPosition)].moveAll(time);
}
