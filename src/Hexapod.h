#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Servocontroller.h"
#include "Leg.h"
#include "Gait.h"

enum class LegPosition { FrontLeft, MiddleLeft, RearLeft, FrontRight, MiddleRight, RearRight};

class Hexapod {
public:
  /*!
  @brief  Instanstiates the Hexapod with it's legs and servocontrollers
  */
  Hexapod();

  /*!
  @brief Updates all legs
  @param currentMillis time passed in milliseconds
  */
  void update(uint32_t currentMillis);

  /*!
  @brief Moves Linear
  */
  template<uint8_t gaitStartSequenceSize, uint8_t gaitPatternSize, uint8_t gaitEndSequenceSize>
  void moveLinear(const Gait<gaitStartSequenceSize, gaitPatternSize, gaitEndSequenceSize>& gait, float slope, bool moveUpwards, bool startSequenze = false, bool endSequenze = false);

  /*
  @brief Complete body rotation
  @param yawAngle The yaw angle
  @param pitchAngle The pitch angle
  @param rollAngle The roll angle
  */
  void bodyIk(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle);

  /*
  @brief Body movement: yaw
  @param angle The yaw angle
  */
  void yaw(int8_t angle);

  /*!
  @brief Body movement: pitch
  @param angle The pitch angle
  */
  void pitch(int8_t angle);

  /*!
  @brief Body movement: roll
  @param angle The roll angle
  */
  void roll(int8_t angle);

  /*!
  @brief Moves a leg to a point in its local coordinate system in the given time
  @param legPosition Position of the leg, which should move (see LegPosition Enum)
  @param destination Point in the local coordinate system of the leg the leg should move to
  @param time The time the leg needs to move
  */
  void moveToLocalPoint(const LegPosition& legPosition, const Pointf& destination, uint16_t time = 0);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time
  @param legPosition Position of the leg, which should move (see LegPosition Enum)
  @param destination Point in the global coordinate system the leg should move to
  @param time The time the leg needs to move
  */
  void moveToGlobalPoint(const LegPosition& legPosition, const Pointf& destination, uint16_t time = 0);

private:
  void move(const LegPosition& legPosition, uint16_t time = 0);

  int8_t yawAngle;
  int8_t pitchAngle;
  int8_t rollAngle;

  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};

template<uint8_t gaitStartSequenceSize, uint8_t gaitPatternSize, uint8_t gaitEndSequenceSize>
void Hexapod::moveLinear(const Gait<gaitStartSequenceSize, gaitPatternSize, gaitEndSequenceSize>& gait, float slope, bool moveUpwards, bool startSequenze, bool endSequenze) {
  constexpr uint8_t intermediateSteps = 0;

  const uint8_t patternSize = gait.getCompleteCycleSize(startSequenze, endSequenze);
  uint8_t pattern[patternSize];
  gait.getCompleteCycle(pattern, startSequenze, endSequenze);

  for(uint8_t i = 0; i < patternSize; ++i) {
    for(uint8_t j = 0; j < intermediateSteps; ++j) {
      for(uint8_t k = 0; k < 6; ++k) {
        Pointf nextPosition = this->legs[k].getNextLinearPoint(slope, moveUpwards, (pattern[i-1] >> j) & 0x01, (pattern[i] >> j) & 0x01, gait.getSwingPhaseCycles()*intermediateSteps, gait.getStancePhaseCycles()*intermediateSteps);
        this->moveToLocalPoint(static_cast<LegPosition>(k), nextPosition);
        avr::cout << "Leg " << k << ": " << nextPosition << '\n';
      }
      avr::cout << '\n';
    }
  }
  avr::cout << '\n';
}

#endif //HEXAPOD_H
