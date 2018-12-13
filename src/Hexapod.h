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
  void moveToLocalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time = 0);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time
  @param legPosition Position of the leg, which should move (see LegPosition Enum)
  @param destination Point in the global coordinate system the leg should move to
  @param time The time the leg needs to move
  */
  void moveToGlobalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time = 0);

private:
  void move(const LegPosition& legPosition, uint16_t time = 0);

  int8_t yawAngle;
  int8_t pitchAngle;
  int8_t rollAngle;

  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};

#include "Hexapod.inl"

#endif //HEXAPOD_H
