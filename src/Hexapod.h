#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Servocontroller.h"
#include "Leg.h"

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
  void moveLinear(float slope, bool moveUpwards);

  /*!
  @brief Complete body rotation
  */
  void bodyIk(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle);

  /*
  @brief Body movement: yaw
  */
  void yaw(int8_t angle);

  /*!
  @brief Body movement: roll
  */
  void roll(int8_t angle);

  /*!
  @brief Body movement: pitch
  */
  void pitch(int8_t angle);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time
  @param legPosition position of the leg (see LegPosition Enum)
  @param desination point in the global coordinate system
  @param time time in milliseconds
  */
  //void moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time = 0);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time following a parabolic function
  @param legPosition position of the leg (see LegPosition Enum)
  @param desination point in the global coordinate system
  @param time time in milliseconds
  */
  //void moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time = 0);

  /*!
  @brief Some test scripts, moves forwards
  */
  void moveForward_test();
  void bodyIk_test();

private:
  int8_t yawAngle;
  int8_t pitchAngle;
  int8_t rollAngle;

  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};
#endif //HEXAPOD_H
