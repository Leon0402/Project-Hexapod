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
  @brief Body movement: roll
  */
  void roll(float angle);

  /*!
  @brief Body movement: pitch
  */
  void pitch(float angle);

  /*
  @brief Body movement: yaw
  */
  void yaw(float angle);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time
  @param legPosition position of the leg (see LegPosition Enum)
  @param desination point in the global coordinate system
  @param time time in milliseconds
  */
  void moveLegDirectlyToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time = 0);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time following a parabolic function
  @param legPosition position of the leg (see LegPosition Enum)
  @param desination point in the global coordinate system
  @param time time in milliseconds
  */
  void moveLegToPoint(LegPosition legPosition, const Pointf& destination, uint16_t time = 0);


  /*!
  @brief Some test scripts, moves forwards
  */
  void startPosition_test();
  void calibration_test();
  void moveForward_test();
  void roll_test();
  void pitch_test();

private:
  float rollAngle;
  float pitchAngle;
  float yawAngle;

  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};
#endif //HEXAPOD_H
