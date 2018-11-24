#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "Point.h"
#include "LinearFunction.h"
#include "QuadraticFunction.h"

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

enum class Joint { Coxa, Femur, Tibia };

struct MotionRange {
  uint8_t radius;
  Point<int16_t> range[4];
};

class Leg {

public:
  /*!
  @brief Instanstiates a Leg with it`s three servos and it's mounting angle and mounting point
  @param legOffset The offset from the rotated mountingPoint on the y-axis
  @param mountingAngle The angle at which the servo is mounted
  @param position The (default) position of the foot
  */
  Leg(Servo&& coxaServo, Servo&& femurServo, Servo&& tibiaServo, Point<int16_t> position, uint8_t legOffset, uint16_t mountingAngle);

  /*!
  @brief Updates all Servos
  */
  void update(uint32_t currentMillis);

  /*!
  @brief Returns the intersection between the linear function with the given slope and the movement range
  */
  Point<int16_t> getLastLinearPoint(const LinearFunction& function, float slope, bool moveUpwards) const;

  /*!
  @brief Puts up a quadratic function from position to destination. Can be used to resolve a z value to a x value
  */
  QuadraticFunction getQuadraticFunction(const Point<int16_t>& destination, int8_t jumpHeight, bool highestPointReached) const;

  /*!
  @brief Puts up a linear function with the given slope through the current position. Can be used to resolve a y value to a x value
  */
  LinearFunction getLinearFunction(float slope) const;

  /*!
  @brief Updates the angles of the servos, so the current position can be reached
  */
  void updateAngles();


  void setGlobalPosition(const Point<int16_t>& position);
  Point<int16_t> getGlobalPosition() const;

  void setLocalPosition(const Point<int16_t>& position);
  const Point<int16_t>& getLocalPosition() const;

  void rotateXYZ(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle);

  /*!Servo wrapper functions!*/

  /*!
  @brief Sets the angles of all Servos
  @param angleCoxa  angle of the coxa  Servo between 0 and  180 degrees
  @param angleFemur angle of the femur Servo between 0 and  180 degrees
  @param angleTibia angle of the tibia Servo between 0 and  180 degrees
  */
  void setAllAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia);

  /*!
  @brief see discription of "void setAngle(uint8_t angle)" in Servo.h
  @param joint Determines the servo the setAngle() function should be called on
  */
  void setAngle(Joint joint, uint8_t angle);

  /*!
  @brief Moves all Servos (enables them for the Servos update method)
  @param time Time the servo should need for the movement
  */
  void moveAll(uint16_t time = 0);

  /*!
  @brief see discription of "void move(uint16_t time)" in Servo.h
  @param joint Determines the servo the setAngle() function should be called on
  */
  void move(Joint joint, uint16_t time = 0);

private:
  float calculateCoxaAngle() const;
  float calculateFemurAngle(float lengthFemDes) const;
  float calculateTibiaAngle(float lengthFemDes) const;

  bool isLegOnLeftSide() const;

  /*! These values probably have to be customized for your hexapod */
  constexpr static uint8_t coxaLength = 25;
  constexpr static uint8_t femurLength = 85;
  constexpr static uint8_t tibiaLength = 120;
  //Differnce between femurMountingHeight (calculations are done from here) and the orginin
  constexpr static uint8_t zOffset = 50;
  //The servo has 45° at 0° + tibia is bent
  constexpr static uint8_t femurAngleOffset = 37; //45 //37.5
  //Because tibia leg is bendt
  constexpr static uint8_t tibiaAngleOffset = 15; //0
  /****************************************************************/

  static MotionRange motionRange;

  Servo coxaServo;
  Servo femurServo;
  Servo tibiaServo;

  //local position in millimeter
  Point<int16_t> position;

  const uint8_t legOffset;
  const uint16_t mountingAngle;
};
#endif //LEG_H
