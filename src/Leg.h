#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "util/Point.h"

#ifdef DEBUG
  #include <cstdint>
#else
 #include <inttypes.h>
#endif

#define COXA 2.5
#define FEMUR 8.5
#define TIBIA 11.5
#define HEIGHT 5 //from femur mounting point
#define STEPS 50 //max 254

enum class Joint { Coxa, Femur, Tibia };

class Leg {
public:
  /*!
  @brief Instanstiates a Leg with it`s three servos and it's mounting angle and mounting point
  @param legOffset The offset from the rotated mountingPoint on the y-axis
  @param mountingAngle The angle at which the servo is mounted
  @param position The (default) position of the foot
  */
  Leg(Servo& coxaServo, Servo& femurServo, Servo& tibiaServo, Pointf position, const float legOffset, const float mountingAngle);

  /*!
  @brief Calculates a movement path (parabolic) to reach a point from the legs current position. The points are written in the given array
  @param destination The destination given in the Hexapod`s coordinate system
  */
  void calculateMovementTo(Pointf movementPath[], const Pointf& destination) const;

  /*!
  @brief Updates the angles of the servos, so the current position can be reached
  */
  void updateAngles();

  //getter and setter
  void setPosition(const Pointf& position) { this->position = position; }
  Pointf getPosition() const { return this->position; }

  /*!Servo wrapper functions!*/

  /*!
  @brief Sets the angles of all Servos
  @param angleCoxa angle of the coxa Servo between 0 and  180 degrees
  @param angleFemur angle of the femur Servo between 0 and  180 degrees
  @param angleTibia angle of the tibia Servo between 0 and  180 degrees
  */
  void setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
    setAngle(Joint::Coxa, angleCoxa);
    setAngle(Joint::Femur, angleFemur);
    setAngle(Joint::Tibia, angleTibia);
  }

  /*!
  @brief see discription of "void setAngle(uint8_t angle)" in Servo.h
  @param joint Determines the servo the setAngle() function should be called on
  */
  void setAngle(Joint joint, uint8_t angle) {
    switch(joint) {
      case Joint::Coxa:  this->coxaServo.setAngle(angle);   break;
      case Joint::Femur: this->femurServo.setAngle(angle);  break;
      case Joint::Tibia:  this->tibiaServo.setAngle(angle); break;
    }
  }

  /*!
  @brief see discription of "void addAngle(uint8_t angle)" in Servo.h
  @param joint Determines the servo the addAngle() function should be called on
  */
  void addAngle(Joint joint, uint8_t angle) {
    switch(joint) {
      case Joint::Coxa:  this->coxaServo.addAngle(angle);   break;
      case Joint::Femur: this->femurServo.addAngle(angle);  break;
      case Joint::Tibia:  this->tibiaServo.addAngle(angle); break;
    }
  }

  /*!
  @brief see discription of "void subAngle(uint8_t angle)" in Servo.h
  @param joint Determines the servo the subAngle() function should be called on
  */
  void subAngle(Joint joint, uint8_t angle) {
    switch(joint) {
      case Joint::Coxa:  this->coxaServo.subAngle(angle);   break;
      case Joint::Femur: this->femurServo.subAngle(angle);  break;
      case Joint::Tibia:  this->tibiaServo.subAngle(angle); break;
    }
  }

  /*!
  @brief see discription of "uint8_t getPin() const" in Servo.h
  @param joint Determines the servo the getPin() function should be called on
  */
  uint8_t getPin(Joint joint) {
    switch(joint) {
      case Joint::Coxa:  return this->coxaServo.getPin();  break;
      case Joint::Femur: return this->femurServo.getPin(); break;
      case Joint::Tibia: return this->tibiaServo.getPin(); break;
      default: return 0;
    }
  }

  /*!
  @brief see discription of "uint16_t getOnTime() const" in Servo.h
  @param joint Determines the servo the getOnTime() function should be called on
  */
  uint16_t getOnTime(Joint joint) {
    switch(joint) {
      case Joint::Coxa:  return this->coxaServo.getOnTime();  break;
      case Joint::Femur: return this->femurServo.getOnTime(); break;
      case Joint::Tibia: return this->tibiaServo.getOnTime(); break;
      default: return 0;
    }
  }

private:
  void calculateParabolicMovement(Pointf nextMovementArray[], float nextStep, float slope, float yIntercept, float a, float b, float c) const;

  float calculateCoxaAngle(const Pointf& destination) const;
  float calculateFemurAngle(const Pointf& destination, float lengthFemDes) const;
  float calculateTibiaAngle(const Pointf& destination, float lengthFemDes) const;

  bool isLegOnLeftSide() const;

  Servo coxaServo;
  Servo femurServo;
  Servo tibiaServo;

  Pointf position;

  const float legOffset;
  const float mountingAngle;
};

#endif //LEG_H
