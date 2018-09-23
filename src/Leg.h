#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "Point.h"

#ifdef DEBUG
  #include <cstdint>
  #include <iostream>
#else
 #include <inttypes.h>
 #include "Stream.h"
#endif

#define COXA 2.5
#define FEMUR 8.5
#define TIBIA 11.5
#define HEIGHT 7 //from femur mounting point
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
  void setAllAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
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
      case Joint::Coxa:   this->coxaServo.setAngle(angle);  break;
      case Joint::Femur:  this->femurServo.setAngle(angle); break;
      case Joint::Tibia:  this->tibiaServo.setAngle(angle); break;
      default: avr::cout << "Joint not available" << '\n';
    }
  }

  void moveAll(uint16_t time = 0) {
    move(Joint::Coxa, time);
    move(Joint::Femur, time);
    move(Joint::Tibia, time);
  }

  void move(Joint joint, uint16_t time = 0) {
    switch(joint) {
      case Joint::Coxa:   this->coxaServo.move(time);  break;
      case Joint::Femur:  this->femurServo.move(time); break;
      case Joint::Tibia:  this->tibiaServo.move(time); break;
      default: avr::cout << "Joint not available" << '\n';
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
