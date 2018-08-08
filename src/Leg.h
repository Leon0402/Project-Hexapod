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
#define STEPS 64

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
    setCoxaAngle(angleCoxa);
    setFemurAngle(angleFemur);
    setTibiaAngle(angleTibia);
  }

  //see discription of "void setAngle(uint8_t angle)" in Servo.h
  void setCoxaAngle(uint8_t angleCoxa) { this->coxaServo.setAngle(angleCoxa); }
  //see discription of "void setAngle(uint8_t angle)" in Servo.h
  void setFemurAngle(uint8_t angleFemur) { this->femurServo.setAngle(angleFemur); }
  //see discription of "void setAngle(uint8_t angle)" in Servo.h
  void setTibiaAngle(uint8_t angleTibia) { this->tibiaServo.setAngle(angleTibia); }

  //see discription of "void addCoxaAngle(uint8_t angle)" in Servo.h
  void addCoxaAngle(uint8_t angle) { this->coxaServo.addAngle(angle); }
  //see discription of "void subCoxaAngle(uint8_t angle)" in Servo.h
  void subCoxaAngle(uint8_t angle) { this->coxaServo.subAngle(angle); }

  //see discription of "uint8_t getPin() const" in Servo.h
  uint8_t getCoxaPin() const  { return this->coxaServo.getPin();  }
  //see discription of "uint8_t getPin() const" in Servo.h
  uint8_t getFemurPin() const { return this->femurServo.getPin(); }
  //see discription of "uint8_t getPin() const" in Servo.h
  uint8_t getTibiaPin() const { return this->tibiaServo.getPin(); }

  //see discription of "uint16_t getCoxaOnTime() const" in Servo.h
  uint16_t getCoxaOnTime() const  { return this->coxaServo.getOnTime(); }
  //see discription of "uint16_t getCoxaOnTime() const" in Servo.h
  uint16_t getFemurOnTime() const { return this->femurServo.getOnTime(); }
  //see discription of "uint16_t getCoxaOnTime() const" in Servo.h
  uint16_t getTibiaOnTime() const { return this->tibiaServo.getOnTime(); }

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
