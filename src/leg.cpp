#include "Leg.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

namespace {
  static MotionRange calculateMotionRange(float radius = 15.0f, float deadRadius = 5.0f, float angle = 60.0f) {
    angle /= 2;
    MotionRange motionRange {radius,
                             Pointf {-deadRadius*tan(angle*M_PI/180.0f), deadRadius}, Pointf {0, radius},
                             Pointf {deadRadius*tan(angle*M_PI/180.0f), deadRadius}, Pointf {0, radius}};

    motionRange.range[1].rotateZ(angle);
    motionRange.range[3].rotateZ(-angle);
    return motionRange;
  }

  static Pointf mapToGlobal(const Pointf& localPoint, float legOffset, float mountingAngle) {
    Pointf globalPoint = localPoint;
    globalPoint.y += legOffset;
    globalPoint.rotateZ(-mountingAngle);
    return globalPoint;
  }

  static Pointf mapToLocal(const Pointf& globalPoint, float legOffset, float mountingAngle) {
    Pointf localPoint = globalPoint;
    localPoint.rotateZ(mountingAngle);
    localPoint.y -= legOffset;
    return localPoint;
  }
}

/******************************************************************************************************************************************************/
//public
/******************************************************************************************************************************************************/
Leg::Leg(Servo&& coxaServo, Servo&& femurServo, Servo&& tibiaServo, Pointf position, float legOffset, float mountingAngle)
: coxaServo {coxaServo}, femurServo {femurServo}, tibiaServo {tibiaServo},
  position {position}, legOffset {legOffset}, mountingAngle {mountingAngle},
  motionRange {calculateMotionRange()} {}

void Leg::update(uint32_t currentMillis) {
  coxaServo.update(currentMillis);
  femurServo.update(currentMillis);
  tibiaServo.update(currentMillis);
}

Pointf Leg::getLastLinearPoint(float slope, bool moveUpwards) const {
  if(this->isLegOnLeftSide()) {
    moveUpwards = !moveUpwards;
  }

  LinearFunction function = this->getLinearFunction(slope);

  //LinearFunction has exactly two intersections with the motionRange
  Pointf intersections[2];
  uint8_t index = 0;

  LinearFunction leftMotionRange {motionRange.range[0], motionRange.range[1]};
  LinearFunction bottomMotionRange {motionRange.range[0], motionRange.range[2]};
  LinearFunction rightMotionRange {motionRange.range[2], motionRange.range[3]};

  //Find both insections. Intersection will be saved in the interection array.
  //Index shows how much intersections have been found
  if(function.getIntersectionWith(leftMotionRange, intersections[index])) {
    if(intersections[index].x >= motionRange.range[1].x && intersections[index].x <= motionRange.range[0].x) {
      ++index;
    }
  }
  if(function.getIntersectionWith(bottomMotionRange, intersections[index])) {
    if(intersections[index].x > motionRange.range[0].x && intersections[index].x  < motionRange.range[2].x) {
      ++index;
    }
  }
  if(index < 2 && function.getIntersectionWith(rightMotionRange, intersections[index])) {
    if(intersections[index].x >= motionRange.range[2].x && intersections[index].x <= motionRange.range[3].x) {
      ++index;
    }
  }
  if(index < 2) {
    Pointf circleIntersections[2];
    function.getIntersectionWith(Pointf {0.0f, 0.0f}, motionRange.radius, circleIntersections);

    for(uint8_t i = 0; i < 2; ++i) {
      if(circleIntersections[i].y > motionRange.range[1].y) {
        intersections[index] = circleIntersections[i];
      }
    }
  }

  //Find the intersection of interest
  if(moveUpwards) {
    if(intersections[0].x > intersections[1].x ) {
      return intersections[0];
    } else {
      return intersections[1];
    }
  } else {
    if(intersections[0].x < intersections[1].x ) {
      return intersections[0];
    } else {
      return intersections[1];
    }
  }
}

// Sets up a function equation (square) to resolve a z value to a x value
// P0 = starting point, P1 = highest point, P2 = endpoint
QuadraticFunction Leg::getQuadraticFunction(const Pointf& destination, const Pointf& position, float jumpHeight) const {
  //a = (z2 + z0 - 2z1 - 2* sqrt(z0*z2 - z2*z1 - z0*z1 + z1*z1)) / (x0-x2)²
  float a = (destination.z + position.z - 2.0f*jumpHeight - 2.0f*sqrt(position.z*destination.z - destination.z*jumpHeight - position.z*jumpHeight + jumpHeight*jumpHeight)) / ((position.x - destination.x)*(position.x - destination.x));
  //b = -1*(a*x2*x2 + z0 - a*x0*x0 - z2) / (-2*a*t2 + 2*a*t0)
  float b = -1.0f*(a*destination.x*destination.x + position.z - a*position.x*position.x - destination.z) / (-2.0f*a*destination.x + 2.0f*a*position.x);
  //c = z1 = HEIGHT
  float c = jumpHeight;

  return QuadraticFunction {a, b, c};
}

LinearFunction Leg::getLinearFunction(float slope) const {
  //Create function in local coordinate system
  LinearFunction temp {slope, 0};
  temp.rotateXY(this->mountingAngle);
  return LinearFunction {temp.slope, this->position};
}

void Leg::updateAngles() {
  //Some calculations needed for the calculation of femur and tibia
  float legLength = sqrt(this->position.x*this->position.x + this->position.y*this->position.y);
  float lengthFemDes = sqrt((this->position.z + zOffset)*(this->position.z + zOffset) + (legLength - Leg::coxaLength)*(legLength - Leg::coxaLength));

  //calculate coxa,femur and tibia angle with trigometry
  float angleCoxa = calculateCoxaAngle();
  float angleFemur = calculateFemurAngle(lengthFemDes);
  float angleTibia = calculateTibiaAngle(lengthFemDes);

  //angles in Servo object will be updated
  setAllAngles(angleCoxa, angleFemur, angleTibia);
}

void Leg::setGlobalPosition(const Pointf& position) {
  this->position = mapToLocal(position, this->legOffset, this->mountingAngle);
}

Pointf Leg::getGlobalPosition() const {
  return mapToGlobal(this->position, this->legOffset, this->mountingAngle);
}

void Leg::setLocalPosition(const Pointf& position) {
  this->position = position;
}

const Pointf& Leg::getLocalPosition() const {
  return this->position;
}

void Leg::rotateXYZ(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle) {
  Pointf globalPosition = this->getGlobalPosition();
  globalPosition.rotateXYZ(static_cast<float>(yawAngle), static_cast<float>(pitchAngle), static_cast<float>(rollAngle));
  this->setGlobalPosition(globalPosition);
}

void Leg::setAllAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
  setAngle(Joint::Coxa, angleCoxa);
  setAngle(Joint::Femur, angleFemur);
  setAngle(Joint::Tibia, angleTibia);
}

void Leg::setAngle(Joint joint, uint8_t angle) {
  if(isLegOnLeftSide()) {
    switch(joint) {
      case Joint::Coxa:   this->coxaServo.setAngle(angle);  break;
      case Joint::Femur:  this->femurServo.setAngle(angle - femurAngleOffset); break;
      case Joint::Tibia:  this->tibiaServo.setAngle(180.0f - angle + tibiaAngleOffset); break;
    }
  } else {
    switch(joint) {
      case Joint::Coxa:   this->coxaServo.setAngle(angle);  break;
      case Joint::Femur:  this->femurServo.setAngle(180.0f + femurAngleOffset - angle); break;
      case Joint::Tibia:  this->tibiaServo.setAngle(angle - tibiaAngleOffset); break;
    }
  }
}

void Leg::moveAll(uint16_t time) {
  move(Joint::Coxa, time);
  move(Joint::Femur, time);
  move(Joint::Tibia, time);
}

void Leg::move(Joint joint, uint16_t time) {
  switch(joint) {
    case Joint::Coxa:   this->coxaServo.move(time);  break;
    case Joint::Femur:  this->femurServo.move(time); break;
    case Joint::Tibia:  this->tibiaServo.move(time); break;
  }
}

/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/

float Leg::calculateCoxaAngle() const {
  //a = tan⁻¹(x/y)
  return atan(this->position.x/this->position.y)*180.0f/M_PI + Servo::angleRange/2.0f;
}

float Leg::calculateFemurAngle(float lengthFemDes) const {
  //a1 = cos⁻¹(-z0 / lengthFemDes)
  float angleFemur1 = acos((-this->position.z - zOffset)/lengthFemDes)*180.0f/M_PI;
  //a2 = cos⁻¹((FEMUR² + lengthFemDes² - Tibia²) / (2 * Femur * lengthFemDes))
  float angleFemur2 = acos((Leg::femurLength*Leg::femurLength + lengthFemDes*lengthFemDes - Leg::tibiaLength*Leg::tibiaLength) / (2.0f*Leg::femurLength*lengthFemDes))*180.0f/M_PI;
  return angleFemur1 + angleFemur2;
}

float Leg::calculateTibiaAngle(float lengthFemDes) const {
  //b = cos⁻¹((FEMUR² + TIBIA² - lengthFemDes²) / (2 * FEMUR * TIBIA)
  return acos((Leg::femurLength*Leg::femurLength + Leg::tibiaLength*Leg::tibiaLength - lengthFemDes*lengthFemDes)/(2.0f*Leg::femurLength*Leg::tibiaLength))*180.0f/M_PI;
}

bool Leg::isLegOnLeftSide() const {
  return this->mountingAngle == 0 || this->mountingAngle == 62 || this->mountingAngle == 298;
}
