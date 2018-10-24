#include "Leg.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

namespace {
  static MotionRange calculateMotionRange(float radius = 20.0f, float deadRadius = 10.0f, float angle = 60.0f) {
    angle /= 2;
    MotionRange motionRange {Pointf {}, radius,
                             Pointf {-deadRadius*tan(angle*M_PI/180.0f), deadRadius}, Pointf {0, radius},
                             Pointf {deadRadius*tan(angle*M_PI/180.0f), deadRadius}, Pointf {0, radius}};

    motionRange.range[1].rotateXY(angle);
    motionRange.range[3].rotateXY(-angle);
    return motionRange;
  }

  static Pointf mapToGlobal(const Pointf& localPoint, float legOffset, float mountingAngle) {
    Pointf globalPoint = localPoint;
    globalPoint.y += legOffset;
    globalPoint.rotateXY(-mountingAngle);
    return globalPoint;
  }

  static Pointf mapToLocal(const Pointf& globalPoint, float legOffset, float mountingAngle) {
    Pointf localPoint = globalPoint;
    localPoint.rotateXY(mountingAngle);
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

/*
* Possible Problems
* - More than two intersections ()
*
*/
uint8_t Leg::getLargestPossibleDistance(float slope, bool inDirectionOfFunction) const {
  //Create function in local coordinate system
  LinearFunction temp {slope, 0};
  temp.rotateXY(this->mountingAngle);
  LinearFunction function {temp.slope, mapToLocal(this->position, this->legOffset, this->mountingAngle)};

  //LinearFunction has exactly two intersections with the motionRange
  Pointf intersections[2];
  uint8_t index = 0;

  LinearFunction leftMotionRange {motionRange.range[0], motionRange.range[1]};
  LinearFunction bottomMotionRange {motionRange.range[0], motionRange.range[2]};
  LinearFunction rightMotionRange {motionRange.range[2], motionRange.range[3]};

  //Find both insections. Intersection will be saved in the interection array. Index shows how much intersections have been found
  if(function.getIntersectionWith(leftMotionRange, intersections[index])) {
    avr::cout << "leftMotionRange " << intersections[index] << '\n';
    if(intersections[index].x >= motionRange.range[0].x && intersections[index].x <= motionRange.range[1].x) {
      ++index;
    }
  }
  if(function.getIntersectionWith(bottomMotionRange, intersections[index])) {
    avr::cout << "bottomMotionRange " << intersections[index] << '\n';
    if(intersections[index].x > motionRange.range[0].x && intersections[index].x  < motionRange.range[2].x) {
      ++index;
    }
  }
  if(index < 2 && function.getIntersectionWith(rightMotionRange, intersections[index])) {
    avr::cout << "rightMotionRange " << intersections[index] << '\n';
    if(intersections[index].x >= motionRange.range[2].x && intersections[index].x <= motionRange.range[3].x) {
      ++index;
    }
  }
  if(index < 2) {
    Pointf circleIntersections[2];
    function.getIntersectionWith(motionRange.circleCenter, motionRange.radius, circleIntersections);

    avr::cout << "circle1 " << circleIntersections[0] << '\n';
    avr::cout << "circle2 " << circleIntersections[1] << '\n';


    for(uint8_t i = 0; i < 2; ++i) {
      if(circleIntersections[i].x > motionRange.range[1].x && circleIntersections[i].y > motionRange.range[1].y
        && circleIntersections[i].x < motionRange.range[3].x) {
        intersections[index] = circleIntersections[i];
      }
    }
  }
  avr::cout << intersections[0] << '\n';
  avr::cout << intersections[1] << '\n';

  //Find the intersection of interest
  if(inDirectionOfFunction) {
    if(intersections[0].x > intersections[1].x ) {
      index = 0;
    } else {
      index = 1;
    }
  } else {
    if(intersections[0].x < intersections[1].x ) {
      index = 0;
    } else {
      index = 1;
    }
  }
  return abs(position.x-intersections[index].x);
}

void Leg::calculateMovementTo(const Pointf& destination, Pointf movementPath[], uint8_t size) const {

  // Calculates distance between position.x and destination.x and divides it through the number of steps bewteen these positions + 1
  // -> position.x + distance/size gives the x1 coordinate of the next step
  // d = (x2 - x1)/ STEPS +1
  float nextStep = (destination.x - position.x)/size;

  // Sets up a function equation (linear) to resolve a y value to a x value
  // m = (y2 - y1) / (x2 - x1)
  float slope = (destination.y - position.y) / (destination.x - position.x);
  // n = y1 - m*x1
  float yIntercept = destination.y - slope*destination.x;

  // Sets up a function equation (square) to resolve a z value to a x value
  // P0 = starting point, P1 = highest point, P2 = endpoint
  //a = (z2 + z0 - 2z1 - 2* sqrt(z0*z2 - z2*z1 - z0*z1 + z1*z1)) / (x0-x2)²
  float height = 4;
  float a = (destination.z + position.z - 2*height - 2*sqrt(position.z*destination.z - destination.z*height - position.z*height + height*height)) / ((position.x - destination.x)*(position.x - destination.x));
  //b = -1*(a*x2*x2 + z0 - a*x0*x0 - z2) / (-2*a*t2 + 2*a*t0)
  float b = -1.0f*(a*destination.x*destination.x + position.z - a*position.x*position.x - destination.z) / (-2*a*destination.x + 2*a*position.x);
  //c = z1 = HEIGHT

  calculateParabolicMovement(movementPath, size, nextStep, slope, yIntercept, a, b, height);
}


void Leg::updateAngles() {
  Pointf destination = mapToLocal(this->position, this->legOffset, this->mountingAngle);

  float legLength = sqrt((destination.x*destination.x) + (destination.y*destination.y));
  float lengthFemDes = sqrt((Leg::height - destination.z) * (Leg::height - destination.z) + (legLength - Leg::coxaLength) * (legLength - Leg::coxaLength));

  float angleCoxa = calculateCoxaAngle(destination);
  float angleFemur = calculateFemurAngle(destination, lengthFemDes);
  float angleTibia = calculateTibiaAngle(lengthFemDes);

  setAllAngles(angleCoxa, angleFemur, angleTibia);
}

void Leg::setPosition(const Pointf& position) {
  this->position = position;
}

const Pointf& Leg::getGlobalPosition() const {
  return this->position;
}

Pointf Leg::getLocalPosition() const {
  return mapToLocal(this->position, this->legOffset, this->mountingAngle);
}

float Leg::getLegOffset() const {
  return this->legOffset;
}

void Leg::setAllAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
  setAngle(Joint::Coxa, angleCoxa);
  setAngle(Joint::Femur, angleFemur);
  setAngle(Joint::Tibia, angleTibia);
}

void Leg::setAngle(Joint joint, uint8_t angle) {
  switch(joint) {
    case Joint::Coxa:   this->coxaServo.setAngle(angle);  break;
    case Joint::Femur:  this->femurServo.setAngle(angle); break;
    case Joint::Tibia:  this->tibiaServo.setAngle(angle); break;
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

void Leg::calculateParabolicMovement(Pointf movementPath[], uint8_t size, float nextStep, float slope, float yIntercept, float a, float b, float c) const {
  Pointf nextPosition {this->position};

  for(uint8_t i = 0; i < size; i++) {
    nextPosition.x += nextStep;
    //f(x) = m*x + n
    nextPosition.y = slope*nextPosition.x + yIntercept;
    //f(x) = a*(x - b)² + c
    nextPosition.z = a*((nextPosition.x - b)*(nextPosition.x-b)) + c;

    movementPath[i] = nextPosition;
  }
}

float Leg::calculateCoxaAngle(const Pointf& destination) const {
  return atan(destination.x/destination.y)*180.0f/M_PI + Servo::angleRange/2.0f;
}

float Leg::calculateFemurAngle(const Pointf& destination, float lengthFemDes) const {
  //a1 = cos⁻¹((Höhe - z0)/ lengthFemDes)
  float angleFemur1 = acos((Leg::height - destination.z) / lengthFemDes)*180.0f/M_PI;
  //a2 = cos⁻¹((FEMUR² + lengthFemDes² - Tibia²) / (2 * Femur * lengthFemdes))
  float angleFemur2 = acos((Leg::femurLength*Leg::femurLength + lengthFemDes*lengthFemDes - Leg::tibiaLength*Leg::tibiaLength) / (2.0f*Leg::femurLength*lengthFemDes))*180.0f/M_PI;
  float angleFemur = angleFemur1 + angleFemur2;

  if(isLegOnLeftSide()) {
    angleFemur -= 45.0f;
  } else {
    angleFemur = 225.0f - angleFemur;
  }

  return angleFemur;
}

float Leg::calculateTibiaAngle(float lengthFemDes) const {
  float angleTibia = acos((Leg::femurLength*Leg::femurLength + Leg::tibiaLength*Leg::tibiaLength - lengthFemDes*lengthFemDes)
                          / (2.0f*Leg::femurLength + Leg::tibiaLength))*180.0f/M_PI;

  if(isLegOnLeftSide()) {
    angleTibia = 180.0f - angleTibia;
  }

  return angleTibia;
}

bool Leg::isLegOnLeftSide() const {
  return (this->mountingAngle == 0 || this->mountingAngle == 62 || this->mountingAngle == 298);
}
