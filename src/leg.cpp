#include "Leg.h"

#ifdef DEBUG
  #include <cmath>
  #include <iostream>
  #define PRINT(X) std::cout << (#X) << '\t' << (X) << std::endl;
#else
  #include <math.h>
#endif

/******************************************************************************************************************************************************/
//public
/******************************************************************************************************************************************************/

Leg::Leg(Servo& coxaServo, Servo& femurServo, Servo& tibiaServo, Pointf position, const float legOffset, const float mountingAngle)
: coxaServo {coxaServo}, femurServo {femurServo}, tibiaServo {tibiaServo}, position {position}, legOffset {legOffset}, mountingAngle {mountingAngle} {}


void Leg::calculateMovementTo(Pointf movementPath[], const Pointf& destination) const {

  // Calculates distance between position.x and destination.x and divides it through the number of steps bewteen these positions + 1
  // -> position.x + distance/(steps +1) gives the x1 coordinate of the next step
  // d = (x2 - x1)/ STEPS +1
  float nextStep = (destination.x - position.x)/(STEPS + 1);

  // Sets up a function equation (linear) to resolve a y value to a x value
  // m = (y2 - y1) / (x2 - x1)
  float slope = (destination.y - position.y) / (destination.x - position.x);
  // n = y1 - m*x1
  float yIntercept = destination.y - slope*destination.x;

  // Sets up a function equation (square) to resolve a z value to a x value
  // P0 = starting point, P1 = highest point, P2 = endpoint
  //a = (z2 + z0 - 2z1 - 2* sqrt(z0*z2 - z2*z1 - z0*z1 + z1*z1)) / (x0-x2)²
  float a = (destination.z + position.z - 2*HEIGHT - 2*sqrt(position.z*destination.z - destination.z*HEIGHT - position.z*HEIGHT + HEIGHT*HEIGHT)) / ((position.x - destination.x)*(position.x - destination.x));
  //b = -1*(a*x2*x2 + z0 - a*x0*x0 - z2) / (-2*a*t2 + 2*a*t0)
  float b = -1.0f*(a*destination.x*destination.x + position.z - a*position.x*position.x - destination.z) / (-2*a*destination.x + 2*a*position.x);
  //c = z1 = HEIGHT

  #ifdef DEBUG
  PRINT(nextStep);
  PRINT(slope);
  PRINT(yIntercept);
  PRINT(a);
  PRINT(b);
  PRINT(HEIGHT);
  #endif

  calculateParabolicMovement(movementPath, nextStep, slope, yIntercept, a, b, HEIGHT);
}


void Leg::updateAngles() {
  Pointf destination {this->position};
  destination.rotateXY(this->mountingAngle);
  destination.y -= this->legOffset;

  float lengthLeg = sqrt((destination.x*destination.x) + (destination.y*destination.y));
  float lengthFemDes = sqrt((HEIGHT - destination.z) * (HEIGHT - destination.z) + (lengthLeg - COXA) * (lengthLeg - COXA));

  float angleCoxa = calculateCoxaAngle(destination);
  float angleFemur = calculateFemurAngle(destination, lengthFemDes);
  float angleTibia = calculateTibiaAngle(destination, lengthFemDes);

  setAngles(angleCoxa, angleFemur, angleTibia);
}

/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/

void Leg::calculateParabolicMovement(Pointf movementPath[], float nextStep, float slope, float yIntercept, float a, float b, float c) const {
  Pointf nextPosition {this->position};

  for(uint8_t i = 0; i <= STEPS; i++) {
    nextPosition.x += nextStep;
    //f(x) = m*x + n
    nextPosition.y = slope*nextPosition.x + yIntercept;
    //f(x) = a*(x - b)² + c
    nextPosition.z = a*((nextPosition.x - b)*(nextPosition.x-b)) + c;

    movementPath[i] = nextPosition;
  }
}

float Leg::calculateCoxaAngle(const Pointf& destination) const {
  float angleCoxa = atan(destination.x/destination.y)*180.0f/M_PI + ANGLEMAX/2.0f;

  #ifdef DEBUG
  PRINT(angleCoxa);
  #endif

  return angleCoxa;
}

float Leg::calculateFemurAngle(const Pointf& destination, float lengthFemDes) const {
  //a1 = cos⁻¹((Höhe - z0)/ lengthFemDes)
  float angleFemur1 = acos(((HEIGHT - destination.z) / lengthFemDes))*180.0f/M_PI;
  //a2 = cos⁻¹((FEMUR² + lengthFemDes² - Tibia²) / (2 * Femur * lengthFemdes))
  float angleFemur2 = acos((FEMUR*FEMUR + lengthFemDes*lengthFemDes - TIBIA*TIBIA) / (2*FEMUR*lengthFemDes))*180.0f/M_PI;
  float angleFemur = angleFemur1 + angleFemur2;

  #ifdef DEBUG
  PRINT(angleFemur);
  #endif

  if(isLegOnLeftSide()) {
    angleFemur -= 45;
  }
  else {
    angleFemur = 225 - angleFemur;
  }

  return angleFemur;
}

float Leg::calculateTibiaAngle(const Pointf& destination, float lengthFemDes) const {
  float angleTibia = acos((FEMUR*FEMUR + TIBIA*TIBIA - lengthFemDes*lengthFemDes) / (2*FEMUR*TIBIA))*180.0f/M_PI;

  #ifdef DEBUG
  PRINT(angleTibia);
  #endif

  if(isLegOnLeftSide()) {
    angleTibia = 180 - angleTibia;
  }

  return angleTibia;
}

bool Leg::isLegOnLeftSide() const {
  return (this->mountingAngle == 0 || this->mountingAngle == 62 || this->mountingAngle == 298);
}
