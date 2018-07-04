#include "Leg.h"

#ifdef DEBUG
#include <cmath>
#include <iostream>
#else
#include <math.h>
#endif

/******************************************************************************************************************************************************/
//public
/******************************************************************************************************************************************************/

Leg::Leg(Servo& coxaServo, Servo& femurServo, Servo& tibiaServo, float legOffset, uint16_t mountingAngle, Pointf position)
: coxaServo(coxaServo), femurServo(femurServo), tibiaServo(tibiaServo), legOffset(legOffset), mountingAngle(mountingAngle), position(position) {
  //calculateAngles();
}

void Leg::moveTo(const Pointf& destination) {

  /*
  * Calculates distance between position.x and destination.x and divides it through the number of steps bewteen these positions + 1
  * -> position.x + distance/(steps +1) gives the x1 coordinate of the next step
  */
  //d = (x2 - x1)/ STEPS +1
  float nextStep = (destination.x - position.x)/(STEPS + 1);

  /*
  * Sets up a function equation (linear) to resolve a y value to a x value
  */
  //m = (y2 - y1) / (x2 - x1)
  float slope = (destination.y - position.y) / (destination.x - position.x);
  //n = y1 - m*x1
  float yIntercept = destination.y - slope*destination.x;

  /*
  * Sets up a function equation (square) to resolve a z value to a x value
  *P0 = starting point, P1 = highest point, P2 = endpoint
  */
  //a = (z2 + z0 - 2z1 - 2* sqrt(z0*z2 - z2*z1 - z0*z1 + z1*z1)) / (x0-x2)²
  float a = (destination.z + position.z - 2*HEIGHT - 2*sqrt(position.z*destination.z - destination.z*HEIGHT - position.z*HEIGHT + HEIGHT*HEIGHT)) / ((position.x - destination.x)*(position.x - destination.x));
  //b = -1*(a*x2*x2 + z0 - a*x0*x0 - z2) / (-2*a*t2 + 2*a*t0)
  float b = -1.0f*(a*destination.x*destination.x + position.z - a*position.x*position.x - destination.z) / (-2*a*destination.x + 2*a*position.x);
  //c = z1
  float c = HEIGHT;

#ifdef DEBUG
      std::cout << "nextStep: " << nextStep << '\n';
      std::cout << "f(x) = m*x + y: " << slope << "*x + " << yIntercept << '\n';
      std::cout << "f(x) = a*(x - b)² + c: " << a << "*(x - " << b << ")² + " << c << '\n';
      std::cout << "currentPosition: " << position << '\n';
#endif

  /*
  *Calculates position of the next step.
  * 1. Add nextStep to position.x to get new x value
  * 2. Resolves the y value to the x value
  * 3. Resolves the z value to the x value
  * 4. Repeating 1-3 for STEPS +1 times (until position == destination -> jump finished)
  */
  for(uint8_t i = 0; i <= STEPS; i++) {
    #ifdef DEBUG
      std::cout << position.x << '\n';
      std::cout << nextStep << '\n';
    #endif
    position.x += nextStep;
    //f(x) = m*x + n
    position.y = slope*position.x + yIntercept;
    //f(x) = a*(x - b)² + c
    position.z = a*((position.x - b)*(position.x-b)) + c;

    #ifdef DEBUG
      std::cout << "*********** Movement **********" << '\n';
      std::cout << "nextPosition: " << position << '\n';
      std::cout << "destination: " << destination << '\n';
      std::cout << "*******************************" << '\n';
    #endif

    /*
    * Calulates the angles to move to the calculated position and moves the servos
    */
    calculateAngles();
  }
}

void Leg::setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
    coxaServo.write(angleCoxa);
    femurServo.write(angleFemur);
    tibiaServo.write(angleTibia);
}

/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/

void Leg::calculateAngles() {
  Pointf destination = this->position;
  destination.rotateXY(mountingAngle);
  destination.y -= legOffset;

  uint8_t angleCoxa = atan((destination.x/destination.y))*180/M_PI + ANGLEMAX/2 + 0.5;

  uint8_t lengthLeg = sqrt((destination.x*destination.x) + (destination.y*destination.y)) + 0.5;
  float lengthFemDes = sqrt((HEIGHT - destination.z) * (HEIGHT - destination.z) + (lengthLeg - COXA) * (lengthLeg - COXA));

  uint8_t angleFemur1 = acos(((HEIGHT - destination.z) / lengthFemDes))*180/M_PI + 0.5;
  uint8_t angleFemur2 = acos((FEMUR*FEMUR + lengthFemDes*lengthFemDes - TIBIA*TIBIA) / (2*FEMUR*lengthFemDes))*180/M_PI + 0.5;
  uint8_t angleFemur = angleFemur1 + angleFemur2 + 0.5;

  uint8_t angleTibia = acos((FEMUR*FEMUR + TIBIA*TIBIA - lengthFemDes*lengthFemDes) / (2*FEMUR*TIBIA))*180/M_PI + 0.5;

  //Change angles depending on the position of the leg and the hardware configuration of the servos
  if(mountingAngle == 0 || mountingAngle == 62 || mountingAngle == 298) {
    angleFemur -= 45;
    angleTibia = 180 - angleTibia;
  } else {
    angleFemur = 225 - angleFemur;
  }

#ifdef DEBUG
  std::cout << "***********  Angles  **********" << '\n';
  std::cout << "angleCoxa: " << (unsigned int)angleCoxa << '\n';
  std::cout << "angleFemur: " << (unsigned int)angleFemur << '\n';
  std::cout << "angleTibia: " << (unsigned int)angleTibia << '\n';
  std::cout << "*******************************" << '\n';
#endif

  setAngles(angleCoxa, angleFemur, angleTibia);
}
