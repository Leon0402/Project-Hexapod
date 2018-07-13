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

void Leg::setCoxaAngle(uint8_t angleCoxa) {
  coxaServo.write(angleCoxa);
}

void Leg::setFemurAngle(uint8_t angleFemur) {
  femurServo.write(angleFemur);
}

void Leg::setTibiaAngle(uint8_t angleTibia) {
  tibiaServo.write(angleTibia);
}

void Leg::setPosition(const Pointf& position) {
  this->position = position;
  calculateAngles();
}
/******************************************************************************************************************************************************/
//private
/******************************************************************************************************************************************************/

void Leg::calculateAngles() {
  Pointf destination = this->position;
  destination.rotateXY(mountingAngle);
  destination.y -= legOffset;

  float angleCoxa = atan((destination.x/destination.y))*180.0f/M_PI + ANGLEMAX/2.0f;

  float lengthLeg = sqrt((destination.x*destination.x) + (destination.y*destination.y));
  float lengthFemDes = sqrt((HEIGHT - destination.z) * (HEIGHT - destination.z) + (lengthLeg - COXA) * (lengthLeg - COXA));

  //a1 = cos⁻¹((Höhe - z0)/ lengthFemDes)
  float angleFemur1 = acos(((HEIGHT - destination.z) / lengthFemDes))*180.0f/M_PI;
  //a2 = cos⁻¹((FEMUR² + lengthFemDes² - Tibia²) / (2 * Femur * lengthFemdes))
  float angleFemur2 = acos((FEMUR*FEMUR + lengthFemDes*lengthFemDes - TIBIA*TIBIA) / (2*FEMUR*lengthFemDes))*180.0f/M_PI;
  float angleFemur = angleFemur1 + angleFemur2;

  float angleTibia = acos((FEMUR*FEMUR + TIBIA*TIBIA - lengthFemDes*lengthFemDes) / (2*FEMUR*TIBIA))*180.0f/M_PI;

  //Change angles depending on the position of the leg and the hardware configuration of the servos
  if(mountingAngle == 0 || mountingAngle == 62 || mountingAngle == 298) {
    angleFemur -= 45;
    angleTibia = 180 - angleTibia;
  } else {
    angleFemur = 225 - angleFemur;
  }

  #ifdef DEBUG
    PRINT(destination);
    PRINT(lengthLeg);
    PRINT(lengthFemDes);
    PRINT(angleFemur1);
    PRINT(angleFemur2);
    PRINT(angleCoxa);
    PRINT(angleFemur);
    PRINT(angleTibia);
  #endif

  setAngles(angleCoxa, angleFemur, angleTibia);
}
