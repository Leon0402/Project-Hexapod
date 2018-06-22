#include "Leg.h"

#ifdef DEBUG
#include <cmath>
#include <iostream>
#else
#include <math.h>
#endif

/*!
@brief  Instanstiates a Leg with it`s three servos and it's mounting angle and mounting point
@param mountingangle The angle at which the servo is mounted
@param defaultposition The default position of the foot
*/
Leg::Leg(Servo& coxaServo, Servo& femurServo, Servo& tibiaServo, float legOffset, uint16_t mountingAngle, Pointf position)
: coxaServo(coxaServo), femurServo(femurServo), tibiaServo(tibiaServo), legOffset(legOffset), mountingAngle(mountingAngle), position(position) {
}

/*!
@brief Moves the leg to the given position in global coordinates with inverse kinematics algorithms
@param destination The destination given in the Hexapod`s coordinate system
*/
void Leg::moveTo(const Pointf& destination) {
    position = destination;
    position.rotateXY(mountingAngle);
    position.y -= legOffset;

    uint8_t angleCoxa = atan((position.x/(float)position.y))*180/M_PI + ANGLEMAX/2 + 0.5;

    uint8_t lengthLeg = sqrt((position.x*position.x) + (position.y*position.y)) + 0.5;
    float lengthFemDes = sqrt((HEIGHT - position.z) * (HEIGHT - position.z) + (lengthLeg - COXA) * (lengthLeg - COXA));

    uint8_t angleFemur1 = acos(((HEIGHT - position.z) / lengthFemDes))*180/M_PI + 0.5;
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
    std::cout << "angleCoxa: " << (unsigned int)angleCoxa << '\n';
    std::cout << "angleFemur: " << (unsigned int)angleFemur << '\n';
    std::cout << "angleTibia: " << (unsigned int)angleTibia << '\n' << '\n';
#endif

    setAngles(angleCoxa, angleFemur, angleTibia);
}
/******************************************************************************************************************************************************/
void Leg::setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
    coxaServo.write(angleCoxa);
    femurServo.write(angleFemur);
    tibiaServo.write(angleTibia);
}
