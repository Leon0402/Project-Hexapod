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
Leg::Leg(Servo* coxaservo, Servo* femurservo, Servo* tibiaservo, float legoffset, uint8_t mountingangle, Pointf defaultposition)
: coxaServo(coxaservo), femurServo(femurservo), tibiaServo(tibiaservo), legOffset(legoffset), mountingAngle(mountingangle), position(defaultposition) {
}

/*!
@brief deletes the Servos to avoid memory leaks
*/
Leg::~Leg() {
    delete coxaServo;
    delete femurServo;
    delete tibiaServo;
}

/*!
@brief Moves the leg to the given position in global coordinates with inverse kinematics algorithms
@param destination The destination given in the Hexapod`s coordinate system
*/
void Leg::moveTo(Pointf destination) {
    position = destination;
    destination.rotateXY(mountingAngle * (int16_t)-1);
    destination.y -= legOffset;
    //std::cout << "Destination: " << destination.x << "/" << destination. y << "/" << destination.z << '\n';

    uint8_t angleCoxa = atan((destination.x/(float)destination.y))*180/M_PI + ANGLEMAX/2 + 0.5;
    //std::cout << "angleCoxa: " << (unsigned int)angleCoxa <<  '\n';

    uint8_t lengthLeg = sqrt((destination.x*destination.x) + (destination.y*destination.y)) + 0.5;
    //std::cout << "lengthLeg: " << (unsigned int) lengthLeg << '\n';
    float lengthFemDes = sqrt((HEIGHT - destination.z) * (HEIGHT - destination.z) + (lengthLeg - COXA) * (lengthLeg - COXA));
    //std::cout << "lengthFemDes: " << lengthFemDes << '\n';

    uint8_t angleFemur1 = acos(((HEIGHT - destination.z) / lengthFemDes))*180/M_PI + 0.5;
    //std::cout << "AngleFemur1: " << (unsigned int)angleFemur1 << '\n';
    uint8_t angleFemur2 = acos((FEMUR*FEMUR + lengthFemDes*lengthFemDes - TIBIA*TIBIA) / (2*FEMUR*lengthFemDes))*180/M_PI + 0.5;
    //std::cout << "AngleFemur2: " << (unsigned int)angleFemur2 << '\n';
    uint8_t angleFemur = angleFemur1 + angleFemur2 + 0.5 - 45;
    //std::cout << "angleFemur: " << (unsigned int)angleFemur << '\n';
    uint8_t angleTibia = 180 - acos((FEMUR*FEMUR + TIBIA*TIBIA - lengthFemDes*lengthFemDes) / (2*FEMUR*TIBIA))*180/M_PI + 0.5;
    //std::cout << "angleTibia: " << (unsigned int)angleTibia << '\n';

#ifdef DEBUG
    std::cout << "angleCoxa: " << (unsigned int)angleCoxa << '\n';
    std::cout << "angleFemur: " << (unsigned int)angleFemur << '\n';
    std::cout << "angleTibia: " << (unsigned int)angleTibia << '\n';
#endif

    setAngles(angleCoxa, angleFemur, angleTibia);
}
/******************************************************************************************************************************************************/
void Leg::setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia) {
    coxaServo->write(angleCoxa);
    femurServo->write(angleFemur);
    tibiaServo->write(angleTibia);
}
