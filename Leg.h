#ifndef LEG_H
#define LEG_H

#include "Controls.h"
#include "Servo.h"
#include "Point.h"

#ifdef DEBUG
#include <cstdint>
#endif

#define COXA 2.5
#define FEMUR 8.5
#define TIBIA 12
#define HEIGHT 3 //from femur mounting point

class Leg {
public:
    /*!
    @brief  Instanstiates a Leg with it`s three servos and it's mounting angle and mounting point
    @param mountingangle The angle at which the servo is mounted
    @param defaultposition The default position of the foot
    */
    Leg(Servo& coxaServo, Servo& femurServo, Servo& tibiaServo, float legOffset, uint16_t mountingAngle, Pointf position);

    /*!
    @brief Moves the leg to the given position in global coordinates with inverse kinematics algorithms
    @param destination The destination given in the Hexapod`s coordinate system
    */
    void moveTo(const Pointf& destination);

private:
    void setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia);

    Servo coxaServo;
    Servo femurServo;
    Servo tibiaServo;

    float legOffset;
    uint16_t mountingAngle;

    Pointf position;
};

#endif //LEG_H
