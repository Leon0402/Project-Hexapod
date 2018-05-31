 #ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Controls.h"
#include "Leg.h"
#include "Servocontroller.h"
#include "Point.h"
#include "Servo.h"

#define FRONT_LEFT 0
#define MIDDLE_LEFT 1
#define REAR_LEFT 2
#define REAR_RIGHT 3
#define MIDDLE_RIGHT 4
#define FRONT_RIGHT 5

class Hexapod {
public:
    Hexapod()
    : servocontroller1(new Servocontroller()), servocontroller2(new Servocontroller()) {

        servocontroller1->begin();
        servocontroller1->setPWMFreq(50);

        servocontroller2->begin();
        servocontroller2->setPWMFreq(50);

        legs[FRONT_LEFT]   = new Leg(new Servo(servocontroller1,0),new Servo(servocontroller1,1,95,472),new Servo(servocontroller1,2,135,510),  8.5, (62*M_PI/180), Pointf(15,8,0));
        legs[MIDDLE_LEFT]  = new Leg(new Servo(servocontroller1,3, 130, 400),new Servo(servocontroller1,4,100,475),new Servo(servocontroller1,5,80,445), 6.5, (0*M_PI/180), Pointf(0,14,0));
        legs[REAR_LEFT]    = new Leg(new Servo(servocontroller1,6,100,450),new Servo(servocontroller1,7,132,492),new Servo(servocontroller1,8,110,475), 8.5, (298*M_PI/180), Pointf(-15,8,0));
        legs[REAR_RIGHT]   = new Leg(new Servo(servocontroller2,0,85,445),new Servo(servocontroller2,1,130,430),new Servo(servocontroller2,2,85,389), 8.5, (242*M_PI/180), Pointf(15,-8,0));
        legs[MIDDLE_RIGHT] = new Leg(new Servo(servocontroller2,3,125,415),new Servo(servocontroller2,4,90,454),new Servo(servocontroller2,5,89,445), 6.5, (180*M_PI/180),   Pointf(0,-14,0));
        legs[FRONT_RIGHT]  = new Leg(new Servo(servocontroller2,6,130,440),new Servo(servocontroller2,7,85,455),new Servo(servocontroller2,8,85 ,380), 8.5, (120*M_PI/180),  Pointf(-15,-8,0));
    }

    ~Hexapod() {
        for(Leg* leg: legs) {
            delete leg;
        }
        delete servocontroller1;
        delete servocontroller2;
    }

    void test() {
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,14,0));
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,16,0));
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,18,0));
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,20,0));
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,22,0));
        legs[MIDDLE_LEFT]->moveTo(Pointf(0,24,0));
    }

    void initServo() {
      legs[FRONT_RIGHT]->initServo();
    }

private:
    Servocontroller* servocontroller1;
    Servocontroller* servocontroller2;
    Leg* legs[6];
};
#endif //HEXAPOD_H
