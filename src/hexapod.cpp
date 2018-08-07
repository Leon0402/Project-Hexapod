#include "Hexapod.h"

#include "util/Point.h"

Hexapod::Hexapod(Servocontroller& servocontroller1, Servocontroller& servocontroller2)
: servocontroller1 {servocontroller1}, servocontroller2 {servocontroller2},
  servos {
    Servo { 0, 105, 465}, Servo { 1,  95, 472}, Servo { 2, 135, 510},
    Servo { 5, 130, 400}, Servo { 6, 100, 475}, Servo { 7,  92, 445},
    Servo {13, 100, 450}, Servo {14, 132, 492}, Servo {15, 110, 475},
    Servo { 0,  85, 445}, Servo { 1, 130, 430}, Servo { 2,  85, 389},
    Servo { 5, 125, 415}, Servo { 6,  90, 454}, Servo { 7,  89, 445},
    Servo {13, 130, 440}, Servo {14,  85, 455}, Servo {15,  85, 380}
  },
  legs {
    Leg { servos[0],  servos[1],  servos[2], Pointf { 11.5f,  14.5f, 0.0f}, 8.5f,  62},
    Leg { servos[3],  servos[4],  servos[5], Pointf {  0.0f,  15.0f, 0.0f}, 6.5f,   0},
    Leg { servos[6],  servos[7],  servos[8], Pointf {-11.5f,  14.5f, 0.0f}, 8.5f, 298},
    Leg { servos[9], servos[10], servos[11], Pointf {-11.5f, -14.5f, 0.0f}, 8.5f, 242},
    Leg {servos[12], servos[13], servos[14], Pointf {  0.0f, -15.0f, 0.0f}, 6.5f, 180},
    Leg {servos[15], servos[16], servos[17], Pointf { 11.5f, -14.5f, 0.0f}, 8.5f, 118}
  } {}

void Hexapod::test() {
  this->legs[MIDDLE_LEFT].setPosition(Pointf { 0.0f,  20.0f, 0.0f });
  this->legs[MIDDLE_LEFT].updateAngles();
  servocontroller1.setPWM(this->legs[MIDDLE_LEFT].getCoxaPin(), 0,  this->legs[MIDDLE_LEFT].getCoxaOnTime());
  servocontroller1.setPWM(this->legs[MIDDLE_LEFT].getFemurPin(), 0,  this->legs[MIDDLE_LEFT].getFemurOnTime());
  servocontroller1.setPWM(this->legs[MIDDLE_LEFT].getTibiaPin(), 0,  this->legs[MIDDLE_LEFT].getTibiaOnTime());
}
