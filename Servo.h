#ifndef SERVO_H
#define SERVO_H

#include "Controls.h"

#include "Servocontroller.h"
#ifdef DEBUG
  #include <cstdint>
  #include <iostream>
#else
  #include <Arduino.h>
  #include <stdint.h>
#endif

#define ANGLEMAX 180

class Servo {
public:
    /*!
    @brief  Instanstiates a Servo with the pin and the servocontroller it's connected to
    @param  conPin connected with this pin on the servocontroller
    @param  conServocontroller connected with this servocontroller
    @param servoMin Minimum pulse length out of 4096 (12-Bit PWM)
    @param servoMax Maximum pulse length out of 4096 (12-Bit PWM)
    */
    Servo(const Servocontroller* conServocontroller = nullptr, uint8_t conPin = 42, uint16_t servomin = 150, uint16_t servomax = 450)
    : servocontroller(conServocontroller), pin(conPin), servoMin(servomin), servoMax(servomax) {
    }

    /*!
    @brief  Deletes the servocontroller to avoid memory leaks
    */
    ~Servo() {
        delete servocontroller;
    }

    /*!
    @brief  Moves the servo to the given angle
    @param  angle Moves to this angle, should be between 0 and 120 degrees
    */
    void write(uint8_t angle) {
        //mapt die Winkel zur einer puslänge
        uint16_t off = ((angle * (servoMax-servoMin) * (uint32_t)2) / (float)ANGLEMAX + 1) / 2 + servoMin;

        /*
        #ifdef DEBUG
        std::cout << "angle: " << (unsigned int)angle << '\n';
        std::cout << "pin: " << (unsigned int)pin << '\n';
        std::cout << "off: " << off << '\n';
        #endif*/

        //übergibt Werte an Servocontroller
        servocontroller->setPWM(pin,0,off);

        #ifndef DEBUG
        delay(1000);
        #endif
    }
private:
    const Servocontroller* servocontroller;
    uint8_t pin;
    uint16_t servoMin;
    uint16_t servoMax;
};
#endif //SERVO_H
