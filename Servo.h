#ifndef SERVO_H
#define SERVO_H

#include "Controls.h"

#include "Servocontroller.h"
#ifdef DEBUG
#include <cstdint>
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
    Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin = 250, uint16_t servoMax = 450);

    /*!
    @brief  Moves the servo to the given angle
    @param  angle Moves to this angle, should be between 0 and 180 degrees
    */
    void write(uint8_t angle);

private:
    Servocontroller servocontroller;
    const uint8_t pin;
    const uint16_t servoMin;
    const uint16_t servoMax;
};
#endif //SERVO_H
