#ifndef SERVO_H
#define SERVO_H

#include "Servocontroller.h"
#ifdef DEBUG
  #include <cstdint>
#else
  #include <inttypes.h>
#endif

#define ANGLEMAX 180

class Servo {
public:
  /*!
  @brief Instanstiates a Servo with its pin and its servMin and ServoMax value
  @param pin connected with this pin on the servocontroller
  @param servoMin Minimum pulse length out of 4096 (12-Bit PWM)
  @param servoMax Maximum pulse length out of 4096 (12-Bit PWM)
  @param angleMax maximum angle the servo can reach
  */
  Servo(uint8_t pin, uint16_t servoMin = 250, uint16_t servoMax = 450);

  /*!
  @brief return the corresponding "on Time" to the angle the servo should move to
  @return returns the "on Time" in 20ms/4096 ticks
  */
  uint16_t getOnTime() const;

  //getter and setter
  uint8_t getPin() const;

  /*!
  @brief Sets the angle of the servo
  @param angle Angle of the servo between 0 and  180 degrees
  */
  void setAngle(uint8_t angle);
  uint8_t getAngle() const;

private:
  uint8_t angle;

  const uint8_t pin;
  const uint16_t servoMin;
  const uint16_t servoMax;
};
#endif //SERVO_H
