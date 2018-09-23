#ifndef SERVO_H
#define SERVO_H

#include "Servocontroller.h"

#ifdef DEBUG
  #include <cstdint>
  #define F_CPU 16000000
#else
  #include <inttypes.h>
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include "Stream.h"
#endif

class Servo {

public:
  /*!
  @brief Instanstiates a Servo with its pin and its servMin and ServoMax value
  @param pin connected with this pin on the servocontroller
  @param servoMin Minimum pulse length out of 4096 (12-Bit PWM)
  @param servoMax Maximum pulse length out of 4096 (12-Bit PWM)
  */
  Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax);

  Servo(const Servo& servo) = delete;

  void update(uint32_t currentMillis);

  void move(float speed, float targetSpeed, float acceleration);
  void move(uint16_t time = 0);

  /*!
  @brief Sets the angle of the servo
  @param angle Angle of the servo between 0 and  180 degrees
  */
  void setAngle(uint8_t angle);


  static const uint8_t angleRange = 180;

private:
  uint16_t mapToPulseWidth(uint8_t angle);


  Servocontroller servocontroller;

  bool active = false;
  uint32_t lastUpdate = 0;

  float velocity = 0.0f;
  float targetVelocity = 0.0f;
  float acceleration = 0.0f;

  uint16_t pulseWidth;
  uint16_t destinationPulseWidth;

  const uint8_t pin;
  const uint16_t servoMin;
  const uint16_t servoMax;
  const uint16_t pulseWidthRange;
};
#endif //SERVO_H
