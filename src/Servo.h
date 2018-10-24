#ifndef SERVO_H
#define SERVO_H

#include "Servocontroller.h"

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
  #define F_CPU 16000000
#endif

class Servo {

public:
  /*!
  @brief Instanstiates a Servo with the pin its attached to and its minimum and maximum pulse length
  @param pin attached to this pin on the servocontroller
  @param servoMin Minimum pulse length out of 4096 (12-Bit PWM)
  @param servoMax Maximum pulse length out of 4096 (12-Bit PWM)
  */
  Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax);

  /*!
  @brief This method updates the position of the servo (if its active). The Method should be called regularly through an ISR
  @param currentMillis The current system time use to determine the change of the position
  */
  void update(uint32_t currentMillis);

  /*!
  @brief Activates the servo if the destinationPulseWidth is different from the pulseWidth. It's also possible to
  move the servo with the given speed and acceleration
  @param speed The speed the servo should move with
  @param targetSpeed speed will be increased/decreased by the acceleration until it reached the target speed
  @param acceleration Increases/Decreases the speed
  */
  void move(float speed, float targetSpeed, float acceleration);

  /*!
  @brief Activates the servo if the destinationPulseWidth is different from the pulseWidth. It's also possible to give
  a time the movement should be done in. Speed will be calculated automatically.
  @param time The time the servo should need to reach his destinationPulseWidth in ms
  */
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

  uint16_t pulseWidth = 0;
  uint16_t destinationPulseWidth = 0;

  const uint8_t pin;
  const uint16_t servoMin;
  const uint16_t servoMax;
};
#endif //SERVO_H
