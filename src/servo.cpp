#include "Servo.h"

#ifdef DEBUG
  #include <cstdint>
  #include <iostream>
#else
  #include <util/delay.h>
  #include <stdint.h>
#endif

Servo::Servo(Servocontroller& servocontroller, uint8_t pin, uint16_t servoMin, uint16_t servoMax)
: servocontroller(servocontroller), pin(pin), servoMin(servoMin), servoMax(servoMax) {}

void Servo::write(uint8_t angle) {
  //mapt die Winkel zur einer puslänge
  uint16_t off = ((angle * (servoMax-servoMin) * (uint32_t)2) / (float)ANGLEMAX + 1) / 2 + servoMin;

  #ifdef DEBUG
  std::cout << "angle: " << (unsigned int)angle << '\n';
  std::cout << "pin: " << (unsigned int)pin << '\n';
  std::cout << "off: " << off << '\n';
  #endif

  //übergibt Werte an Servocontroller
  servocontroller.setPWM(pin,0,off);

  #ifndef DEBUG
  _delay_ms(1000);
  #endif
}
